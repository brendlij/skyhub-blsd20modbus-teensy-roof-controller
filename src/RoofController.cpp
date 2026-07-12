#include "RoofController.h"
#include <EEPROM.h>

namespace
{
    constexpr int EEPROM_ADDR = 0;
    constexpr uint32_t CAL_MAGIC = 0x524F4631; // "ROF1"

    struct CalibrationBlob
    {
        uint32_t magic;
        int32_t closedPositionCounts;
        int32_t openPositionCounts;
    };
}

RoofController::RoofController(BLSD20Modbus& motor) : _motor(motor) {}

void RoofController::applyDefaultConfig()
{
    _motor.setControlMode(BLSD20ControlMode::Modbus);
    _motor.setRotationMode(BLSD20RotationMode::Continuous);
    _motor.useExternalSpeedInput(false);

    _motor.setPoleCount(RoofConfig::POLE_COUNT);
    _motor.setCurrentLimit(RoofConfig::CURRENT_LIMIT_MA);

    _motor.setAcceleration(RoofConfig::ACCEL);
    _motor.setDeceleration(RoofConfig::DECEL);

    _motor.enableAcceleration(false);
    _motor.enableDeceleration(false);
    _motor.enableFourQuadrantRegulation(true);
    _motor.enableAutoHold(false);
}

void RoofController::loadCalibration()
{
    CalibrationBlob blob;
    EEPROM.get(EEPROM_ADDR, blob);
    if (blob.magic == CAL_MAGIC)
    {
        _closedPositionCounts = blob.closedPositionCounts;
        _openPositionCounts = blob.openPositionCounts;
        _homed = true;
    }
}

void RoofController::saveCalibration()
{
    CalibrationBlob blob{CAL_MAGIC, _closedPositionCounts, _openPositionCounts};
    EEPROM.put(EEPROM_ADDR, blob);
}

void RoofController::begin()
{
    _btnOpen.begin(Pins::BTN_OPEN, RoofConfig::DEBOUNCE_MS);
    _btnStop.begin(Pins::BTN_STOP, RoofConfig::DEBOUNCE_MS);
    _btnClose.begin(Pins::BTN_CLOSE, RoofConfig::DEBOUNCE_MS);
    _modeSwitch.begin(Pins::MODE_SWITCH, RoofConfig::DEBOUNCE_MS);
    _limitOpen.begin(Pins::LIMIT_OPEN, RoofConfig::DEBOUNCE_MS);
    _limitClose.begin(Pins::LIMIT_CLOSE, RoofConfig::DEBOUNCE_MS);

    _mode = _modeSwitch.isActive() ? RoofMode::Manual : RoofMode::Auto;

    applyDefaultConfig();
    loadCalibration();

    _state = RoofState::Idle;
}

void RoofController::update()
{
    _btnOpen.update();
    _btnStop.update();
    _btnClose.update();
    _modeSwitch.update();
    _limitOpen.update();
    _limitClose.update();

    _mode = _modeSwitch.isActive() ? RoofMode::Manual : RoofMode::Auto;

    updatePositionPoll();

    if (_btnStop.wasPressed())
    {
        requestStop();
    }

    if (_mode == RoofMode::Auto)
    {
        if (_btnOpen.wasPressed()) requestOpen();
        if (_btnClose.wasPressed()) requestClose();
    }
    else
    {
        updateManualHold();
    }

    switch (_state)
    {
    case RoofState::Homing:
        updateHoming();
        break;
    case RoofState::Opening:
    case RoofState::Closing:
        if (_mode == RoofMode::Auto)
        {
            updateAutoMove();
            checkMotorHealth();
        }
        break;
    default:
        break;
    }
}

void RoofController::updateManualHold()
{
    if (_state == RoofState::Fault) return;

    if (_btnOpen.wasPressed() && _state == RoofState::Idle)
    {
        if (startMove(RoofConfig::DIR_OPEN, RoofConfig::SPEED_MANUAL))
            _state = RoofState::Opening;
    }
    else if (_btnOpen.wasReleased() && _state == RoofState::Opening)
    {
        stopMotor(false);
        _state = RoofState::Idle;
    }

    if (_btnClose.wasPressed() && _state == RoofState::Idle)
    {
        if (startMove(RoofConfig::DIR_CLOSE, RoofConfig::SPEED_MANUAL))
            _state = RoofState::Closing;
    }
    else if (_btnClose.wasReleased() && _state == RoofState::Closing)
    {
        stopMotor(false);
        _state = RoofState::Idle;
    }

    // Hard safety cutoff even while under manual (hold) control.
    if (_state == RoofState::Opening && _limitOpen.isActive())
    {
        stopMotor(false);
        _state = RoofState::Idle;
    }
    if (_state == RoofState::Closing && _limitClose.isActive())
    {
        stopMotor(false);
        _state = RoofState::Idle;
    }
}

void RoofController::updateAutoMove()
{
    if (millis() - _moveStartedMs > RoofConfig::MOVE_TIMEOUT_MS)
    {
        enterFault("move timeout");
        return;
    }

    int32_t range = abs(_openPositionCounts - _closedPositionCounts);
    int32_t marginCounts = (int32_t)(range * RoofConfig::SLOWDOWN_FRACTION);

    if (_state == RoofState::Opening)
    {
        if (_limitOpen.isActive())
        {
            stopMotor(false);
            _state = RoofState::Idle;
            return;
        }
        int32_t remaining = abs(_openPositionCounts - _lastPosition);
        if (!_slowdownApplied && remaining <= marginCounts)
        {
            _motor.setSpeed(RoofConfig::SPEED_AUTO_SLOW);
            _slowdownApplied = true;
        }
    }
    else if (_state == RoofState::Closing)
    {
        if (_limitClose.isActive())
        {
            stopMotor(false);
            _state = RoofState::Idle;
            return;
        }
        int32_t remaining = abs(_closedPositionCounts - _lastPosition);
        if (!_slowdownApplied && remaining <= marginCounts)
        {
            _motor.setSpeed(RoofConfig::SPEED_AUTO_SLOW);
            _slowdownApplied = true;
        }
    }
}

void RoofController::updateHoming()
{
    if (millis() - _moveStartedMs > RoofConfig::HOMING_TIMEOUT_MS)
    {
        enterFault("homing timeout");
        return;
    }

    if (_homingStep == HomingStep::SeekClose)
    {
        if (_limitClose.isActive())
        {
            stopMotor(false);
            _motor.resetPosition();
            _closedPositionCounts = 0;
            _homingStep = HomingStep::SeekOpen;
            _moveStartedMs = millis();
            if (!startMove(RoofConfig::DIR_OPEN, RoofConfig::SPEED_HOMING))
            {
                enterFault("homing: could not start open leg");
            }
        }
    }
    else // SeekOpen
    {
        if (_limitOpen.isActive())
        {
            stopMotor(false);
            _openPositionCounts = _motor.getPosition();

            int32_t range = abs(_openPositionCounts - _closedPositionCounts);
            if (range < RoofConfig::MIN_HOMING_RANGE_COUNTS)
            {
                enterFault("homing range too small");
                return;
            }

            _homed = true;
            saveCalibration();
            _state = RoofState::Idle;
        }
    }
}

void RoofController::updatePositionPoll()
{
    uint32_t now = millis();
    if (now - _lastPositionPollMs < RoofConfig::POSITION_POLL_MS) return;
    _lastPositionPollMs = now;

    _lastPosition = _motor.getPosition();
    _lastSpeed = _motor.getSpeed();
    _lastCurrent = _motor.getCurrent();

    if (_motor.lastResult() == ModbusResult::Success)
    {
        _commFailCount = 0;
    }
    else if (_commFailCount < 255)
    {
        _commFailCount++;
    }
}

void RoofController::checkMotorHealth()
{
    if (_motor.hasError())
    {
        enterFault("motor error flag set");
        return;
    }
    if (_commFailCount >= RoofConfig::MAX_COMM_FAILURES)
    {
        // Comms may already be down, so this stop command can itself fail;
        // enterFault() still moves us out of Opening/Closing locally.
        enterFault("modbus communication lost");
    }
}

void RoofController::enterFault(const char* reason)
{
    _faultReason = reason;
    _state = RoofState::Fault;
    stopMotor(true);
}

void RoofController::stopMotor(bool emergency)
{
    if (emergency)
        _motor.emergencyStop();
    else
        _motor.stop();
}

bool RoofController::startMove(BLSD20Direction dir, uint16_t speed)
{
    if (!_motor.setDirection(dir)) return false;
    if (!_motor.setSpeed(speed)) return false;
    if (!_motor.start()) return false;
    return true;
}

bool RoofController::requestOpen()
{
    if (_mode != RoofMode::Auto) return false;
    if (_state == RoofState::Opening) return true;
    if (_state != RoofState::Idle) return false;
    if (!_homed) return false;
    if (_limitOpen.isActive()) return false;

    if (!startMove(RoofConfig::DIR_OPEN, RoofConfig::SPEED_AUTO_FAST)) return false;
    _state = RoofState::Opening;
    _slowdownApplied = false;
    _moveStartedMs = millis();
    return true;
}

bool RoofController::requestClose()
{
    if (_mode != RoofMode::Auto) return false;
    if (_state == RoofState::Closing) return true;
    if (_state != RoofState::Idle) return false;
    if (!_homed) return false;
    if (_limitClose.isActive()) return false;

    if (!startMove(RoofConfig::DIR_CLOSE, RoofConfig::SPEED_AUTO_FAST)) return false;
    _state = RoofState::Closing;
    _slowdownApplied = false;
    _moveStartedMs = millis();
    return true;
}

bool RoofController::requestStop()
{
    stopMotor(false);
    _state = RoofState::Idle;
    _faultReason = "";
    return true;
}

bool RoofController::requestHome()
{
    if (_state != RoofState::Idle) return false;

    _homed = false;
    _homingStep = HomingStep::SeekClose;
    _moveStartedMs = millis();

    if (!startMove(RoofConfig::DIR_CLOSE, RoofConfig::SPEED_HOMING)) return false;
    _state = RoofState::Homing;
    return true;
}

bool RoofController::requestSaveSettings()
{
    if (_state != RoofState::Idle) return false;
    bool ok = _motor.saveSettings();
    ok = _motor.restart() && ok;
    return ok;
}

int8_t RoofController::percentOpen() const
{
    if (!_homed) return -1;
    int32_t range = _openPositionCounts - _closedPositionCounts;
    if (range == 0) return -1;

    float pct = 100.0f * (float)(_lastPosition - _closedPositionCounts) / (float)range;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (int8_t)(pct + 0.5f);
}

const char* RoofController::modeToString(RoofMode m)
{
    switch (m)
    {
    case RoofMode::Auto: return "AUTO";
    case RoofMode::Manual: return "MANUAL";
    }
    return "?";
}

const char* RoofController::stateToString(RoofState s)
{
    switch (s)
    {
    case RoofState::Uninitialized: return "UNINITIALIZED";
    case RoofState::Idle: return "IDLE";
    case RoofState::Homing: return "HOMING";
    case RoofState::Opening: return "OPENING";
    case RoofState::Closing: return "CLOSING";
    case RoofState::Fault: return "FAULT";
    }
    return "?";
}
