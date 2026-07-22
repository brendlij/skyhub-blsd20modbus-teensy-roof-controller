#include "BLSD20Driver.h"

BLSD20Driver::BLSD20Driver(BLSD20Modbus& motor) : _motor(motor) {}

void BLSD20Driver::begin()
{
    applyConfig();
}

void BLSD20Driver::applyConfig()
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

bool BLSD20Driver::startMove(Direction dir, uint16_t speedRpm)
{
    if (dir == Direction::None) return false;
    BLSD20Direction motorDir = (dir == Direction::Opening) ? RoofConfig::DIR_OPEN : RoofConfig::DIR_CLOSE;
    if (!_motor.setDirection(motorDir)) return false;
    if (!_motor.setSpeed(speedRpm)) return false;
    if (!_motor.start()) return false;
    _targetSpeed = speedRpm;
    return true;
}

bool BLSD20Driver::setSpeed(uint16_t speedRpm)
{
    if (!_motor.setSpeed(speedRpm)) return false;
    _targetSpeed = speedRpm;
    return true;
}

bool BLSD20Driver::stop()
{
    return _motor.stop();
}

bool BLSD20Driver::emergencyStop()
{
    return _motor.emergencyStop();
}

bool BLSD20Driver::resetPositionCounter()
{
    return _motor.resetPosition();
}

bool BLSD20Driver::saveAndRestart()
{
    bool ok = _motor.saveSettings();
    return _motor.restart() && ok;
}

// Reboots the drive without persisting anything, which also clears whatever
// is currently latched in its ERROR register (e.g. after an
// emergencyStop()/HARD_STOP). Since nothing was saved, the reboot reverts to
// the drive's last-saved flash config, so the caller must re-apply the
// runtime config (applyConfig()/begin()) once it's back up.
bool BLSD20Driver::restart()
{
    return _motor.restart();
}

void BLSD20Driver::pollTelemetry()
{
    uint32_t now = millis();
    if (now - _lastTelemetryPollMs < RoofConfig::POSITION_POLL_MS) return;
    _lastTelemetryPollMs = now;

    _position = _motor.getPosition();
    _speed = _motor.getSpeed();
    _current = _motor.getCurrent();
    _errorFlags = _motor.getErrorFlags();
    _motorStatus = _motor.getStatus();
    _targetSpeed = _motor.getTargetSpeed();
    _lastHasErrorCall = _motor.hasError();

    if (_motor.lastResult() == ModbusResult::Success)
    {
        _commFailCount = 0;
    }
    else if (_commFailCount < 255)
    {
        _commFailCount++;
    }

    // Informational only: the hardware E-stop loop trips BLSD20 HARD_STOP
    // directly, without Teensy involvement. Doesn't affect _commFailCount.
    _hardStopInput = _motor.readHardStopInput();
}

void BLSD20Driver::pollTemperature()
{
    uint32_t now = millis();
    if (now - _lastTempPollMs < RoofConfig::TEMP_POLL_MS) return;
    _lastTempPollMs = now;

    _tempMcu = _motor.getTemperatureMCU();
    _tempMosfet = _motor.getTemperatureMosfet();
    _tempBrake = _motor.getTemperatureBrake();
}
