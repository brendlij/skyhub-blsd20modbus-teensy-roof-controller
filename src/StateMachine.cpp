#include "StateMachine.h"
#include <EEPROM.h>

namespace
{
    constexpr int EEPROM_ADDR = 0;
    constexpr uint32_t CAL_MAGIC = 0x524F4632; // "ROF2"

    struct CalibrationBlob
    {
        uint32_t magic;
        int32_t closedPositionCounts;
        int32_t openPositionCounts;
    };
}

StateMachine::StateMachine(BLSD20Driver& driver, MotionController& motion) : _driver(driver), _motionCtl(motion) {}

void StateMachine::loadCalibration()
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

void StateMachine::saveCalibration()
{
    CalibrationBlob blob{CAL_MAGIC, _closedPositionCounts, _openPositionCounts};
    EEPROM.put(EEPROM_ADDR, blob);
}

// Called every time a limit switch is actually reached, not just during an
// explicit HOME run. This lets the Hall-counter calibration self-heal after
// a BLSD20 restart resets its Hall counter: the next ordinary full
// open/close naturally re-syncs that endpoint, without requiring a
// dedicated re-home. Comfort-only (Spec Abschnitt 8) -- never affects
// Position/Zone/safety, which come solely from the limit/slow switches.
void StateMachine::refreshCalibrationAtLimit(bool closedEnd)
{
    if (closedEnd)
    {
        _driver.resetPositionCounter();
        if (_closedPositionCounts != 0)
        {
            _closedPositionCounts = 0;
            saveCalibration();
        }
        _calibStale = false;
    }
    else if (!_calibStale)
    {
        int32_t pos = _driver.position();
        if (abs(pos - _openPositionCounts) > RoofConfig::CALIBRATION_DRIFT_TOLERANCE_COUNTS)
        {
            _openPositionCounts = pos;
            saveCalibration();
        }
    }
}

void StateMachine::begin()
{
    _limitOpen.begin(Pins::LIMIT_OPEN, RoofConfig::DEBOUNCE_MS);
    _limitClose.begin(Pins::LIMIT_CLOSE, RoofConfig::DEBOUNCE_MS);
    _slowOpen.begin(Pins::SLOW_OPEN, RoofConfig::DEBOUNCE_MS);
    _slowClose.begin(Pins::SLOW_CLOSE, RoofConfig::DEBOUNCE_MS);
    _rain.begin(Pins::RAIN_SENSOR, RoofConfig::DEBOUNCE_MS, RoofConfig::RAIN_SENSOR_INVERTED);

    pinMode(Pins::MODBUS_WATCHDOG_RELAY, OUTPUT);
    digitalWrite(Pins::MODBUS_WATCHDOG_RELAY, LOW); // fail-safe until comms are proven healthy

    loadCalibration();
    updatePositionAndZone(); // initial Position/Zone from live limit switches, before any move accepted

    _motion = Motion::Stopped;
    _bootAutoRestartAtMs = millis() + RoofConfig::BOOT_AUTO_RESTART_DELAY_MS;
}

void StateMachine::updateSensorInputs()
{
    _limitOpen.update();
    _limitClose.update();
    _slowOpen.update();
    _slowClose.update();
    _rain.update();
}

// Spec Abschnitt 6: Zone kommt ausschließlich aus den vier Endlagen-/
// Slowdown-Schaltern. SLOW_OPEN/SLOW_CLOSE are point sensors (inductive
// proximity switches), so the zone they imply is latched on the edge and
// combined with which way the motor is currently commanded to move
// (Direction, from the Motion Controller) to know which side of the switch
// we ended up on. It otherwise holds its last value across stops, since the
// switch itself isn't continuously active away from that one point.
void StateMachine::updatePositionAndZone()
{
    if (_limitOpen.isActive()) _position = Position::Open;
    else if (_limitClose.isActive()) _position = Position::Closed;
    else _position = Position::Unknown;

    if (_limitOpen.isActive())
    {
        _zone = Zone::OpenSlow;
        return;
    }
    if (_limitClose.isActive())
    {
        _zone = Zone::CloseSlow;
        return;
    }

    Direction dir = _motionCtl.direction();
    if (_slowOpen.wasPressed() && dir == Direction::Opening) _zone = Zone::OpenSlow;
    if (_slowOpen.wasReleased() && dir == Direction::Closing) _zone = Zone::Main;
    if (_slowClose.wasPressed() && dir == Direction::Closing) _zone = Zone::CloseSlow;
    if (_slowClose.wasReleased() && dir == Direction::Opening) _zone = Zone::Main;
}

void StateMachine::updateAutoMove()
{
    if (millis() - _moveStartedMs > RoofConfig::MOVE_TIMEOUT_MS)
    {
        enterFault("move timeout");
        return;
    }

    if (_motion == Motion::Opening && _position == Position::Open)
    {
        _motionCtl.commandStop(false);
        refreshCalibrationAtLimit(false);
        _motion = Motion::Stopped;
        _percentMoveActive = false;
        return;
    }
    if (_motion == Motion::Closing && _position == Position::Closed)
    {
        _motionCtl.commandStop(false);
        refreshCalibrationAtLimit(true);
        _motion = Motion::Stopped;
        _percentMoveActive = false;
        return;
    }

    // PERCENT-target stop: best-effort, Hall-counter-derived -- the limit
    // switch checks above always take priority and remain the real safety
    // stop (Spec Abschnitt 8: Hall counter is comfort-only).
    bool approachSlowdown = false;
    if (_percentMoveActive)
    {
        int8_t current = percentOpen();
        bool reached = (_motion == Motion::Opening) ? (current >= _targetPercent) : (current <= _targetPercent);
        if (reached)
        {
            _motionCtl.commandStop(false);
            _motion = Motion::Stopped;
            _percentMoveActive = false;
            return;
        }
        approachSlowdown = abs((int)current - (int)_targetPercent) <= RoofConfig::PERCENT_APPROACH_BAND;
    }

    _motionCtl.update(_zone, approachSlowdown);
}

void StateMachine::updateHoming()
{
    if (millis() - _moveStartedMs > RoofConfig::HOMING_TIMEOUT_MS)
    {
        enterFault("homing timeout");
        return;
    }

    if (_homingStep == HomingStep::SeekClose)
    {
        if (_position == Position::Closed)
        {
            _motionCtl.commandStop(false);
            _driver.resetPositionCounter();
            _closedPositionCounts = 0;
            _calibStale = false;
            saveCalibration();
            _homingStep = HomingStep::PauseBeforeOpen;
            _homingPauseUntilMs = millis() + RoofConfig::HOMING_LEG_PAUSE_MS;
        }
    }
    else if (_homingStep == HomingStep::PauseBeforeOpen)
    {
        if (millis() >= _homingPauseUntilMs)
        {
            _moveStartedMs = millis();
            if (_motionCtl.commandHomingMove(Direction::Opening))
            {
                _homingStep = HomingStep::SeekOpen;
            }
            else
            {
                enterFault("homing: could not start open leg");
            }
        }
    }
    else // SeekOpen
    {
        if (_position == Position::Open)
        {
            _motionCtl.commandStop(false);
            _openPositionCounts = _driver.position();

            int32_t range = abs(_openPositionCounts - _closedPositionCounts);
            if (range < RoofConfig::MIN_HOMING_RANGE_COUNTS)
            {
                enterFault("homing range too small");
                return;
            }

            _homed = true;
            saveCalibration();
            _motion = Motion::Stopped;
        }
    }
}

// Keeps the watchdog relay energized only while Modbus comms are healthy.
// Its contact sits in series inside the hardware E-stop loop, so a comm
// loss -- or the Teensy itself crashing/resetting, since the pin defaults
// LOW at boot -- breaks the loop and hard-stops the motor independently of
// any command the Teensy would otherwise have to send over Modbus.
void StateMachine::updateWatchdogRelay()
{
    digitalWrite(Pins::MODBUS_WATCHDOG_RELAY, _driver.modbusConnected() ? HIGH : LOW);
}

void StateMachine::checkMotorHealth()
{
    if (_driver.hasError())
    {
        enterFault("motor error flag set");
        return;
    }
    if (_driver.commFailCount() >= RoofConfig::MAX_COMM_FAILURES)
    {
        enterFault("modbus communication lost");
    }
}

void StateMachine::enterFault(const char* reason)
{
    _faultReason = reason;
    _motion = Motion::Fault;
    _motionCtl.commandStop(true);
}

bool StateMachine::commandsBlocked() const
{
    return _bootAutoRestartPending || _configReapplyPending;
}

void StateMachine::updateBootAndReapplyGates()
{
    if (_bootAutoRestartPending && millis() >= _bootAutoRestartAtMs)
    {
        _bootAutoRestartPending = false;
        requestRestart();
    }
    if (_configReapplyPending && millis() >= _configReapplyAtMs)
    {
        _configReapplyPending = false;
        _driver.begin(); // re-apply config now that the drive finished rebooting
    }
}

void StateMachine::tick()
{
    updateSensorInputs();
    updatePositionAndZone();
    updateBootAndReapplyGates();
    updateWatchdogRelay();

    // Hard safety cutoff even under manual (hold) control, if enabled.
    if (RoofConfig::MANUAL_RESPECTS_LIMIT_SWITCHES && _mode == RoofMode::Manual)
    {
        if (_motion == Motion::Opening && _position == Position::Open)
        {
            _motionCtl.commandStop(false);
            refreshCalibrationAtLimit(false);
            _motion = Motion::Stopped;
        }
        if (_motion == Motion::Closing && _position == Position::Closed)
        {
            _motionCtl.commandStop(false);
            refreshCalibrationAtLimit(true);
            _motion = Motion::Stopped;
        }
    }

    switch (_motion)
    {
    case Motion::Stopped:
    case Motion::Disabled:
        // Also catches a hardware E-stop trip while sitting idle -- the
        // BLSD20 latches EmergencyStop in its ERROR register regardless of
        // what the Teensy was doing, so this is what promotes that into a
        // visible Fault (red LED) instead of silently staying Stopped.
        checkMotorHealth();
        break;
    case Motion::Homing:
        updateHoming();
        checkMotorHealth();
        break;
    case Motion::Opening:
    case Motion::Closing:
        if (_mode == RoofMode::Auto)
        {
            updateAutoMove();
        }
        if ((_motion == Motion::Opening || _motion == Motion::Closing) &&
            _stopRequestedMs != 0 &&
            millis() - _stopRequestedMs >= RoofConfig::STOP_SLOWDOWN_DELAY_MS)
        {
            _motionCtl.commandStop(false);
            _motion = Motion::Stopped;
            _stopRequestedMs = 0;
        }
        checkMotorHealth();
        break;
    case Motion::Fault:
    default:
        break;
    }
}

void StateMachine::setMode(RoofMode mode)
{
    _mode = mode;
    // Lockout: force-stop any move that was already running rather than
    // just refusing new ones, so turning the switch to Off is an
    // immediate, unconditional halt (Spec Abschnitt 10).
    if (_mode == RoofMode::Off &&
        (_motion == Motion::Opening || _motion == Motion::Closing || _motion == Motion::Homing))
    {
        _motionCtl.commandStop(false);
        _motion = Motion::Stopped;
        _stopRequestedMs = 0;
    }
}

bool StateMachine::requestOpen()
{
    _lastRejectReason = "";
    if (commandsBlocked()) { _lastRejectReason = "booting"; return false; }
    if (_mode != RoofMode::Auto) { _lastRejectReason = "wrong_mode"; return false; }
    // Already opening -- e.g. overriding an in-progress PERCENT move -- just
    // drop the target and continue on to the full limit instead of stopping
    // early.
    if (_motion == Motion::Opening) { _percentMoveActive = false; return true; }
    // Reversing straight into a still-decelerating (or still-moving) motor
    // isn't reliable on this hardware -- require a full stop before
    // accepting the opposite direction. Callers should STOP and wait for
    // Stopped rather than expect an immediate reversal.
    if (_motion != Motion::Stopped)
    {
        _lastRejectReason = (_stopRequestedMs != 0) ? "stopping" : "busy";
        return false;
    }
    if (RoofConfig::AUTO_REQUIRES_HOMING && !_homed) { _lastRejectReason = "not_homed"; return false; }
    if (_position == Position::Open) { _lastRejectReason = "limit_open_active"; return false; }
    if (_driver.hasError())
    {
        enterFault("motor error flag set");
        _lastRejectReason = "motor_error";
        return false;
    }

    if (!_motionCtl.commandAutoMove(Direction::Opening, _zone))
    {
        _lastRejectReason = "motor_comm_error";
        return false;
    }
    _motion = Motion::Opening;
    _stopRequestedMs = 0;
    _moveStartedMs = millis();
    return true;
}

bool StateMachine::requestClose()
{
    _lastRejectReason = "";
    if (commandsBlocked()) { _lastRejectReason = "booting"; return false; }
    if (_mode != RoofMode::Auto) { _lastRejectReason = "wrong_mode"; return false; }
    // Already closing -- e.g. overriding an in-progress PERCENT move -- just
    // drop the target and continue on to the full limit instead of stopping
    // early.
    if (_motion == Motion::Closing) { _percentMoveActive = false; return true; }
    if (_motion != Motion::Stopped)
    {
        _lastRejectReason = (_stopRequestedMs != 0) ? "stopping" : "busy";
        return false;
    }
    if (RoofConfig::AUTO_REQUIRES_HOMING && !_homed) { _lastRejectReason = "not_homed"; return false; }
    if (_position == Position::Closed) { _lastRejectReason = "limit_close_active"; return false; }
    if (_driver.hasError())
    {
        enterFault("motor error flag set");
        _lastRejectReason = "motor_error";
        return false;
    }

    if (!_motionCtl.commandAutoMove(Direction::Closing, _zone))
    {
        _lastRejectReason = "motor_comm_error";
        return false;
    }
    _motion = Motion::Closing;
    _stopRequestedMs = 0;
    _moveStartedMs = millis();
    return true;
}

bool StateMachine::requestMoveToPercent(uint8_t percent)
{
    _lastRejectReason = "";
    if (percent > 100) { _lastRejectReason = "invalid_percent"; return false; }
    if (commandsBlocked()) { _lastRejectReason = "booting"; return false; }
    if (_mode != RoofMode::Auto) { _lastRejectReason = "wrong_mode"; return false; }
    // Unconditional -- unlike OPEN/CLOSE's RoofConfig::AUTO_REQUIRES_HOMING
    // gate, a percentage is meaningless without calibration.
    if (!_homed) { _lastRejectReason = "not_homed"; return false; }
    if (_motion != Motion::Stopped)
    {
        _lastRejectReason = (_stopRequestedMs != 0) ? "stopping" : "busy";
        return false;
    }
    if (_driver.hasError())
    {
        enterFault("motor error flag set");
        _lastRejectReason = "motor_error";
        return false;
    }

    int8_t current = percentOpen();
    if (current == (int8_t)percent) return true; // already there

    Direction dir = (percent > (uint8_t)current) ? Direction::Opening : Direction::Closing;
    if (dir == Direction::Opening && _position == Position::Open) { _lastRejectReason = "limit_open_active"; return false; }
    if (dir == Direction::Closing && _position == Position::Closed) { _lastRejectReason = "limit_close_active"; return false; }

    if (!_motionCtl.commandAutoMove(dir, _zone))
    {
        _lastRejectReason = "motor_comm_error";
        return false;
    }
    _motion = (dir == Direction::Opening) ? Motion::Opening : Motion::Closing;
    _percentMoveActive = true;
    _targetPercent = (int8_t)percent;
    _stopRequestedMs = 0;
    _moveStartedMs = millis();
    return true;
}

// STOP besitzt höchste Priorität (Spec Abschnitt 10). A running Auto move
// gets one graceful soft-stop step (slow down, then actually stop --
// completed in tick()); every other state stops the motor immediately.
bool StateMachine::requestStop()
{
    _percentMoveActive = false;
    if (_motion == Motion::Opening || _motion == Motion::Closing)
    {
        if (!RoofConfig::STOP_USES_SOFT_SLOWDOWN)
        {
            _motionCtl.commandStop(false);
            _motion = Motion::Stopped;
            _stopRequestedMs = 0;
            return true;
        }
        if (_stopRequestedMs == 0)
        {
            _motionCtl.beginSoftStop();
            _stopRequestedMs = millis();
        }
        return true;
    }

    _motionCtl.commandStop(false);
    if (_motion != Motion::Fault && _motion != Motion::Disabled)
    {
        _motion = Motion::Stopped;
    }
    return true;
}

bool StateMachine::requestHome()
{
    if (commandsBlocked()) return false;
    if (_mode == RoofMode::Off) return false;
    if (_motion != Motion::Stopped) return false;
    if (_driver.hasError())
    {
        enterFault("motor error flag set");
        return false;
    }

    _homed = false;
    _homingStep = HomingStep::SeekClose;
    _moveStartedMs = millis();

    if (!_motionCtl.commandHomingMove(Direction::Closing)) return false;
    _motion = Motion::Homing;
    return true;
}

bool StateMachine::requestEnable()
{
    if (_motion != Motion::Disabled) return false;
    _motion = Motion::Stopped;
    return true;
}

// Disabled: nur möglich wenn Motion = Stopped (Spec Abschnitt 10).
bool StateMachine::requestDisable()
{
    if (_motion != Motion::Stopped) return false;
    _motionCtl.commandStop(false);
    _motion = Motion::Disabled;
    return true;
}

// Clears a latched Fault by rebooting the drive (which also clears its
// latched ERROR register) and re-applying the runtime config once it's
// back up.
bool StateMachine::requestResetFault()
{
    if (_motion != Motion::Fault) return false;

    bool ok = _driver.restart();
    _faultReason = "";
    _motion = Motion::Stopped;
    _calibStale = true;
    _configReapplyPending = true;
    _configReapplyAtMs = millis() + RoofConfig::RESTART_REAPPLY_DELAY_MS;
    return ok;
}

// Deliberate drive reboot outside of a fault (e.g. after a wiring change).
// Reverts the drive to its last-saved flash config, so the runtime config
// is re-applied once the reboot delay elapses (updateBootAndReapplyGates()).
bool StateMachine::requestRestart()
{
    if (_motion != Motion::Stopped && _motion != Motion::Disabled && _motion != Motion::Fault) return false;
    if (!_driver.restart()) return false;

    _preRestartMotion = (_motion == Motion::Fault) ? Motion::Stopped : _motion;
    _faultReason = "";
    _calibStale = true;
    _configReapplyPending = true;
    _configReapplyAtMs = millis() + RoofConfig::RESTART_REAPPLY_DELAY_MS;
    _motion = _preRestartMotion;
    return true;
}

// Firmware updates aren't part of the firmware itself (Spec Abschnitt 14):
// this only drops the Teensy into its USB bootloader so SkyHub or
// teensy_loader_cli can flash new firmware; the flashing itself is external.
bool StateMachine::requestFwUpdate()
{
    if (_motion == Motion::Opening || _motion == Motion::Closing || _motion == Motion::Homing) return false;
#if defined(TEENSYDUINO)
    _reboot_Teensyduino_(); // declared by the Teensy core (core_pins.h)
#endif
    return true;
}

bool StateMachine::beginManualMove(Direction dir)
{
    if (commandsBlocked()) return false;
    if (_mode != RoofMode::Manual) return false;
    if (_motion != Motion::Stopped) return false;
    if (RoofConfig::MANUAL_RESPECTS_LIMIT_SWITCHES)
    {
        if (dir == Direction::Opening && _position == Position::Open) return false;
        if (dir == Direction::Closing && _position == Position::Closed) return false;
    }
    if (!_motionCtl.commandManualMove(dir)) return false;
    _motion = (dir == Direction::Opening) ? Motion::Opening : Motion::Closing;
    return true;
}

void StateMachine::endManualMove()
{
    if (_motion == Motion::Opening || _motion == Motion::Closing)
    {
        _motionCtl.commandStop(false);
        _motion = Motion::Stopped;
    }
}

int8_t StateMachine::percentOpen() const
{
    if (!_homed) return -1;
    int32_t range = _openPositionCounts - _closedPositionCounts;
    if (range == 0) return -1;

    float pct = 100.0f * (float)(_driver.position() - _closedPositionCounts) / (float)range;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (int8_t)(pct + 0.5f);
}

const char* StateMachine::modeToString(RoofMode m)
{
    switch (m)
    {
    case RoofMode::Auto: return "AUTO";
    case RoofMode::Manual: return "MANUAL";
    case RoofMode::Off: return "OFF";
    }
    return "?";
}

const char* StateMachine::motionToString(Motion s)
{
    switch (s)
    {
    case Motion::Stopped: return "STOPPED";
    case Motion::Opening: return "OPENING";
    case Motion::Closing: return "CLOSING";
    case Motion::Homing: return "HOMING";
    case Motion::Fault: return "FAULT";
    case Motion::Disabled: return "DISABLED";
    }
    return "?";
}

const char* StateMachine::positionToString(Position p)
{
    switch (p)
    {
    case Position::Unknown: return "UNKNOWN";
    case Position::Open: return "OPEN";
    case Position::Closed: return "CLOSED";
    }
    return "?";
}

const char* StateMachine::directionToString(Direction d)
{
    switch (d)
    {
    case Direction::None: return "NONE";
    case Direction::Opening: return "OPENING";
    case Direction::Closing: return "CLOSING";
    }
    return "?";
}

const char* StateMachine::zoneToString(Zone z)
{
    switch (z)
    {
    case Zone::Unknown: return "UNKNOWN";
    case Zone::OpenSlow: return "OPEN_SLOW";
    case Zone::Main: return "MAIN";
    case Zone::CloseSlow: return "CLOSE_SLOW";
    }
    return "?";
}
