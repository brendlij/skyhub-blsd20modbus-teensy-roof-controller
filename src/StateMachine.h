#pragma once

#include <Arduino.h>
#include "BLSD20Driver.h"
#include "Config.h"
#include "DebouncedInput.h"
#include "MotionController.h"
#include "Types.h"

// Firmware layer: State Machine (Spec Abschnitt 4).
// The single source of truth (Spec Abschnitt 2 "Der Teensy ist die einzige
// Wahrheit"): owns Motion/Position/Zone/RoofMode, reads the safety-relevant
// physical sensors (limit + slowdown switches -- Spec Abschnitt 2 "Physische
// Sensoren besitzen höchste Priorität"), enforces the safety rules (Spec
// Abschnitt 10), runs homing, and drives the Motion Controller below it.
// Buttons and USB both funnel through requestOpen()/requestClose()/etc., so
// both control paths obey the same guards (Spec Abschnitt 2: "SkyHub darf
// niemals eigene Bewegungslogik implementieren").
class StateMachine
{
public:
    StateMachine(BLSD20Driver& driver, MotionController& motion);

    void begin();
    void tick(); // sensors, position/zone, homing, auto-slowdown, health, watchdog, boot/restart gates

    // Called by the Command Layer every tick with the live mode-switch
    // reading; enforces the Off lockout (Spec Abschnitt 10: Off force-stops
    // whatever is already running, rather than merely refusing new moves).
    void setMode(RoofMode mode);
    RoofMode mode() const { return _mode; }

    // Auto-mode / USB requests.
    bool requestOpen();
    bool requestClose();
    bool requestStop();
    bool requestHome();
    bool requestEnable();
    bool requestDisable();
    bool requestResetFault();
    bool requestRestart();
    bool requestFwUpdate();

    // Moves toward a target open-percentage instead of a limit switch.
    // Comfort feature built on top of the Hall-counter percentOpen()
    // readout -- requires isHomed() regardless of AUTO_REQUIRES_HOMING
    // (which only gates OPEN/CLOSE), since a percentage is meaningless
    // without calibration. The physical limit switches remain the hard
    // stop exactly as for a full OPEN/CLOSE; this only adds an earlier,
    // best-effort stop plus a slowdown band (RoofConfig::PERCENT_APPROACH_BAND)
    // before the target.
    bool requestMoveToPercent(uint8_t percent);

    // Manual (hold-to-run) moves -- continuous, so start/stop are separate
    // calls rather than a single request the way Auto's OPEN/CLOSE are.
    bool beginManualMove(Direction dir);
    void endManualMove();

    Motion motion() const { return _motion; }
    Position position() const { return _position; }
    Direction direction() const { return _motionCtl.direction(); }
    Zone zone() const { return _zone; }
    bool isHomed() const { return _homed; }
    int8_t percentOpen() const; // 0..100, or -1 if not homed -- comfort/UI only, see Hall Counter note below
    int8_t targetPercent() const { return _percentMoveActive ? _targetPercent : -1; } // -1 if no PERCENT move in progress
    int32_t hallCounter() const { return _driver.position(); }
    uint16_t currentSpeedRpm() const { return _driver.speed(); }
    uint16_t targetSpeedRpm() const { return _driver.targetSpeed(); }
    bool motorEnabled() const { return _motion != Motion::Disabled; }
    bool hasFault() const { return _motion == Motion::Fault; }
    const char* faultReason() const { return _faultReason; }
    const char* lastRejectReason() const { return _lastRejectReason; }

    bool modbusConnected() const { return _driver.modbusConnected(); }
    uint16_t motorCurrentMa() const { return _driver.current(); }
    uint16_t errorFlags() const { return _driver.errorFlags(); }
    uint8_t commFailCount() const { return _driver.commFailCount(); }
    float tempMcuC() const { return _driver.tempMcu(); }
    float tempMosfetC() const { return _driver.tempMosfet(); }
    float tempBrakeC() const { return _driver.tempBrake(); }
    bool isReapplyingConfig() const { return _configReapplyPending || _bootAutoRestartPending; }

    // True after a RESTART, until the roof next fully closes (which
    // re-zeroes the Hall counter and confirms it's back in sync).
    // Informational only -- OPEN/CLOSE are never blocked by this.
    bool calibStale() const { return _calibStale; }

    // Raw sensor states, for diagnostics (e.g. checking wiring/direction).
    bool limitOpenActive() const { return _limitOpen.isActive(); }
    bool limitCloseActive() const { return _limitClose.isActive(); }
    bool slowOpenActive() const { return _slowOpen.isActive(); }
    bool slowCloseActive() const { return _slowClose.isActive(); }
    bool hardStopActive() const { return _driver.hardStopActive(); }

    static const char* modeToString(RoofMode m);
    static const char* motionToString(Motion s);
    static const char* positionToString(Position p);
    static const char* directionToString(Direction d);
    static const char* zoneToString(Zone z);

private:
    enum class HomingStep : uint8_t
    {
        SeekClose,
        PauseBeforeOpen,
        SeekOpen
    };

    void loadCalibration();
    void saveCalibration();
    void refreshCalibrationAtLimit(bool closedEnd);

    void updateSensorInputs();
    void updatePositionAndZone();
    void updateHoming();
    void updateAutoMove();
    void checkMotorHealth();
    void updateWatchdogRelay();
    void updateBootAndReapplyGates();

    void enterFault(const char* reason);
    bool commandsBlocked() const; // true while boot/restart config reapply is pending

    BLSD20Driver& _driver;
    MotionController& _motionCtl;

    DebouncedInput _limitOpen, _limitClose, _slowOpen, _slowClose;

    RoofMode _mode = RoofMode::Auto;
    Motion _motion = Motion::Stopped;
    Position _position = Position::Unknown;
    Zone _zone = Zone::Unknown;
    const char* _faultReason = "";
    const char* _lastRejectReason = "";

    bool _homed = false;
    bool _calibStale = false;
    int32_t _closedPositionCounts = 0;
    int32_t _openPositionCounts = 0;

    HomingStep _homingStep = HomingStep::SeekClose;
    uint32_t _moveStartedMs = 0;
    uint32_t _homingPauseUntilMs = 0;
    uint32_t _stopRequestedMs = 0; // 0 = no soft-stop in progress, see requestStop()

    bool _percentMoveActive = false; // true while Opening/Closing toward a requestMoveToPercent() target
    int8_t _targetPercent = -1;

    // The watchdog relay pin starts LOW at boot (fail-safe), which briefly
    // breaks the hardware E-stop loop and latches EmergencyStop on the
    // BLSD20. This clears that automatically shortly after boot instead of
    // requiring a manual RESTART every time. No Fahrbefehle are accepted
    // while pending (Spec Abschnitt 12).
    bool _bootAutoRestartPending = true;
    uint32_t _bootAutoRestartAtMs = 0;

    bool _configReapplyPending = false;
    uint32_t _configReapplyAtMs = 0;

    Motion _preRestartMotion = Motion::Stopped; // state to return to once reapply finishes (Restart from Disabled stays Disabled)
};
