#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "Config.h"
#include "DebouncedInput.h"

enum class RoofMode : uint8_t
{
    Auto,
    Manual,
    Off // 3-position switch center: lockout, no OPEN/CLOSE/HOME accepted
};

enum class RoofState : uint8_t
{
    Uninitialized,
    Idle,
    Homing,
    Opening,
    Closing,
    Fault,
    Restarting
};

// Owns all safety-relevant I/O (buttons, limit switches) and the motor.
// Buttons and the USB serial link both funnel through the same
// requestOpen/requestClose/requestStop/requestHome entry points, so both
// control paths obey the same guards.
class RoofController
{
public:
    explicit RoofController(BLSD20Modbus& motor);

    void begin();
    void update();

    bool requestOpen();
    bool requestClose();
    bool requestStop();
    bool requestHome();
    bool requestSaveSettings(); // one-time provisioning: saveSettings() + restart()
    bool requestRestart();      // reboot the BLSD20 only, then re-apply the runtime config

    RoofMode mode() const { return _mode; }
    RoofState state() const { return _state; }
    bool isHomed() const { return _homed; }
    int8_t percentOpen() const; // 0..100, or -1 if not homed
    int32_t positionCounts() const { return _lastPosition; }
    uint16_t motorSpeedRpm() const { return _lastSpeed; }
    uint16_t motorCurrentMa() const { return _lastCurrent; }
    bool hasFault() const { return _state == RoofState::Fault; }
    const char* faultReason() const { return _faultReason; }

    // Why the most recent requestOpen()/requestClose() call returned false,
    // e.g. "busy" (moving the other way), "stopping" (still coasting through
    // a soft-stop), "not_homed", "limit_open_active"/"limit_close_active",
    // "wrong_mode", "motor_error". Empty string if the last call succeeded.
    const char* lastRejectReason() const { return _lastRejectReason; }
    uint16_t lastErrorFlags() const { return _lastErrorFlags; }
    uint8_t commFailCount() const { return _commFailCount; }

    float tempMcuC() const { return _lastTempMcu; }
    float tempMosfetC() const { return _lastTempMosfet; }
    float tempBrakeC() const { return _lastTempBrake; }

    // True after a RESTART, until the roof next fully closes (which
    // re-zeroes the controller's position counter and confirms it's back in
    // sync). Informational only -- OPEN/CLOSE are never blocked by this.
    bool calibStale() const { return _calibStale; }

    // Best-effort read of the BLSD20's own HARD_STOP input (DI_HARD_STOP).
    // Purely informational -- the hardware E-stop loop already stops the
    // motor without any help from the Teensy; this just lets that trip be
    // reported upstream (e.g. to the Pi5).
    bool hardStopActive() const { return _lastHardStopInput; }

    // Raw sensor states, for diagnostics (e.g. checking wiring/direction).
    bool btnOpenActive() const { return _btnOpen.isActive(); }
    bool btnStopActive() const { return _btnStop.isActive(); }
    bool btnCloseActive() const { return _btnClose.isActive(); }
    bool modeSwitchAutoActive() const { return _modeSwitchAuto.isActive(); }
    bool modeSwitchManualActive() const { return _modeSwitchManual.isActive(); }
    bool limitOpenActive() const { return _limitOpen.isActive(); }
    bool limitCloseActive() const { return _limitClose.isActive(); }
    bool slowOpenActive() const { return _slowOpen.isActive(); }
    bool slowCloseActive() const { return _slowClose.isActive(); }

    // Direction as last commanded by the software (Forward/Backward, per
    // RoofConfig::DIR_OPEN/DIR_CLOSE) vs. what the BLSD20 itself reports it's
    // actually doing -- compare the two to spot a reversed motor/wiring.
    BLSD20Direction commandedDirection() const { return _lastCommandedDir; }
    BLSD20Status motorStatus() const { return _lastMotorStatus; }

    static const char* modeToString(RoofMode m);
    static const char* stateToString(RoofState s);
    static const char* directionToString(BLSD20Direction d);
    static const char* motorStatusToString(BLSD20Status s);

private:
    enum class HomingStep : uint8_t
    {
        SeekClose,
        PauseBeforeOpen,
        SeekOpen
    };

    void applyDefaultConfig();
    void loadCalibration();
    void saveCalibration();
    void refreshCalibrationAtLimit(bool closedEnd);
    void learnSlowTriggerPosition(bool openSide, int32_t pos);
    RoofMode readModeSwitch() const;

    void updateManualHold();
    void updateAutoMove();
    void updateHoming();
    void updatePositionPoll();
    void updateTemperaturePoll();
    void checkMotorHealth();
    void updateWatchdogRelay();

    void enterFault(const char* reason);
    void stopMotor(bool emergency);
    bool startMove(BLSD20Direction dir, uint16_t speed);

    BLSD20Modbus& _motor;

    DebouncedInput _btnOpen, _btnStop, _btnClose, _modeSwitchAuto, _modeSwitchManual, _limitOpen, _limitClose;
    DebouncedInput _slowOpen, _slowClose;

    RoofMode _mode = RoofMode::Auto;
    RoofState _state = RoofState::Uninitialized;
    const char* _faultReason = "";
    const char* _lastRejectReason = "";

    bool _homed = false;
    bool _calibStale = false;
    int32_t _closedPositionCounts = 0;
    int32_t _openPositionCounts = 0;

    // Position (in the same counts as closed/openPositionCounts) at which
    // SLOW_OPEN/SLOW_CLOSE were last observed to trip during a real move.
    // These are point sensors (inductive proximity switches), not
    // continuously-active zone sensors, so requestOpen()/requestClose() use
    // this remembered position -- not just the switch's live state -- to
    // decide whether a restart from a stop already sitting past that point
    // should start slow. Self-corrects (and re-saves) whenever the switch
    // actually trips again with a meaningfully different reading.
    static constexpr int32_t SLOW_POSITION_UNKNOWN = INT32_MIN;
    int32_t _slowOpenPositionCounts = SLOW_POSITION_UNKNOWN;
    int32_t _slowClosePositionCounts = SLOW_POSITION_UNKNOWN;

    HomingStep _homingStep = HomingStep::SeekClose;
    uint32_t _moveStartedMs = 0;
    uint32_t _homingPauseUntilMs = 0;
    bool _slowdownApplied = false;
    uint32_t _slowdownTriggeredMs = 0; // 0 = slow switch not (yet continuously) active
    uint32_t _stopRequestedMs = 0; // 0 = no soft-stop in progress, see requestStop()

    // The watchdog relay pin starts LOW at boot (fail-safe), which briefly
    // breaks the hardware E-stop loop and latches EmergencyStop on the
    // BLSD20. This clears that automatically shortly after boot instead of
    // requiring a manual RESTART every time.
    bool _bootAutoRestartPending = true;
    uint32_t _bootAutoRestartAtMs = 0;

    // Manual-mode button arbitration: a lone press is armed for a short
    // grace window before it actually starts a move, so a genuine
    // Open+Close-together combo can be told apart from a single press.
    uint32_t _comboHoldStartMs = 0;
    int8_t _armedDir = 0; // 0 none, +1 open pending, -1 close pending
    uint32_t _armedPressMs = 0;

    int32_t _lastPosition = 0;
    uint16_t _lastSpeed = 0;
    uint16_t _lastCurrent = 0;
    uint16_t _lastErrorFlags = 0;
    uint32_t _lastPositionPollMs = 0;
    uint8_t _commFailCount = 0;
    bool _lastHardStopInput = false;
    BLSD20Status _lastMotorStatus = BLSD20Status::Stopped;
    BLSD20Direction _lastCommandedDir = RoofConfig::DIR_OPEN;

    float _lastTempMcu = 0;
    float _lastTempMosfet = 0;
    float _lastTempBrake = 0;
    uint32_t _lastTempPollMs = 0;

    uint32_t _restartReapplyAtMs = 0;
};
