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
    uint16_t lastErrorFlags() const { return _lastErrorFlags; }

    // True after a RESTART, until the roof next fully closes (which
    // re-zeroes the controller's position counter and confirms it's back in
    // sync). Informational only -- OPEN/CLOSE are never blocked by this.
    bool calibStale() const { return _calibStale; }

    // Best-effort read of the BLSD20's own HARD_STOP input (DI_HARD_STOP).
    // Purely informational -- the hardware E-stop loop already stops the
    // motor without any help from the Teensy; this just lets that trip be
    // reported upstream (e.g. to the Pi5).
    bool hardStopActive() const { return _lastHardStopInput; }

    static const char* modeToString(RoofMode m);
    static const char* stateToString(RoofState s);

private:
    enum class HomingStep : uint8_t
    {
        SeekClose,
        SeekOpen
    };

    void applyDefaultConfig();
    void loadCalibration();
    void saveCalibration();
    void refreshCalibrationAtLimit(bool closedEnd);
    RoofMode readModeSwitch() const;

    void updateManualHold();
    void updateAutoMove();
    void updateHoming();
    void updatePositionPoll();
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

    bool _homed = false;
    bool _calibStale = false;
    int32_t _closedPositionCounts = 0;
    int32_t _openPositionCounts = 0;

    HomingStep _homingStep = HomingStep::SeekClose;
    uint32_t _moveStartedMs = 0;
    bool _slowdownApplied = false;

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

    uint32_t _restartReapplyAtMs = 0;
};
