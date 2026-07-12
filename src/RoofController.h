#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "Config.h"
#include "DebouncedInput.h"

enum class RoofMode : uint8_t
{
    Auto,
    Manual
};

enum class RoofState : uint8_t
{
    Uninitialized,
    Idle,
    Homing,
    Opening,
    Closing,
    Fault
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

    RoofMode mode() const { return _mode; }
    RoofState state() const { return _state; }
    bool isHomed() const { return _homed; }
    int8_t percentOpen() const; // 0..100, or -1 if not homed
    int32_t positionCounts() const { return _lastPosition; }
    uint16_t motorSpeedRpm() const { return _lastSpeed; }
    uint16_t motorCurrentMa() const { return _lastCurrent; }
    bool hasFault() const { return _state == RoofState::Fault; }
    const char* faultReason() const { return _faultReason; }

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

    void updateManualHold();
    void updateAutoMove();
    void updateHoming();
    void updatePositionPoll();
    void checkMotorHealth();

    void enterFault(const char* reason);
    void stopMotor(bool emergency);
    bool startMove(BLSD20Direction dir, uint16_t speed);

    BLSD20Modbus& _motor;

    DebouncedInput _btnOpen, _btnStop, _btnClose, _modeSwitch, _limitOpen, _limitClose;

    RoofMode _mode = RoofMode::Auto;
    RoofState _state = RoofState::Uninitialized;
    const char* _faultReason = "";

    bool _homed = false;
    int32_t _closedPositionCounts = 0;
    int32_t _openPositionCounts = 0;

    HomingStep _homingStep = HomingStep::SeekClose;
    uint32_t _moveStartedMs = 0;
    bool _slowdownApplied = false;

    int32_t _lastPosition = 0;
    uint16_t _lastSpeed = 0;
    uint16_t _lastCurrent = 0;
    uint32_t _lastPositionPollMs = 0;
    uint8_t _commFailCount = 0;
};
