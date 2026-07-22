#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "Config.h"
#include "Types.h"

// Firmware layer: BLSD20 Driver (Spec Abschnitt 4).
// Thin wrapper around BLSD20Modbus: applies the fixed drive configuration,
// issues motion primitives, and caches telemetry so the layers above never
// touch the Modbus object directly.
//
// Note: BLSD20Modbus exposes no getters for control mode / pole count /
// current limit / accel / decel / feature coils, so the "read config,
// compare with Sollwert, write only differences" verification described in
// Spec Abschnitt 11 can only be done for what the library *can* read back
// (target speed, status). Everything else is written unconditionally on
// every boot -- the writes are idempotent and cheap, so this is safe, it
// just can't skip unchanged fields the way the spec's diagram shows.
class BLSD20Driver
{
public:
    explicit BLSD20Driver(BLSD20Modbus& motor);

    // Applies the fixed drive configuration. Synchronous -- see Config.h /
    // RoofConfig for the Sollwerte. Called once at boot before any move
    // command is accepted.
    void begin();

    bool startMove(Direction dir, uint16_t speedRpm);
    bool stop();
    bool emergencyStop();
    bool resetPositionCounter();
    bool setSpeed(uint16_t speedRpm);

    bool saveAndRestart(); // persist current config to flash, then reboot the drive
    bool restart();        // reboot without persisting (also clears latched errors)

    void pollTelemetry();   // position/speed/current/error/hardstop, rate-limited
    void pollTemperature(); // MCU/MOSFET/brake temps, rate-limited (slow-changing)

    int32_t position() const { return _position; }
    uint16_t speed() const { return _speed; }
    uint16_t targetSpeed() const { return _targetSpeed; }
    uint16_t current() const { return _current; }
    uint16_t errorFlags() const { return _errorFlags; }
    bool hasError() const { return (_errorFlags != 0) || _lastHasErrorCall; }
    bool hardStopActive() const { return _hardStopInput; }
    uint8_t commFailCount() const { return _commFailCount; }
    bool modbusConnected() const { return _commFailCount < RoofConfig::MAX_COMM_FAILURES; }
    BLSD20Status motorStatus() const { return _motorStatus; }
    float tempMcu() const { return _tempMcu; }
    float tempMosfet() const { return _tempMosfet; }
    float tempBrake() const { return _tempBrake; }

private:
    void applyConfig();

    BLSD20Modbus& _motor;

    int32_t _position = 0;
    uint16_t _speed = 0;
    uint16_t _targetSpeed = 0;
    uint16_t _current = 0;
    uint16_t _errorFlags = 0;
    bool _lastHasErrorCall = false;
    bool _hardStopInput = false;
    BLSD20Status _motorStatus = BLSD20Status::Stopped;
    uint8_t _commFailCount = 0;
    uint32_t _lastTelemetryPollMs = 0;

    float _tempMcu = 0;
    float _tempMosfet = 0;
    float _tempBrake = 0;
    uint32_t _lastTempPollMs = 0;
};
