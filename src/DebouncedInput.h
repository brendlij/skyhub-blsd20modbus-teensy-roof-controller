#pragma once

#include <Arduino.h>

// Debounced digital input, active-low (button/switch pulled to GND).
class DebouncedInput
{
public:
    void begin(uint8_t pin, uint32_t debounceMs)
    {
        _pin = pin;
        _debounceMs = debounceMs;
        pinMode(_pin, INPUT_PULLUP);
        _stable = readRaw();
        _prevStable = _stable;
        _lastRaw = _stable;
        _lastChangeMs = millis();
    }

    void update()
    {
        bool raw = readRaw();
        if (raw != _lastRaw)
        {
            _lastRaw = raw;
            _lastChangeMs = millis();
        }
        _prevStable = _stable;
        if ((millis() - _lastChangeMs) >= _debounceMs)
        {
            _stable = _lastRaw;
        }
    }

    bool isActive() const { return _stable; }
    bool wasPressed() const { return _stable && !_prevStable; }
    bool wasReleased() const { return !_stable && _prevStable; }

private:
    bool readRaw() const { return digitalRead(_pin) == LOW; }

    uint8_t _pin = 0;
    uint32_t _debounceMs = 30;
    bool _stable = false;
    bool _prevStable = false;
    bool _lastRaw = false;
    uint32_t _lastChangeMs = 0;
};
