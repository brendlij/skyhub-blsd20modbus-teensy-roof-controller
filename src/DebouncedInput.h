#pragma once

#include <Arduino.h>

// Debounced digital input, active-low by default (button/switch pulled to GND).
// inverted=true flips the logic only -- the internal pull-up stays on, so it
// works whether the source drives HIGH or just goes high-Z, and a broken wire
// then reads active (the fail-safe direction for the rain sensor).
class DebouncedInput
{
public:
    void begin(uint8_t pin, uint32_t debounceMs, bool inverted = false)
    {
        _pin = pin;
        _debounceMs = debounceMs;
        _inverted = inverted;
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
    bool readRaw() const
    {
        bool high = digitalRead(_pin) == HIGH;
        return _inverted ? high : !high;
    }

    uint8_t _pin = 0;
    uint32_t _debounceMs = 30;
    bool _inverted = false;
    bool _stable = false;
    bool _prevStable = false;
    bool _lastRaw = false;
    uint32_t _lastChangeMs = 0;
};
