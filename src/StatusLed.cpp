#include "StatusLed.h"

namespace
{
    constexpr uint32_t BLINK_PERIOD_MS = 500;
}

void StatusLed::begin(uint8_t pinR, uint8_t pinG, uint8_t pinB, uint8_t pinButton, uint32_t debounceMs)
{
    _pinR = pinR;
    _pinG = pinG;
    _pinB = pinB;
    pinMode(_pinR, OUTPUT);
    pinMode(_pinG, OUTPUT);
    pinMode(_pinB, OUTPUT);
    setColor(false, false, false);

    _button.begin(pinButton, debounceMs);
}

void StatusLed::setColor(bool r, bool g, bool b)
{
    digitalWrite(_pinR, r ? HIGH : LOW);
    digitalWrite(_pinG, g ? HIGH : LOW);
    digitalWrite(_pinB, b ? HIGH : LOW);
}

// Color scheme when enabled:
//   solid green    - idle, homed
//   blinking yellow- idle but not yet homed, or homing in progress
//   solid blue     - opening
//   solid magenta  - closing
//   blinking white - restarting the drive
// Fault always blinks red, even if the LED has been switched off.
void StatusLed::update(const RoofController& roof)
{
    _button.update();
    if (_button.wasPressed())
    {
        toggle();
    }

    uint32_t now = millis();
    if (now - _blinkStartMs >= BLINK_PERIOD_MS)
    {
        _blinkStartMs = now;
        _blinkOn = !_blinkOn;
    }

    if (roof.state() == RoofState::Fault)
    {
        setColor(_blinkOn, false, false);
        return;
    }

    if (!_enabled)
    {
        setColor(false, false, false);
        return;
    }

    switch (roof.state())
    {
    case RoofState::Homing:
        setColor(_blinkOn, _blinkOn, false);
        break;
    case RoofState::Restarting:
        setColor(_blinkOn, _blinkOn, _blinkOn);
        break;
    case RoofState::Opening:
        setColor(false, false, true);
        break;
    case RoofState::Closing:
        setColor(true, false, true);
        break;
    case RoofState::Idle:
        if (roof.isHomed())
            setColor(false, true, false);
        else
            setColor(_blinkOn, _blinkOn, false);
        break;
    case RoofState::Uninitialized:
    default:
        setColor(false, false, false);
        break;
    }
}
