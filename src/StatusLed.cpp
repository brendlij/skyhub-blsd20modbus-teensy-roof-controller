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
//   solid green     - stopped, homed
//   blinking yellow - stopped but not yet homed, homing in progress, or
//                      re-applying config after a restart
//   solid blue      - opening
//   solid magenta   - closing
//   solid cyan      - disabled (motor lockout)
// Fault always blinks red, even if the LED has been switched off.
void StatusLed::update(const StateMachine& sm)
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

    if (sm.motion() == Motion::Fault)
    {
        setColor(_blinkOn, false, false);
        return;
    }

    if (!_enabled)
    {
        setColor(false, false, false);
        return;
    }

    if (sm.isReapplyingConfig())
    {
        setColor(_blinkOn, _blinkOn, _blinkOn);
        return;
    }

    switch (sm.motion())
    {
    case Motion::Homing:
        setColor(_blinkOn, _blinkOn, false);
        break;
    case Motion::Opening:
        setColor(false, false, true);
        break;
    case Motion::Closing:
        setColor(true, false, true);
        break;
    case Motion::Disabled:
        setColor(false, true, true);
        break;
    case Motion::Stopped:
        if (sm.isHomed())
            setColor(false, true, false);
        else
            setColor(_blinkOn, _blinkOn, false);
        break;
    case Motion::Fault:
    default:
        setColor(false, false, false);
        break;
    }
}
