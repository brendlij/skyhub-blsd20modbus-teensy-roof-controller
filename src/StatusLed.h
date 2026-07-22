#pragma once

#include <Arduino.h>
#include "DebouncedInput.h"
#include "StateMachine.h"

// Drives a 3-pin common-cathode RGB LED module (e.g. AZDelivery KY-016) to
// give an at-a-glance readout of Motion. A physical button and/or the
// serial link can turn the LED off entirely (cosmetic -- e.g. it shining
// into a room at night), but Fault always overrides that and blinks red
// regardless, since that's safety-relevant and must stay visible.
class StatusLed
{
public:
    void begin(uint8_t pinR, uint8_t pinG, uint8_t pinB, uint8_t pinButton, uint32_t debounceMs);
    void update(const StateMachine& sm);

    void setEnabled(bool enabled) { _enabled = enabled; }
    void toggle() { _enabled = !_enabled; }
    bool isEnabled() const { return _enabled; }

private:
    void setColor(bool r, bool g, bool b);

    uint8_t _pinR = 0, _pinG = 0, _pinB = 0;
    DebouncedInput _button;
    uint32_t _blinkStartMs = 0;
    bool _blinkOn = false;
    bool _enabled = true;
};
