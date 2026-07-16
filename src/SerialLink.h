#pragma once

#include <Arduino.h>
#include "RoofController.h"
#include "StatusLed.h"

// Line-based USB command protocol for the Pi.
// Commands (newline terminated, case-insensitive): OPEN, CLOSE, STOP, HOME,
// SAVE, RESTART, LED ON, LED OFF, LED TOGGLE, STATUS. Each is answered with
// "OK <cmd>" or "ERR <reason> <cmd>". Telemetry is pushed unprompted as
// "STATUS key=value ..." lines, both periodically and immediately on any
// state change.
class SerialLink
{
public:
    SerialLink(RoofController& roof, StatusLed& led, Stream& port);

    void begin();
    void update();

private:
    void handleLine(const String& line);
    void printStatus();

    RoofController& _roof;
    StatusLed& _led;
    Stream& _port;
    String _lineBuf;
    uint32_t _lastStatusMs = 0;
    RoofState _lastReportedState = RoofState::Uninitialized;
};
