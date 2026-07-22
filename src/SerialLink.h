#pragma once

#include <Arduino.h>
#include "CommandLayer.h"
#include "StateMachine.h"
#include "StatusLed.h"

// USB transport for the Pi (Spec Abschnitt 13/14). Pure transport: parses
// newline-terminated lines and forwards movement/system commands to the
// Command Layer, so USB and buttons dispatch through the exact same
// StateMachine guards. LED/FAN are peripherals, not roof motion, so they're
// handled directly here.
//
// Commands (case-insensitive): OPEN, CLOSE, STOP, HOME, ENABLE, DISABLE,
// RESETFAULT, RESTART, FWUPDATE, LED ON, LED OFF, LED TOGGLE, FAN ON,
// FAN OFF, FAN TOGGLE, STATUS. Each is answered with "OK <cmd>" or
// "ERR <reason> <cmd>". Telemetry is pushed unprompted as
// "STATUS key=value ..." lines, both periodically and immediately on any
// Motion change -- SkyHub should never need to compute roof state itself
// (Spec Abschnitt 13).
class SerialLink
{
public:
    SerialLink(StateMachine& sm, CommandLayer& cmd, StatusLed& led, Stream& port);

    void begin();
    void update();

private:
    void handleLine(const String& line);
    void printStatus();
    void setFan(bool on);

    StateMachine& _sm;
    CommandLayer& _cmd;
    StatusLed& _led;
    Stream& _port;
    String _lineBuf;
    uint32_t _lastStatusMs = 0;
    Motion _lastReportedMotion = Motion::Stopped;
    bool _fanOn = true;
};
