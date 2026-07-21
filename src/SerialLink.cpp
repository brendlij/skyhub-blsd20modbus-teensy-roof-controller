#include "SerialLink.h"
#include "Config.h"

SerialLink::SerialLink(RoofController& roof, StatusLed& led, Stream& port) : _roof(roof), _led(led), _port(port) {}

void SerialLink::begin()
{
    _lineBuf.reserve(32);

    pinMode(Pins::CASE_FAN_RELAY, OUTPUT);
    setFan(true);
}

void SerialLink::update()
{
    while (_port.available())
    {
        char c = (char)_port.read();
        if (c == '\n' || c == '\r')
        {
            if (_lineBuf.length() > 0)
            {
                handleLine(_lineBuf);
                _lineBuf = "";
            }
        }
        else if (_lineBuf.length() < 31)
        {
            _lineBuf += c;
        }
    }

    uint32_t now = millis();
    if (now - _lastStatusMs >= RoofConfig::STATUS_REPORT_MS || _roof.state() != _lastReportedState)
    {
        _lastStatusMs = now;
        _lastReportedState = _roof.state();
        printStatus();
    }
}

void SerialLink::handleLine(const String& lineIn)
{
    String line = lineIn;
    line.trim();
    line.toUpperCase();

    bool ok = false;
    if (line == "OPEN") ok = _roof.requestOpen();
    else if (line == "CLOSE") ok = _roof.requestClose();
    else if (line == "STOP") ok = _roof.requestStop();
    else if (line == "HOME") ok = _roof.requestHome();
    else if (line == "SAVE") ok = _roof.requestSaveSettings();
    else if (line == "RESTART") ok = _roof.requestRestart();
    else if (line == "LED ON") { _led.setEnabled(true); ok = true; }
    else if (line == "LED OFF") { _led.setEnabled(false); ok = true; }
    else if (line == "LED TOGGLE") { _led.toggle(); ok = true; }
    else if (line == "FAN ON") { setFan(true); ok = true; }
    else if (line == "FAN OFF") { setFan(false); ok = true; }
    else if (line == "FAN TOGGLE") { setFan(!_fanOn); ok = true; }
    else if (line == "STATUS")
    {
        printStatus();
        return;
    }
    else
    {
        _port.print("ERR unknown_command ");
        _port.println(line);
        return;
    }

    _port.print(ok ? "OK " : "ERR rejected ");
    _port.println(line);
}

void SerialLink::setFan(bool on)
{
    _fanOn = on;
    digitalWrite(Pins::CASE_FAN_RELAY, on ? LOW : HIGH); // relay NC contact: LOW = fan on
}

void SerialLink::printStatus()
{
    _port.print("STATUS mode=");
    _port.print(RoofController::modeToString(_roof.mode()));
    _port.print(" state=");
    _port.print(RoofController::stateToString(_roof.state()));
    _port.print(" homed=");
    _port.print(_roof.isHomed() ? 1 : 0);
    _port.print(" calib_stale=");
    _port.print(_roof.calibStale() ? 1 : 0);
    _port.print(" percent=");
    _port.print(_roof.percentOpen());
    _port.print(" pos=");
    _port.print(_roof.positionCounts());
    _port.print(" speed=");
    _port.print(_roof.motorSpeedRpm());
    _port.print(" current=");
    _port.print(_roof.motorCurrentMa());
    _port.print(" led=");
    _port.print(_led.isEnabled() ? 1 : 0);
    _port.print(" fan=");
    _port.print(_fanOn ? 1 : 0);
    _port.print(" hardstop=");
    _port.print(_roof.hardStopActive() ? 1 : 0);
    _port.print(" dir_cmd=");
    _port.print(RoofController::directionToString(_roof.commandedDirection()));
    _port.print(" dir_motor=");
    _port.print(RoofController::motorStatusToString(_roof.motorStatus()));
    _port.print(" btn_open=");
    _port.print(_roof.btnOpenActive() ? 1 : 0);
    _port.print(" btn_stop=");
    _port.print(_roof.btnStopActive() ? 1 : 0);
    _port.print(" btn_close=");
    _port.print(_roof.btnCloseActive() ? 1 : 0);
    _port.print(" sw_auto=");
    _port.print(_roof.modeSwitchAutoActive() ? 1 : 0);
    _port.print(" sw_manual=");
    _port.print(_roof.modeSwitchManualActive() ? 1 : 0);
    _port.print(" limit_open=");
    _port.print(_roof.limitOpenActive() ? 1 : 0);
    _port.print(" limit_close=");
    _port.print(_roof.limitCloseActive() ? 1 : 0);
    _port.print(" slow_open=");
    _port.print(_roof.slowOpenActive() ? 1 : 0);
    _port.print(" slow_close=");
    _port.print(_roof.slowCloseActive() ? 1 : 0);
    if (_roof.hasFault())
    {
        _port.print(" fault=");
        _port.print(_roof.faultReason());
        _port.print(" errflags=0x");
        _port.print(_roof.lastErrorFlags(), HEX);
    }
    _port.println();
}
