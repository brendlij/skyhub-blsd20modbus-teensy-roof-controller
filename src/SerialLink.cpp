#include "SerialLink.h"
#include "Config.h"

SerialLink::SerialLink(RoofController& roof, Stream& port) : _roof(roof), _port(port) {}

void SerialLink::begin()
{
    _lineBuf.reserve(32);
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

void SerialLink::printStatus()
{
    _port.print("STATUS mode=");
    _port.print(RoofController::modeToString(_roof.mode()));
    _port.print(" state=");
    _port.print(RoofController::stateToString(_roof.state()));
    _port.print(" homed=");
    _port.print(_roof.isHomed() ? 1 : 0);
    _port.print(" percent=");
    _port.print(_roof.percentOpen());
    _port.print(" pos=");
    _port.print(_roof.positionCounts());
    _port.print(" speed=");
    _port.print(_roof.motorSpeedRpm());
    _port.print(" current=");
    _port.print(_roof.motorCurrentMa());
    if (_roof.hasFault())
    {
        _port.print(" fault=");
        _port.print(_roof.faultReason());
    }
    _port.println();
}
