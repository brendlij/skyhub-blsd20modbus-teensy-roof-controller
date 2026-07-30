#include "SerialLink.h"
#include "Config.h"
#include "FirmwareInfo.h"

SerialLink::SerialLink(StateMachine& sm, CommandLayer& cmd, StatusLed& led, Stream& port)
    : _sm(sm), _cmd(cmd), _led(led), _port(port) {}

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
    if (now - _lastStatusMs >= RoofConfig::STATUS_REPORT_MS || _sm.motion() != _lastReportedMotion)
    {
        _lastStatusMs = now;
        _lastReportedMotion = _sm.motion();
        printStatus();
    }
}

void SerialLink::handleLine(const String& lineIn)
{
    String line = lineIn;
    line.trim();
    line.toUpperCase();

    bool ok = false;
    bool useRejectReason = false;
    if (line == "OPEN") { ok = _cmd.cmdOpen(); useRejectReason = true; }
    else if (line == "CLOSE") { ok = _cmd.cmdClose(); useRejectReason = true; }
    else if (line == "STOP") ok = _cmd.cmdStop();
    else if (line == "HOME") ok = _cmd.cmdHome();
    else if (line == "ENABLE") ok = _cmd.cmdEnable();
    else if (line == "DISABLE") ok = _cmd.cmdDisable();
    else if (line == "RESETFAULT") ok = _cmd.cmdResetFault();
    else if (line == "RESTART") ok = _cmd.cmdRestart();
    else if (line == "FWUPDATE")
    {
        // Handled inline rather than through the shared OK/ERR tail below,
        // because the acknowledgement has to be on the wire *before* the
        // jump to the bootloader tears the USB link down -- otherwise SkyHub
        // only ever sees the port vanish and cannot tell success from a
        // crash. Blocking the loop here is acceptable: the roof is not
        // moving (fwUpdateAllowed()) and we are about to reset anyway.
        if (!_cmd.cmdFwUpdateAllowed())
        {
            _port.println("ERR rejected FWUPDATE");
            return;
        }
        _port.println("OK FWUPDATE");
        _port.flush();
        delay(RoofConfig::FWUPDATE_ACK_GRACE_MS);
        _cmd.cmdEnterBootloader(); // does not return
        return;
    }
    else if (line == "INFO")
    {
        FirmwareInfo::printJson(_port);
        return;
    }
    else if (line.startsWith("PERCENT "))
    {
        long val = line.substring(8).toInt();
        if (val < 0 || val > 100)
        {
            _port.print("ERR invalid_percent ");
            _port.println(line);
            return;
        }
        ok = _cmd.cmdPercent((uint8_t)val);
        useRejectReason = true;
    }
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

    if (ok)
    {
        _port.print("OK ");
    }
    else if (useRejectReason && _sm.lastRejectReason()[0] != '\0')
    {
        _port.print("ERR ");
        _port.print(_sm.lastRejectReason());
        _port.print(" ");
    }
    else
    {
        _port.print("ERR rejected ");
    }
    _port.println(line);
}

void SerialLink::setFan(bool on)
{
    _fanOn = on;
    digitalWrite(Pins::CASE_FAN_RELAY, on ? LOW : HIGH); // relay NC contact: LOW = fan on
}

// SkyHub soll niemals selbst Zustände berechnen müssen (Spec Abschnitt 13) --
// every field the roof's own logic depends on is reported directly.
void SerialLink::printStatus()
{
    _port.print("STATUS mode=");
    _port.print(StateMachine::modeToString(_sm.mode()));
    _port.print(" motion=");
    _port.print(StateMachine::motionToString(_sm.motion()));
    _port.print(" position=");
    _port.print(StateMachine::positionToString(_sm.position()));
    _port.print(" direction=");
    _port.print(StateMachine::directionToString(_sm.direction()));
    _port.print(" zone=");
    _port.print(StateMachine::zoneToString(_sm.zone()));
    _port.print(" percent=");
    _port.print(_sm.percentOpen());
    _port.print(" target_percent=");
    _port.print(_sm.targetPercent());
    _port.print(" hall=");
    _port.print(_sm.hallCounter());
    _port.print(" speed=");
    _port.print(_sm.currentSpeedRpm());
    _port.print(" target_speed=");
    _port.print(_sm.targetSpeedRpm());
    _port.print(" current=");
    _port.print(_sm.motorCurrentMa());
    _port.print(" motor_enabled=");
    _port.print(_sm.motorEnabled() ? 1 : 0);
    _port.print(" homed=");
    _port.print(_sm.isHomed() ? 1 : 0);
    _port.print(" calib_stale=");
    _port.print(_sm.calibStale() ? 1 : 0);
    _port.print(" modbus_connected=");
    _port.print(_sm.modbusConnected() ? 1 : 0);
    _port.print(" comm_fail=");
    _port.print(_sm.commFailCount());
    _port.print(" temp_mcu=");
    _port.print(_sm.tempMcuC(), 1);
    _port.print(" temp_mosfet=");
    _port.print(_sm.tempMosfetC(), 1);
    _port.print(" temp_brake=");
    _port.print(_sm.tempBrakeC(), 1);
    _port.print(" led=");
    _port.print(_led.isEnabled() ? 1 : 0);
    _port.print(" fan=");
    _port.print(_fanOn ? 1 : 0);
    _port.print(" hardstop=");
    _port.print(_sm.hardStopActive() ? 1 : 0);
    _port.print(" btn_open=");
    _port.print(_cmd.btnOpenActive() ? 1 : 0);
    _port.print(" btn_stop=");
    _port.print(_cmd.btnStopActive() ? 1 : 0);
    _port.print(" btn_close=");
    _port.print(_cmd.btnCloseActive() ? 1 : 0);
    _port.print(" sw_auto=");
    _port.print(_cmd.modeSwitchAutoActive() ? 1 : 0);
    _port.print(" sw_manual=");
    _port.print(_cmd.modeSwitchManualActive() ? 1 : 0);
    _port.print(" limit_open=");
    _port.print(_sm.limitOpenActive() ? 1 : 0);
    _port.print(" limit_close=");
    _port.print(_sm.limitCloseActive() ? 1 : 0);
    _port.print(" slow_open=");
    _port.print(_sm.slowOpenActive() ? 1 : 0);
    _port.print(" slow_close=");
    _port.print(_sm.slowCloseActive() ? 1 : 0);
    _port.print(" rain=");
    _port.print(_sm.rainActive() ? 1 : 0);
    _port.print(" fw_version=");
    _port.print(FirmwareInfo::VERSION);
    if (_sm.hasFault())
    {
        _port.print(" fault=");
        _port.print(_sm.faultReason());
        _port.print(" errflags=0x");
        _port.print(_sm.errorFlags(), HEX);
    }
    _port.println();
}
