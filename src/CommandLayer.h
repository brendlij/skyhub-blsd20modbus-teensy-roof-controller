#pragma once

#include <Arduino.h>
#include "Config.h"
#include "DebouncedInput.h"
#include "StateMachine.h"

// Firmware layer: Command Layer (Spec Abschnitt 4).
// Turns raw button/mode-switch input into the same abstract requests the
// USB link issues, so both control paths funnel through
// StateMachine::request*() and obey identical guards (Spec Abschnitt 2).
class CommandLayer
{
public:
    explicit CommandLayer(StateMachine& sm);

    void begin();
    void tick();

    // Pass-throughs used by SerialLink for USB text commands, kept here so
    // both button and USB dispatch share one place.
    bool cmdOpen() { return _sm.requestOpen(); }
    bool cmdClose() { return _sm.requestClose(); }
    bool cmdStop() { return _sm.requestStop(); }
    bool cmdHome() { return _sm.requestHome(); }
    bool cmdEnable() { return _sm.requestEnable(); }
    bool cmdDisable() { return _sm.requestDisable(); }
    bool cmdResetFault() { return _sm.requestResetFault(); }
    bool cmdRestart() { return _sm.requestRestart(); }
    bool cmdPercent(uint8_t percent) { return _sm.requestMoveToPercent(percent); }

    // Two-step, unlike the commands above -- see StateMachine::fwUpdateAllowed().
    bool cmdFwUpdateAllowed() const { return _sm.fwUpdateAllowed(); }
    void cmdEnterBootloader() { _sm.enterBootloader(); }

    // Raw sensor states, for diagnostics (e.g. checking wiring/direction).
    bool btnOpenActive() const { return _btnOpen.isActive(); }
    bool btnStopActive() const { return _btnStop.isActive(); }
    bool btnCloseActive() const { return _btnClose.isActive(); }
    bool modeSwitchAutoActive() const { return _modeSwitchAuto.isActive(); }
    bool modeSwitchManualActive() const { return _modeSwitchManual.isActive(); }

private:
    RoofMode readModeSwitch() const;
    void updateManualHold();

    StateMachine& _sm;
    DebouncedInput _btnOpen, _btnStop, _btnClose, _modeSwitchAuto, _modeSwitchManual;

    // Manual-mode button arbitration: a lone press is armed for a short
    // grace window before it actually starts a move, so a genuine
    // Open+Close-together combo can be told apart from a single press.
    uint32_t _comboHoldStartMs = 0;
    int8_t _armedDir = 0; // 0 none, +1 open pending, -1 close pending
    uint32_t _armedPressMs = 0;
};
