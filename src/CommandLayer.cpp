#include "CommandLayer.h"

CommandLayer::CommandLayer(StateMachine& sm) : _sm(sm) {}

void CommandLayer::begin()
{
    _btnOpen.begin(Pins::BTN_OPEN, RoofConfig::DEBOUNCE_MS);
    _btnStop.begin(Pins::BTN_STOP, RoofConfig::DEBOUNCE_MS);
    _btnClose.begin(Pins::BTN_CLOSE, RoofConfig::DEBOUNCE_MS);
    _modeSwitchAuto.begin(Pins::MODE_SWITCH_AUTO, RoofConfig::DEBOUNCE_MS);
    _modeSwitchManual.begin(Pins::MODE_SWITCH_MANUAL, RoofConfig::DEBOUNCE_MS);
    _sm.setMode(readModeSwitch());
}

RoofMode CommandLayer::readModeSwitch() const
{
    bool autoLeg = _modeSwitchAuto.isActive();
    bool manualLeg = _modeSwitchManual.isActive();
    if (manualLeg && !autoLeg) return RoofMode::Manual;
    if (autoLeg && !manualLeg) return RoofMode::Auto;
    // Center position grounds neither leg; both active simultaneously would
    // mean a wiring fault. Either way, fail safe into lockout.
    return RoofMode::Off;
}

void CommandLayer::tick()
{
    _btnOpen.update();
    _btnStop.update();
    _btnClose.update();
    _modeSwitchAuto.update();
    _modeSwitchManual.update();

    RoofMode mode = readModeSwitch();
    _sm.setMode(mode);

    if (_btnStop.wasPressed())
    {
        _sm.requestStop();
    }

    switch (mode)
    {
    case RoofMode::Auto:
        if (_btnOpen.wasPressed()) _sm.requestOpen();
        if (_btnClose.wasPressed()) _sm.requestClose();
        break;
    case RoofMode::Manual:
        updateManualHold();
        break;
    case RoofMode::Off:
        _armedDir = 0;
        _comboHoldStartMs = 0;
        break;
    }
}

void CommandLayer::updateManualHold()
{
    bool openHeld = _btnOpen.isActive();
    bool closeHeld = _btnClose.isActive();
    bool moving = _sm.motion() == Motion::Opening || _sm.motion() == Motion::Closing;

    if (!moving)
    {
        if (openHeld && closeHeld)
        {
            // Combo: both held together -> arm a HOME instead of a move.
            _armedDir = 0;
            if (_comboHoldStartMs == 0)
            {
                _comboHoldStartMs = millis();
            }
            else if (millis() - _comboHoldStartMs >= RoofConfig::HOME_COMBO_HOLD_MS)
            {
                _sm.requestHome();
                _comboHoldStartMs = 0;
            }
        }
        else
        {
            _comboHoldStartMs = 0;

            if (_btnOpen.wasPressed()) { _armedDir = 1; _armedPressMs = millis(); }
            if (_btnClose.wasPressed()) { _armedDir = -1; _armedPressMs = millis(); }

            if (_armedDir == 1 && !openHeld) _armedDir = 0;   // released before grace elapsed
            if (_armedDir == -1 && !closeHeld) _armedDir = 0;

            if (_armedDir != 0 && millis() - _armedPressMs >= RoofConfig::MANUAL_COMBO_GRACE_MS)
            {
                _sm.beginManualMove(_armedDir == 1 ? Direction::Opening : Direction::Closing);
                _armedDir = 0;
            }
        }
    }
    else
    {
        _armedDir = 0;
        _comboHoldStartMs = 0;

        bool shouldStop = (_sm.motion() == Motion::Opening && !openHeld) ||
                           (_sm.motion() == Motion::Closing && !closeHeld);
        if (shouldStop) _sm.endManualMove();
    }
}
