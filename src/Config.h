#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>

// Placeholder pinout -- adjust to match your actual wiring.
namespace Pins
{
    // RS-485 (Modbus). Serial1 hardware pins RX1=0 / TX1=1 are fixed by the
    // Teensy 4.1 UART. The Waveshare "TTL TO RS485 (B)" converter switches
    // direction automatically, so no DE/driver-enable GPIO is needed here.
    constexpr int8_t MODBUS_DE = -1;

    // Same physical buttons are read differently depending on Mode: edge
    // triggered in Auto, hold-to-run in Manual.
    constexpr uint8_t BTN_OPEN = 2;
    constexpr uint8_t BTN_STOP = 3;
    constexpr uint8_t BTN_CLOSE = 4;

    // 3-position rotary switch (2NO 2NC), e.g. Nimomo 19mm: one leg grounds
    // MODE_SWITCH_AUTO in the Auto position, the other grounds
    // MODE_SWITCH_MANUAL in the Manual position; the center position grounds
    // neither, which reads as RoofMode::Off (lockout).
    constexpr uint8_t MODE_SWITCH_AUTO = 5;
    constexpr uint8_t MODE_SWITCH_MANUAL = 12;

    // Endschalter, wired directly into the Teensy (not the motor controller)
    // so they remain a hard safety cutoff even if the Modbus link is unwell.
    constexpr uint8_t LIMIT_OPEN = 6;
    constexpr uint8_t LIMIT_CLOSE = 7;

    // Inductive slowdown switches, one per side, mounted inboard of
    // LIMIT_OPEN/LIMIT_CLOSE. Mark the point where an auto move drops from
    // SPEED_AUTO_FAST to SPEED_AUTO_SLOW, replacing the old counts-based
    // guess. Full speed is allowed anywhere between the two.
    constexpr uint8_t SLOW_OPEN = 14;
    constexpr uint8_t SLOW_CLOSE = 15;

    // Drives a relay whose contact sits in series inside the hardware E-stop
    // loop (LIMIT_OPEN's outer twin -> LIMIT_CLOSE's outer twin -> this
    // relay -> E-stop button -> BLSD20 HARD_STOP input). Energized only
    // while Modbus comms are healthy, so a comm loss or a Teensy crash/reset
    // (pin defaults LOW at boot) breaks the loop and hard-stops the motor
    // entirely outside of Modbus.
    constexpr uint8_t MODBUS_WATCHDOG_RELAY = 16;

    // Case fan relay. Fan is wired to the relay's NC contact, so it runs
    // whenever the pin is LOW (relay de-energized) and stops when the pin
    // is driven HIGH. Defaults on at boot; toggled via the FAN serial
    // command, see SerialLink.
    constexpr uint8_t CASE_FAN_RELAY = 17;

    // Status RGB LED (e.g. AZDelivery KY-016), common cathode: common pin to
    // GND, each color pin driven HIGH through the module's onboard resistors.
    constexpr uint8_t LED_R = 8;
    constexpr uint8_t LED_G = 9;
    constexpr uint8_t LED_B = 10;

    // Momentary push button: toggles the status LED on/off (a Fault still
    // always blinks red regardless, see StatusLed::update).
    constexpr uint8_t LED_BUTTON = 11;
}

namespace RoofConfig
{
    constexpr uint8_t MODBUS_SLAVE_ID = 1;
    constexpr uint32_t MODBUS_BAUD = 115200;

    // If false, Auto-mode OPEN/CLOSE work even with homed=false (e.g. slow
    // switches not wired yet -- LIMIT_OPEN/LIMIT_CLOSE alone still stop the
    // move). percent stays -1 and no calibration is saved until a HOME
    // actually succeeds. Set true once homing + slow switches are wired up.
    constexpr bool AUTO_REQUIRES_HOMING = false;

    // If false, SLOW_OPEN/SLOW_CLOSE aren't wired up yet, so an Auto move
    // never uses SPEED_AUTO_FAST -- it starts (and stays) at SPEED_AUTO_SLOW
    // for the whole travel, same as if the slow switch had already tripped.
    // Set true once the slow switches are wired up, to get real fast/slow
    // Auto moves.
    constexpr bool AUTO_HAS_SLOW_SWITCHES = false;

    // Speeds in rpm, per BLSD20Modbus::setSpeed().
    constexpr uint16_t SPEED_AUTO_FAST = 2000;
    constexpr uint16_t SPEED_AUTO_SLOW = 500;
    constexpr uint16_t SPEED_MANUAL = 500;
    constexpr uint16_t SPEED_HOMING = 500;

    // A homing run producing less travel than this is rejected as bogus
    // (e.g. a limit switch that never triggered).
    constexpr int32_t MIN_HOMING_RANGE_COUNTS = 1000;

    // Swap these two if the roof moves the wrong way relative to Open/Close.
    constexpr BLSD20Direction DIR_OPEN = BLSD20Direction::Forward;
    constexpr BLSD20Direction DIR_CLOSE = BLSD20Direction::Backward;

    // Drive defaults, applied every boot (see RoofController::begin). These
    // mirror the one-time provisioning values; saveSettings()/restart() are
    // only ever triggered explicitly via the "SAVE" serial command.
    constexpr uint16_t POLE_COUNT = 10;
    constexpr uint16_t CURRENT_LIMIT_MA = 13000;
    constexpr uint16_t ACCEL = 1000;
    constexpr uint16_t DECEL = 1000;

    constexpr uint32_t DEBOUNCE_MS = 30;
    constexpr uint32_t POSITION_POLL_MS = 100;
    constexpr uint32_t STATUS_REPORT_MS = 500;
    constexpr uint32_t HOMING_TIMEOUT_MS = 60000;
    constexpr uint32_t MOVE_TIMEOUT_MS = 120000;
    constexpr uint8_t MAX_COMM_FAILURES = 3;

    // Time to let the BLSD20 finish rebooting after restart() before we
    // resume sending it configuration writes.
    constexpr uint32_t RESTART_REAPPLY_DELAY_MS = 800;

    // A freshly observed open-end position is only re-persisted to EEPROM if
    // it differs from the stored value by more than this many counts, so
    // ordinary Hall-sensor jitter between cycles doesn't wear the flash.
    constexpr int32_t CALIBRATION_DRIFT_TOLERANCE_COUNTS = 50;

    // Manual mode: hold Open+Close together (from Idle) for this long to
    // trigger a HOME run without going through the serial link.
    constexpr uint32_t HOME_COMBO_HOLD_MS = 3000;

    // A lone button press only starts a manual move once this grace window
    // has passed without the other button also coming down -- otherwise the
    // first button pressed would always win the race against the combo.
    constexpr uint32_t MANUAL_COMBO_GRACE_MS = 150;
}
