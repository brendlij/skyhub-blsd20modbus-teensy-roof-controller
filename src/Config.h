#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>

// Matches confirmed wiring -- changing these needs rewiring to match.
namespace Pins
{
    constexpr int8_t MODBUS_DE = -1; // Waveshare RS-485 auto-direction, no DE pin needed

    // Buttons: edge-triggered in Auto, hold-to-run in Manual.
    constexpr uint8_t BTN_OPEN = 4; // DO NOT CHANGE without rewiring
    constexpr uint8_t BTN_STOP = 3;
    constexpr uint8_t BTN_CLOSE = 2; // DO NOT CHANGE without rewiring

    // 3-position rotary switch; center position grounds neither -> RoofMode::Off.
    constexpr uint8_t MODE_SWITCH_AUTO = 5;
    constexpr uint8_t MODE_SWITCH_MANUAL = 12;

    // Endschalter, wired directly into the Teensy (hard safety cutoff, independent of Modbus).
    constexpr uint8_t LIMIT_OPEN = 7;  // DO NOT CHANGE without rewiring
    constexpr uint8_t LIMIT_CLOSE = 6; // DO NOT CHANGE without rewiring

    // Inductive slowdown switches, inboard of LIMIT_OPEN/LIMIT_CLOSE.
    constexpr uint8_t SLOW_OPEN = 15;  // DO NOT CHANGE without rewiring
    constexpr uint8_t SLOW_CLOSE = 14; // DO NOT CHANGE without rewiring

    // Watchdog relay: energized only while Modbus comms are healthy.
    constexpr uint8_t MODBUS_WATCHDOG_RELAY = 16;

    // Case fan relay, NC contact: LOW = fan on. See SerialLink FAN command.
    constexpr uint8_t CASE_FAN_RELAY = 17;

    // Status RGB LED, common cathode.
    constexpr uint8_t LED_R = 8;
    constexpr uint8_t LED_G = 9;
    constexpr uint8_t LED_B = 10;
    constexpr uint8_t LED_BUTTON = 11; // toggles the LED on/off
}

namespace RoofConfig
{
    constexpr uint8_t MODBUS_SLAVE_ID = 1;
    constexpr uint32_t MODBUS_BAUD = 115200;

    constexpr uint32_t BOOT_AUTO_RESTART_DELAY_MS = 2000; // auto-RESTART the BLSD20 this long after Teensy boot

    constexpr bool AUTO_REQUIRES_HOMING = false;           // require a successful HOME before Auto OPEN/CLOSE
    constexpr bool AUTO_HAS_SLOW_SWITCHES = true;          // false = always SPEED_AUTO_SLOW, never SPEED_AUTO_FAST
    constexpr bool MANUAL_RESPECTS_LIMIT_SWITCHES = false; // false = Manual ignores LIMIT_OPEN/LIMIT_CLOSE entirely

    // Speeds in rpm, per BLSD20Modbus::setSpeed().
    constexpr uint16_t SPEED_AUTO_FAST = 2000;
    constexpr uint16_t SPEED_AUTO_SLOW = 500;
    constexpr uint16_t SPEED_MANUAL = 500;
    constexpr uint16_t SPEED_HOMING = 500;

    constexpr uint32_t SLOWDOWN_DELAY_MS = 5000;      // delay after SLOW_OPEN/SLOW_CLOSE first trips before dropping speed
    constexpr uint32_t STOP_SLOWDOWN_DELAY_MS = 1500; // soft-stop delay after STOP while Opening/Closing

    constexpr int32_t MIN_HOMING_RANGE_COUNTS = 1000; // shorter homing runs are rejected as bogus
    constexpr uint32_t HOMING_LEG_PAUSE_MS = 500;     // pause between the SeekClose stop and starting SeekOpen

    // DO NOT CHANGE -- confirmed against actual motor rotation.
    constexpr BLSD20Direction DIR_OPEN = BLSD20Direction::Backward;
    constexpr BLSD20Direction DIR_CLOSE = BLSD20Direction::Forward;

    // Drive defaults, re-applied every boot -- see RoofController::applyDefaultConfig.
    constexpr uint16_t POLE_COUNT = 10;
    constexpr uint16_t CURRENT_LIMIT_MA = 13000;
    constexpr uint16_t ACCEL = 1000;
    constexpr uint16_t DECEL = 1000;

    constexpr uint32_t DEBOUNCE_MS = 30;
    constexpr uint32_t POSITION_POLL_MS = 100;
    constexpr uint32_t STATUS_REPORT_MS = 500;
    constexpr uint32_t HOMING_TIMEOUT_MS = 160000;
    constexpr uint32_t MOVE_TIMEOUT_MS = 48000;
    constexpr uint8_t MAX_COMM_FAILURES = 3;

    constexpr uint32_t RESTART_REAPPLY_DELAY_MS = 1000;        // let the BLSD20 finish rebooting before reconfiguring
    constexpr int32_t CALIBRATION_DRIFT_TOLERANCE_COUNTS = 50; // EEPROM re-write threshold for the open-end position
    constexpr uint32_t HOME_COMBO_HOLD_MS = 3000;              // Manual: hold Open+Close together this long to HOME
    constexpr uint32_t MANUAL_COMBO_GRACE_MS = 150;            // grace window to detect a combo vs. a lone press
}
