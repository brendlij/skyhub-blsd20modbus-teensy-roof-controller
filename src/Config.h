#pragma once

#include <Arduino.h>
#include <BLSD20Modbus.h>

// Placeholder pinout -- adjust to match your actual wiring.
namespace Pins
{
    // RS-485 (Modbus). Serial1 hardware pins RX1=0 / TX1=1 are fixed by the
    // Teensy 4.1 UART, only the transceiver DE pin is configurable.
    constexpr int8_t MODBUS_DE = 2;

    // Same physical buttons are read differently depending on Mode: edge
    // triggered in Auto, hold-to-run in Manual.
    constexpr uint8_t BTN_OPEN = 3;
    constexpr uint8_t BTN_STOP = 4;
    constexpr uint8_t BTN_CLOSE = 5;

    // Toggle switch: tied to GND = Manual, floating/pulled-up = Auto.
    constexpr uint8_t MODE_SWITCH = 6;

    // Endschalter, wired directly into the Teensy (not the motor controller)
    // so they remain a hard safety cutoff even if the Modbus link is unwell.
    constexpr uint8_t LIMIT_OPEN = 7;
    constexpr uint8_t LIMIT_CLOSE = 8;
}

namespace RoofConfig
{
    constexpr uint8_t MODBUS_SLAVE_ID = 1;
    constexpr uint32_t MODBUS_BAUD = 115200;

    // Speeds in rpm, per BLSD20Modbus::setSpeed().
    constexpr uint16_t SPEED_AUTO_FAST = 2000;
    constexpr uint16_t SPEED_AUTO_SLOW = 500;
    constexpr uint16_t SPEED_MANUAL = 800;
    constexpr uint16_t SPEED_HOMING = 500;

    // Fraction of the calibrated travel range, at each end, where an auto
    // move drops from SPEED_AUTO_FAST to SPEED_AUTO_SLOW before the limit
    // switch. Speed changes take effect immediately since acceleration and
    // deceleration are disabled in the drive config below.
    constexpr float SLOWDOWN_FRACTION = 0.08f;

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
}
