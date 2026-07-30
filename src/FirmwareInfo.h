#pragma once

#include <Arduino.h>
#include "Config.h"

// Single source of truth for everything identifying this build and this
// physical controller (Spec Abschnitt 14). SkyHub Core reads it via the INFO
// command to identify devices, verify that a firmware image matches the
// hardware before flashing, and diagnose failures -- so nothing here may be
// duplicated elsewhere in the firmware.
//
// Field-by-field documentation, including type and stability guarantees, is
// in the README under "Firmware metadata and updates". Adding a field means
// touching this file and that table, nothing else.
//
// BUILD_TIMESTAMP and GIT_COMMIT are injected by scripts/build_metadata.py
// (wired up via extra_scripts in platformio.ini). The fallbacks below keep
// the firmware compiling when it is built without that script, e.g. from a
// plain Arduino/CMake setup or an exported source tree with no git history.
#ifndef FW_BUILD_TIMESTAMP
#define FW_BUILD_TIMESTAMP "unknown"
#endif
#ifndef FW_GIT_COMMIT
#define FW_GIT_COMMIT "unknown"
#endif

namespace FirmwareInfo
{
    // ---- Identity -------------------------------------------------------
    // Three deliberately independent identifiers, so none of them has to
    // carry two meanings at once:
    //   DEVICE  human-facing display name, free to change at any time
    //   MODEL   stable hardware model, shared by every revision of this box
    //   PRODUCT stable software identifier for this firmware
    constexpr const char* DEVICE = "SkyHub Roof Controller";
    constexpr const char* MODEL = "roof-controller";
    constexpr const char* PRODUCT = "skyhub-roof-controller";

    // ---- Hardware / compatibility ---------------------------------------
    // BOARD must match the PlatformIO board id in platformio.ini.
    // HARDWARE is the wiring revision documented in the README pin table;
    // HARDWARE_REVISION is the same thing as an integer so SkyHub can do
    // range checks without parsing strings. Bump BOTH together whenever a
    // pin assignment changes (RevA=1, RevB=2, ...).
    constexpr const char* BOARD = "TEENSY41";
    constexpr const char* HARDWARE = "RevA";
    constexpr uint16_t HARDWARE_REVISION = 1;

    // Firmware version. This is the ONLY place it is defined; STATUS reports
    // it as fw_version= from here.
    constexpr const char* VERSION = "0.1.0";

    // Serial protocol version. Bump on any breaking change to the command
    // set or the STATUS/INFO line format, so SkyHub can detect a controller
    // it does not know how to talk to. Additive changes (a new STATUS field,
    // a new command, a new entry in "features") do not count as breaking.
    constexpr uint16_t PROTOCOL = 1;

    // ---- Build provenance -----------------------------------------------
    constexpr const char* BUILD_TIMESTAMP = FW_BUILD_TIMESTAMP; // ISO 8601 UTC
    constexpr const char* GIT_COMMIT = FW_GIT_COMMIT;           // short hash, "-dirty" if the tree was modified

    // ---- Capabilities ----------------------------------------------------
    // What this build can actually do, so SkyHub never has to infer it from
    // a version number. Only capabilities that exist today are listed; the
    // ones tied to a RoofConfig flag genuinely vary between builds, the rest
    // are compiled in unconditionally and exist so a future variant can
    // report false without a protocol change.
    namespace Features
    {
        constexpr bool HOMING = true;         // HOME command and the homing sequence
        constexpr bool RAIN_SENSOR = true;    // rain= in STATUS
        constexpr bool MANUAL_MODE = true;    // hold-to-run via the mode switch
        constexpr bool PERCENT_MOVES = true;  // PERCENT <0-100>
        constexpr bool TEMPERATURE = true;    // temp_mcu/temp_mosfet/temp_brake in STATUS
        constexpr bool WATCHDOG_RELAY = true; // Modbus watchdog relay driving the HARD_STOP chain

        // These two mirror build-time policy that changes what SkyHub may
        // expect from a move request -- see Config.h.
        constexpr bool SLOW_SWITCHES = RoofConfig::AUTO_HAS_SLOW_SWITCHES;
        constexpr bool REQUIRES_HOMING = RoofConfig::AUTO_REQUIRES_HOMING;
    }

    // ---- Runtime identity and diagnostics --------------------------------

    // Unique per-chip id, formatted exactly like the USB serial number the
    // host sees (see usb_init_serialnumber() in the Teensy core), so SkyHub
    // can match an INFO reply against the /dev/serial/by-id path it opened.
    inline uint32_t serialNumber()
    {
#if defined(__IMXRT1062__)
        uint32_t num = HW_OCOTP_MAC0 & 0xFFFFFF;
        if (num < 10000000) num *= 10; // same OS-X CDC-ACM workaround the core applies
        return num;
#else
        return 0;
#endif
    }

    // Cause of the last reset, decoded from the i.MX RT1062 reset status
    // register. Read-only and idempotent -- nothing in this firmware clears
    // SRC_SRSR, so it stays valid for the whole uptime.
    //
    // Teensy 4.1 caveat, important for interpreting this: the MKL02
    // bootloader chip manages 3.3V supervision and the RESET line, so
    // power-on, brownout, a reset pulse and the reboot after a
    // teensy_loader_cli flash ALL arrive as the same plain reset and are
    // reported as "power_on". A firmware update is therefore NOT
    // distinguishable here -- verify updates by comparing firmware/build/git
    // across a reconnect instead. "software" comes from a SCB_AIRCR write
    // (_restart_Teensyduino_), which this firmware never issues.
    inline const char* resetReason()
    {
#if defined(__IMXRT1062__)
        const uint32_t srsr = SRC_SRSR;
        // Several bits can be set at once; report the most specific cause
        // first. The raw register is exposed alongside for the rest.
        if (srsr & SRC_SRSR_TEMPSENSE_RST_B) return "temperature";
        if (srsr & (SRC_SRSR_WDOG_RST_B | SRC_SRSR_WDOG3_RST_B)) return "watchdog";
        if (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ)
        {
            // SRC_GPR5 tells a deliberate software reset apart from the
            // core's auto-reboot on a fault or bad interrupt vector.
            return (SRC_GPR5 == 0x0BAD00F1) ? "fault" : "software";
        }
        if (srsr & SRC_SRSR_CSU_RESET_B) return "security";
        if (srsr & (SRC_SRSR_JTAG_RST_B | SRC_SRSR_JTAG_SW_RST)) return "jtag";
        if (srsr & SRC_SRSR_IPP_USER_RESET_B) return "external";
        if (srsr & SRC_SRSR_IPP_RESET_B) return "power_on";
#endif
        return "unknown";
    }

    inline uint32_t resetRaw()
    {
#if defined(__IMXRT1062__)
        return SRC_SRSR;
#else
        return 0;
#endif
    }

    // ---- JSON serialisation ----------------------------------------------
    // Every string value emitted here is either a compile-time constant we
    // control or a decoded reason from the fixed table above -- none contain
    // quotes or backslashes, so no JSON escaping is needed. Keep it that way
    // when adding fields.

    inline void printKey(Print& out, const char* key)
    {
        out.print('"');
        out.print(key);
        out.print("\":");
    }

    inline void printString(Print& out, const char* key, const char* value)
    {
        printKey(out, key);
        out.print('"');
        out.print(value);
        out.print('"');
    }

    inline void printUInt(Print& out, const char* key, uint32_t value)
    {
        printKey(out, key);
        out.print(value);
    }

    inline void printBool(Print& out, const char* key, bool value)
    {
        printKey(out, key);
        out.print(value ? "true" : "false");
    }

    // One-line JSON object, answering the INFO command.
    inline void printJson(Print& out)
    {
        out.print('{');

        // Identity
        printString(out, "device", DEVICE);          out.print(',');
        printString(out, "model", MODEL);            out.print(',');
        printString(out, "product", PRODUCT);        out.print(',');
        printString(out, "board", BOARD);            out.print(',');
        printKey(out, "serial");
        out.print('"');
        out.print(serialNumber());
        out.print("\",");

        // Compatibility
        printString(out, "hardware", HARDWARE);                     out.print(',');
        printUInt(out, "hardware_revision", HARDWARE_REVISION);     out.print(',');
        printString(out, "firmware", VERSION);                      out.print(',');
        printUInt(out, "protocol", PROTOCOL);                       out.print(',');

        // Build provenance
        printString(out, "build", BUILD_TIMESTAMP);  out.print(',');
        printString(out, "git", GIT_COMMIT);         out.print(',');

        // Diagnostics
        printString(out, "reset_reason", resetReason());  out.print(',');
        printUInt(out, "reset_raw", resetRaw());           out.print(',');

        // Capabilities
        printKey(out, "features");
        out.print('{');
        printBool(out, "homing", Features::HOMING);                  out.print(',');
        printBool(out, "rain_sensor", Features::RAIN_SENSOR);        out.print(',');
        printBool(out, "manual_mode", Features::MANUAL_MODE);        out.print(',');
        printBool(out, "percent_moves", Features::PERCENT_MOVES);    out.print(',');
        printBool(out, "temperature", Features::TEMPERATURE);        out.print(',');
        printBool(out, "watchdog_relay", Features::WATCHDOG_RELAY);  out.print(',');
        printBool(out, "slow_switches", Features::SLOW_SWITCHES);    out.print(',');
        printBool(out, "requires_homing", Features::REQUIRES_HOMING);
        out.print('}');

        out.println('}');
    }
}
