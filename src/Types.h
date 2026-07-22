#pragma once

#include <Arduino.h>

// 3-position rotary switch. Off is a hard lockout: no OPEN/CLOSE/HOME/manual
// hold is accepted, and any move already running is force-stopped.
enum class RoofMode : uint8_t
{
    Auto,
    Manual,
    Off
};

// Current activity of the roof. This is the top-level state the rest of the
// firmware (LED, telemetry, safety interlocks) keys off of.
enum class Motion : uint8_t
{
    Stopped,
    Opening,
    Closing,
    Homing,
    Fault,
    Disabled // motor lockout; only enterable from Stopped, see StateMachine::requestDisable()
};

// Dach-Endposition. Es gibt bewusst keine Zwischenposition -- siehe
// Firmware Specification v0.1, Abschnitt 5. Use percentOpen() for a
// continuous (Hall-counter-derived, comfort-only) readout.
enum class Position : uint8_t
{
    Unknown,
    Open,
    Closed
};

// Motorbewegungsrichtung. Wird ausschließlich durch Motorbefehle gesetzt,
// niemals aus Sensoren abgeleitet (Spec Abschnitt 5).
enum class Direction : uint8_t
{
    None,
    Opening,
    Closing
};

// Geschwindigkeitszone, ausschließlich aus LIMIT_OPEN/LIMIT_CLOSE/
// SLOW_OPEN/SLOW_CLOSE ermittelt (Spec Abschnitt 6). Latched: once known,
// stays valid across stops until the corresponding switch is crossed again,
// since the slowdown switches are point sensors, not continuous zone
// sensors.
enum class Zone : uint8_t
{
    Unknown,
    OpenSlow,
    Main,
    CloseSlow
};
