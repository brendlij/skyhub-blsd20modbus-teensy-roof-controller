#pragma once

#include <Arduino.h>
#include "BLSD20Driver.h"
#include "Config.h"
#include "Types.h"

// Firmware layer: Motion Controller (Spec Abschnitt 4).
// Owns Direction (Spec Abschnitt 5: "wird ausschließlich durch Motorbefehle
// bestimmt") and turns Zone + Direction into the correct speed
// (Spec Abschnitt 7). Knows nothing about buttons, USB, homing sequencing or
// safety policy -- that's the State Machine's job; this layer only ever does
// what it's told, via the BLSD20 Driver below it.
class MotionController
{
public:
    explicit MotionController(BLSD20Driver& driver);

    // Auto-mode move: speed is derived from `zone` (Spec Abschnitt 7/8).
    bool commandAutoMove(Direction dir, Zone zone);

    // Manual (hold-to-run) and homing moves: fixed speed, no zone logic --
    // matches the existing tuned behavior of both modes.
    bool commandManualMove(Direction dir);
    bool commandHomingMove(Direction dir);

    // Recomputes the target speed from the latest Zone every tick while an
    // auto move is in progress (call only after commandAutoMove()). Applies
    // Spec Abschnitt 8: a move that starts already inside a slow zone starts
    // slow immediately (see commandAutoMove()); this handles the slowdown
    // transition while already moving, smoothed by
    // RoofConfig::ZONE_ENTRY_SLOWDOWN_DELAY_MS.
    //
    // approachSlowdown: set by the State Machine while nearing a PERCENT
    // target (comfort-only, Hall-counter-derived -- see
    // StateMachine::requestMoveToPercent()). OR'd with the physical-zone
    // slowdown so both sources share one speed arbitration instead of
    // racing calls to setSpeed(); unlike the zone case this applies
    // immediately, no entry delay, since it's a smooth precomputed approach
    // rather than a debounced switch trip.
    void update(Zone currentZone, bool approachSlowdown);

    // Soft-stop: drop to slow speed but keep moving (used while coasting
    // through RoofConfig::STOP_SLOWDOWN_DELAY_MS after a STOP request).
    void beginSoftStop();

    bool commandStop(bool emergency);

    Direction direction() const { return _direction; }
    bool isAutoMoveActive() const { return _autoMoveActive; }

private:
    static uint16_t speedForZone(Zone zone, Direction dir);

    BLSD20Driver& _driver;
    Direction _direction = Direction::None;
    Zone _zone = Zone::Unknown;
    bool _autoMoveActive = false;
    bool _slowApplied = false;
    uint32_t _zoneChangedMs = 0;
};
