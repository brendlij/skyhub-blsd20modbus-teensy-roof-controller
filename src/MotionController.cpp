#include "MotionController.h"

MotionController::MotionController(BLSD20Driver& driver) : _driver(driver) {}

// Spec Abschnitt 7: slow only while heading *into* the slow zone that
// guards the limit ahead; Main is always fast, and heading *out* of a slow
// zone toward Main is also fast. A still-Unknown zone (e.g. right after
// boot, before either slow switch has been crossed yet -- possible if the
// roof is resting partway through a slow zone rather than right at a
// limit) is deliberately treated as "could be a slow zone" and starts
// slow: the switches are point sensors, so there's no other physical
// signal to fall back on (Hall counter is comfort-only, never safety --
// Spec Abschnitt 8), and slow-when-it-didn't-need-to-be is always the
// safe side of that guess.
uint16_t MotionController::speedForZone(Zone zone, Direction dir)
{
    bool wantSlow = zone == Zone::Unknown ||
                    (zone == Zone::OpenSlow && dir == Direction::Opening) ||
                    (zone == Zone::CloseSlow && dir == Direction::Closing);
    return wantSlow ? RoofConfig::SPEED_AUTO_SLOW : RoofConfig::SPEED_AUTO_FAST;
}

bool MotionController::commandAutoMove(Direction dir, Zone zone)
{
    // Spec Abschnitt 8: a move starting from a stop already inside the
    // relevant slow zone starts slow immediately -- no entry delay, that
    // delay only smooths the transition while already moving (see update()).
    uint16_t speed = RoofConfig::AUTO_HAS_SLOW_SWITCHES ? speedForZone(zone, dir) : RoofConfig::SPEED_AUTO_SLOW;
    if (!_driver.startMove(dir, speed)) return false;

    _direction = dir;
    _zone = zone;
    _autoMoveActive = true;
    _slowApplied = (speed == RoofConfig::SPEED_AUTO_SLOW);
    _zoneChangedMs = millis();
    return true;
}

bool MotionController::commandManualMove(Direction dir)
{
    if (!_driver.startMove(dir, RoofConfig::SPEED_MANUAL)) return false;
    _direction = dir;
    _autoMoveActive = false;
    _slowApplied = false;
    return true;
}

bool MotionController::commandHomingMove(Direction dir)
{
    if (!_driver.startMove(dir, RoofConfig::SPEED_HOMING)) return false;
    _direction = dir;
    _autoMoveActive = false;
    _slowApplied = false;
    return true;
}

void MotionController::update(Zone currentZone, bool approachSlowdown)
{
    if (!_autoMoveActive) return;

    if (currentZone != _zone)
    {
        _zone = currentZone;
        _zoneChangedMs = millis();
    }

    uint16_t zoneDesired = RoofConfig::AUTO_HAS_SLOW_SWITCHES ? speedForZone(_zone, _direction) : RoofConfig::SPEED_AUTO_SLOW;
    bool zoneWantsSlow = zoneDesired == RoofConfig::SPEED_AUTO_SLOW;
    bool wantSlow = zoneWantsSlow || approachSlowdown;

    if (wantSlow && !_slowApplied)
    {
        // approachSlowdown is a smooth, precomputed approach (Hall-counter
        // percent vs. target) -- apply it immediately, skipping the entry
        // delay that only exists to debounce a physical switch trip.
        if (approachSlowdown || millis() - _zoneChangedMs >= RoofConfig::ZONE_ENTRY_SLOWDOWN_DELAY_MS)
        {
            _driver.setSpeed(RoofConfig::SPEED_AUTO_SLOW);
            _slowApplied = true;
        }
    }
    else if (!wantSlow && _slowApplied)
    {
        // Leaving a slow zone toward Main: speed back up immediately, no
        // need to smooth an acceleration the way we smooth a deceleration.
        _driver.setSpeed(RoofConfig::SPEED_AUTO_FAST);
        _slowApplied = false;
    }
}

void MotionController::beginSoftStop()
{
    _driver.setSpeed(RoofConfig::SPEED_AUTO_SLOW);
}

bool MotionController::commandStop(bool emergency)
{
    bool ok = emergency ? _driver.emergencyStop() : _driver.stop();
    _direction = Direction::None;
    _autoMoveActive = false;
    _slowApplied = false;
    return ok;
}
