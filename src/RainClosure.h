#pragma once
#include <stdint.h>

// Pure policy; no I/O. One attempt per wet episode. STOP/fault are latched
// until a sustained dry interval followed by new rain. Times use wrap-safe deltas.
class RainClosure {
public:
    enum class Action { None, Stop, Close };
    struct Inputs {
        bool wet, automatic, ready, fault, closed, closing, moving, stationary, safe;
    };
    RainClosure(uint32_t wetMs, uint32_t dryMs, uint32_t settleMs, uint32_t stopTimeout)
        : wetMs(wetMs), dryMs(dryMs), settleMs(settleMs), stopTimeout(stopTimeout) {}
    const char* phase = "idle";
    const char* block = "none";
    uint32_t id = 0;
    bool active() const { return state == Pending || state == Stopping || state == Closing; }
    bool locked() const { return wet || active(); }
    void cancel() { if (active()) { state = Terminal; phase = "interrupted"; block = "stop"; } }
    void fail(const char* why) { state = Terminal; phase = "error"; block = why; }
    void accepted() { state = Closing; phase = "closing"; block = "none"; }
    Action tick(uint32_t now, Inputs i) {
        wet = i.wet;
        if (i.wet != lastWet) { lastWet = i.wet; edgeAt = now; }
        if (!i.wet && uint32_t(now-edgeAt) >= dryMs && !active()) armed = true;
        if (i.wet && armed && uint32_t(now-edgeAt) >= wetMs) {
            armed = false; ++id; state = Pending; phase = "pending"; block = "none";
        }
        if (!active()) return Action::None;
        // A pending attempt goes stale once the rain that triggered it is
        // long gone: closing for it would be a surprise movement, and
        // holding the open lockout would leave the roof shut in dry
        // weather. Same threshold that arms a new attempt, so one expires
        // exactly when a fresh one could begin.
        if (state == Pending && !i.wet && uint32_t(now-edgeAt) >= dryMs) {
            state = Terminal; phase = "expired"; block = "none";
            return Action::None;
        }
        if (i.fault) { fail("controller_fault"); return Action::None; }
        if (i.closed && !i.moving) { state = Terminal; phase = "closed"; block = "none"; return Action::None; }
        if (!i.automatic || !i.ready || !i.safe) {
            phase = "blocked";
            block = !i.automatic ? "wrong_mode" : !i.ready ? "not_ready" : "scope_not_safe";
            if (state != Pending) { state = Terminal; return Action::Stop; }
            if (i.automatic && i.moving) return Action::Stop;
            return Action::None;
        }
        if (state == Closing) {
            if (!i.closing) { cancel(); }
            return Action::None;
        }
        if (state == Pending) {
            // A partial close becomes a full close via requestClose().
            if (i.closing) { phase = "pending"; return Action::Close; }
            state = Stopping; stopAt = now; phase = "stopping"; block = "none";
            return Action::Stop;
        }
        if (uint32_t(now-stopAt) >= stopTimeout) { fail("stop_timeout"); return Action::None; }
        if (!i.moving && i.stationary && uint32_t(now-stopAt) >= settleMs) return Action::Close;
        return Action::None;
    }
private:
    enum State { Idle, Pending, Stopping, Closing, Terminal } state = Idle;
    uint32_t wetMs, dryMs, settleMs, stopTimeout, edgeAt = 0, stopAt = 0;
    bool armed = true, wet = false, lastWet = false;
};
