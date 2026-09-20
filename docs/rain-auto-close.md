# Autonomous rain closure (feature branch)

Rain at pin 18 is debounced, then must remain active for RAIN_CONFIRM_MS
(default 5000 ms). In AUTO the controller closes to LIMIT_CLOSE, independent
of USB/SkyHub. An opening/homing movement is stopped first; reversal requires
a successful stop command, valid zero-RPM telemetry and 1500 ms settling.
Failure to establish standstill within 10 s terminates the attempt.
An existing closing percentage move is promoted to the full closed endpoint.
The existing 64 s MOVE_TIMEOUT_MS remains unchanged (including an existing
closing move's elapsed time). No opening/homing/percentage commands during rain.

STOP cancels an active rain attempt. Faults and aborted attempts do not restart
on RESETFAULT, reconnect or a mode change. A new attempt is armed only after
30 s continuously dry followed by confirmed rain. Drying during closure does
not cancel the close. No automatic reopening. Already closed is confirmed
without moving. OFF, MANUAL, DISABLED and boot/reapply gates are respected.
Manual hold-to-run behavior is unchanged; rain automation runs only in AUTO.
While initially blocked, the attempt waits for AUTO/readiness/clearance.

## Optional scope clearance

SCOPE_SAFE_PIN=-1 disables the additional GPIO clearance (default). Configure
a free pin in Config.h only after checking the wiring/pin assignment. Active
LOW with INPUT_PULLUP means an open/disconnected contact denies clearance.
The gate applies to AUTO CLOSE, percentage moves and HOME; lost clearance
during an AUTO close/homing enters Fault. Manual motion retains existing rules.
RAIN_AUTO_CLOSE=true enables the feature; set false to retain reporting only.
These options are compile-time, not runtime USB settings.

## STATUS extension / SkyHub integration contract

Existing fields remain unchanged. Additional fields:

```
rain_auto=1 close_allowed=1 close_reason=rain close_phase=closing close_block=none close_id=1
```

close_phase: idle, pending, stopping, blocked, closing, closed, interrupted,
error. close_block is a single token (wrong_mode, not_ready, scope_not_safe,
controller_fault, stop_timeout, stop_failed, stop or the rejected command reason).
closed requires the physical endpoint and stopped logical motion, never percent.
These describe the LAST rain episode, not every manual close; manual actions
do not erase the reason. They survive USB reconnect, not a Teensy reset.
close_id is a per-boot episode counter, NOT globally unique or persisted.
SkyHub must detect reset/reconnect and reconcile snapshots; it must not infer
new movements solely from a counter increase. No new unsolicited EVENT format.

SkyHub follow-up (not part of this firmware change): parse these fields,
store transitions with source=controller/reason=rain, notify once per transition,
and distinguish blocked/failed from closed. Existing rain=1 already feeds the
Alpaca SafetyMonitor; Dome state continues to come from physical roof status.
The classic SafetyMonitor communicates IsSafe, not the detailed closure reason.

Future software scope permission: define an explicit opt-in, expiring permission
lease/heartbeat with sequence and timeout, boot denied, invalidated on expiry.
Do not treat USB connectivity or a stale NINA parked value as permission.
No software permission command has been introduced in this branch.

## Verification before deployment

Build: `pio run -e teensy41` (no upload).
Host policy tests (use a desktop compiler, not arm-none-eabi g++): `g++ -std=c++11 -Wall -Wextra -Werror test/rain_policy.cpp -o rain_policy_test` then run it.
Bench-test with motor mechanically isolated: wet at boot, rain while opening,
close endpoint, STOP, OFF/manual, scope loss, Modbus failure, speed read failure,
timeout and reconnect. Verify sensor polarity and physical stopping distance.
The policy tests and firmware compilation are not hardware validation.

Windows/MSVC: from a Developer Command Prompt run `cl /EHsc /W4 /WX test\rain_policy.cpp /Fo.pio\rain_policy.obj /Fe.pio\rain_policy_test.exe`, then `.pio\rain_policy_test.exe`.
