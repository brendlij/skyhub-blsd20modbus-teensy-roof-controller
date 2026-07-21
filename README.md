# SkyHub BLSD20 Teensy Controller

Roof/blind controller on a Teensy 4.1, driving a BLSD20 BLDC motor controller over Modbus RTU (RS-485). See [src/RoofController.h](src/RoofController.h) for behavior and [src/Config.h](src/Config.h) for all tunables — this file just documents the wiring.

## Pinout

| Pin     | Signal                | Notes                                                                                                                                                                                   |
| ------- | --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0 (RX1) | Modbus RS-485 RX      | Waveshare TTL-to-RS485 (B), auto direction switching — no DE pin needed                                                                                                                 |
| 1 (TX1) | Modbus RS-485 TX      | same converter                                                                                                                                                                          |
| 2       | BTN_CLOSE             | edge-triggered in Auto, hold-to-run in Manual                                                                                                                                           |
| 3       | BTN_STOP              |                                                                                                                                                                                         |
| 4       | BTN_OPEN              | edge-triggered in Auto, hold-to-run in Manual                                                                                                                                           |
| 5       | MODE_SWITCH_AUTO      | left position of the 3-position rotary switch: C → GND, NO → this pin                                                                                                                   |
| 6       | LIMIT_CLOSE           | inductive, operational stop (Auto mode), read by Teensy                                                                                                                                 |
| 7       | LIMIT_OPEN            | inductive, operational stop (Auto mode), read by Teensy                                                                                                                                 |
| 8       | LED_R                 | status RGB LED, common cathode                                                                                                                                                          |
| 9       | LED_G                 | status RGB LED                                                                                                                                                                          |
| 10      | LED_B                 | status RGB LED                                                                                                                                                                          |
| 11      | LED_BUTTON            | toggles status LED on/off                                                                                                                                                               |
| 12      | MODE_SWITCH_MANUAL    | right position of the rotary switch: C → GND, NO → this pin; center position grounds neither → `RoofMode::Off` (lockout)                                                                |
| 14      | SLOW_CLOSE            | inductive, inboard of LIMIT_CLOSE — triggers slowdown to `SPEED_AUTO_SLOW`                                                                                                              |
| 15      | SLOW_OPEN             | inductive, inboard of LIMIT_OPEN — triggers slowdown to `SPEED_AUTO_SLOW`                                                                                                               |
| 16      | MODBUS_WATCHDOG_RELAY | output → relay module `IN`; energized (`HIGH`) only while Modbus comms are healthy                                                                                                      |
| 17      | CASE_FAN_RELAY        | output → relay module `IN`; fan wired to the relay's NC contact, so `LOW` = fan on (default at boot), `HIGH` = fan off. Toggled via the serial `FAN ON`/`FAN OFF`/`FAN TOGGLE` commands |

All digital inputs are active-low with internal pull-ups (`INPUT_PULLUP`), debounced in software (see [src/DebouncedInput.h](src/DebouncedInput.h)).

## Endschalter (6 total, 3 per side)

Per side (Open/Close), from inboard to outboard:

1. **Slow switch** (`SLOW_OPEN` pin 15 / `SLOW_CLOSE` pin 14) — marks where an Auto move drops from `SPEED_AUTO_FAST` to `SPEED_AUTO_SLOW`. Full speed is allowed anywhere between the two. Optional: while `RoofConfig::AUTO_HAS_SLOW_SWITCHES = false` (default), these aren't required — an Auto move just stays at `SPEED_AUTO_SLOW` for the whole travel instead of ever using `SPEED_AUTO_FAST`. Set the flag `true` once they're wired up to get real fast/slow moves.
2. **Operational limit switch** (`LIMIT_OPEN` pin 7 / `LIMIT_CLOSE` pin 6) — normal software stop point in Auto mode, and the homing reference points. This is the switch that actually stops an Auto move; `RoofConfig::AUTO_REQUIRES_HOMING = false` (default) lets `OPEN`/`CLOSE` work off these alone, without a prior `HOME` run — `percent` just stays `-1` and no calibration gets saved until homing actually succeeds. In Manual mode, `RoofConfig::MANUAL_RESPECTS_LIMIT_SWITCHES = false` (default) makes hold-to-run moves ignore both limit switches entirely — only releasing the button or `STOP` stops them. Set `true` to restore the software cutoff in Manual too. The outer hardware E-stop below is unaffected either way.
3. **Hardware E-stop switch** (outermost, one per side) — **not wired to the Teensy at all.** Wired directly into a hardware series loop, independent of software/Modbus:

   ```
   Endschalter-Open(outer) -> Endschalter-Close(outer) -> MODBUS_WATCHDOG_RELAY contact -> E-stop button -> BLSD20 HARD_STOP input
   ```

   Any break in this chain (over-travel past the operational limit, E-stop pressed, or the watchdog relay dropping out) hard-stops the motor directly at the BLSD20, with no Teensy or Modbus involvement. The Teensy still polls `DI_HARD_STOP` (`RoofController::hardStopActive()`) purely for status reporting upstream (see `hardstop=` in the serial `STATUS` line).

   The two outer E-stop limit switches use their **NC** contact (closed at rest, opens on trip or on a broken wire — fails safe either way).

## Homing

Triggered by the serial `HOME` command, or in Manual mode by holding `BTN_OPEN`+`BTN_CLOSE` together (from `Idle`) for `HOME_COMBO_HOLD_MS` (3s) — no serial link needed.

1. **SeekClose**: drives `DIR_CLOSE` at `SPEED_HOMING` until `LIMIT_CLOSE` trips, then resets the BLSD20's position counter to 0 there (`closedPositionCounts = 0`).
2. Pauses `HOMING_LEG_PAUSE_MS` (500 ms) — gives the BLSD20 time to settle; issuing the next `start()` immediately after `stop()`+position-reset can otherwise get silently dropped.
3. **SeekOpen**: drives `DIR_OPEN` at `SPEED_HOMING` until `LIMIT_OPEN` trips, recording `openPositionCounts`. If the travel between the two ends is under `MIN_HOMING_RANGE_COUNTS`, the run is rejected as bogus (`homing range too small` fault) — e.g. a switch that never triggered.
4. On success: `homed=1`, and both counts are saved to EEPROM (survives Teensy reboots/reflashes).

The whole run is capped at `HOMING_TIMEOUT_MS`; timing out enters `Fault`. The hardware E-stop loop above still applies throughout, independent of any of this.

### Calibration drift & self-healing

The BLSD20's own position counter is volatile — it resets on every BLSD20 restart (`RESTART`, `SAVE`, or the automatic post-boot one below), which can desync it from the EEPROM-stored `closedPositionCounts`/`openPositionCounts`. This is flagged as `calib_stale=1` (doesn't block `OPEN`/`CLOSE` — `percent`/`pos` may just be inaccurate until it clears). It self-heals automatically the next time either operational limit switch is actually reached during normal use (`RoofController::refreshCalibrationAtLimit`) — no dedicated re-`HOME` needed.

## Watchdog relay module

`MODBUS_WATCHDOG_RELAY` (pin 16) drives a 1-channel opto-isolated relay module (`SRD-DC03V-SL-C` relay, `EL817` opto, 3–3.3 V logic, high-level trigger — e.g. the common "3V relay module for ESP8266"):

- `VCC` → Teensy `3V3` (not 5V), `GND` → Teensy `GND`, `IN` → pin 16
- `JP1`/`JP2` jumpers left in place (single supply; no need for the logic/coil sides to be separately powered here)
- Module is high-level triggered, matching the code's `HIGH` = energized/healthy convention directly — no inversion needed
- Wired into the E-stop chain via its **COM + NO** contacts (closed only while energized/healthy), see above

### Boot recovery

The watchdog relay pin starts `LOW` at boot (fail-safe) until Modbus comms are proven healthy — this briefly breaks the E-stop loop above and latches `EmergencyStop` on the BLSD20, on *every* Teensy boot/reflash, not just a real fault. `BOOT_AUTO_RESTART_DELAY_MS` (2s) after boot, the controller automatically sends a `RESTART` to the BLSD20 to clear that — no manual `RESTART` needed after a flash or power cycle. This also sets `calib_stale=1`, same as a manual `RESTART` (see Homing above).

## Status LED

3-pin common-cathode RGB LED (e.g. AZDelivery KY-016) on pins 8/9/10, driven via `RoofController` state — see [src/StatusLed.cpp](src/StatusLed.cpp):

| Color           | Meaning                                                                  |
| --------------- | ------------------------------------------------------------------------ |
| Solid green     | Idle, homed                                                              |
| Blinking yellow | Idle but not yet homed, or homing in progress                            |
| Solid blue      | Opening                                                                  |
| Solid magenta   | Closing                                                                  |
| Blinking white  | Restarting the BLSD20                                                    |
| Blinking red    | Fault — overrides everything else, even if the LED has been switched off |
| Off             | LED switched off (`LED_BUTTON` or `LED OFF`), and no fault               |

`LED_BUTTON` (pin 11) or the serial `LED ON`/`LED OFF`/`LED TOGGLE` commands toggle the LED off/on for cosmetic reasons (e.g. it shining into a room at night); a Fault always blinks red regardless.

## Motor controller (BLSD20)

- RS-485 Modbus RTU, slave ID `RoofConfig::MODBUS_SLAVE_ID`, 115200 baud, 8E1.
- Pole count, current limit, accel/decel etc. are re-applied every boot — see `RoofController::applyDefaultConfig()`.

## Host link

USB serial (115200 baud) to a Raspberry Pi 5, line-based, newline-terminated, case-insensitive. Every command gets exactly one reply line: `OK <CMD>` or `ERR <reason> <CMD>`. See [src/SerialLink.cpp](src/SerialLink.cpp).

### Commands

| Command                             | Effect                                                                        | Rejected when                                                                                           |
| ----------------------------------- | ----------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| `OPEN`                              | Starts an Auto-mode opening move                                              | not in `RoofMode::Auto`, not `Idle`, not homed (only if `RoofConfig::AUTO_REQUIRES_HOMING`), `LIMIT_OPEN` already active, or motor has an error flag |
| `CLOSE`                             | Starts an Auto-mode closing move                                              | same as `OPEN`, mirrored for the close side                                                             |
| `STOP`                              | If `Opening`/`Closing`: soft stop — drops to `SPEED_AUTO_SLOW`, then a real stop after `STOP_SLOWDOWN_DELAY_MS`. Otherwise: stops the motor immediately, clears any fault, returns to `Idle` | never — always succeeds (immediately, except while `Opening`/`Closing`)                                                                                 |
| `HOME`                              | Runs the homing sequence (see Homing section above; also triggerable in Manual by holding `BTN_OPEN`+`BTN_CLOSE`) | `RoofMode::Off`, not `Idle`, or motor has an error flag                                                 |
| `SAVE`                              | One-time provisioning: `saveSettings()` + `restart()` on the BLSD20           | not `Idle`                                                                                              |
| `RESTART`                           | Reboots the BLSD20 only, then re-applies the runtime config once it's back up | not `Idle` and not `Fault`                                                                              |
| `LED ON` / `LED OFF` / `LED TOGGLE` | Controls the status RGB LED                                                   | never                                                                                                   |
| `FAN ON` / `FAN OFF` / `FAN TOGGLE` | Controls the case fan relay (see `CASE_FAN_RELAY`, pin 17)                    | never                                                                                                   |
| `STATUS`                            | Prints a `STATUS ...` line immediately (no `OK`/`ERR` reply)                  | never                                                                                                   |
| _(anything else)_                   | —                                                                             | `ERR unknown_command <CMD>`                                                                             |

### Telemetry

A `STATUS ...` line is pushed unprompted every `STATUS_REPORT_MS` (500 ms) and immediately on any state change, in addition to being sent in reply to an explicit `STATUS` command:

```
STATUS mode=AUTO state=IDLE homed=1 calib_stale=0 percent=100 pos=48213 speed=0 current=0 comm_fail=0 temp_mcu=32.5 temp_mosfet=30.1 temp_brake=28.4 led=1 fan=1 hardstop=0 dir_cmd=BACKWARD dir_motor=STOPPED btn_open=0 btn_stop=0 btn_close=0 sw_auto=1 sw_manual=0 limit_open=1 limit_close=0 slow_open=0 slow_close=0
```

| Field                | Meaning                                                                                       |
| -------------------- | --------------------------------------------------------------------------------------------- |
| `mode`               | `AUTO` / `MANUAL` / `OFF`                                                                     |
| `state`              | `UNINITIALIZED` / `IDLE` / `HOMING` / `OPENING` / `CLOSING` / `FAULT` / `RESTARTING`          |
| `homed`              | `1` once a valid homing run has completed                                                     |
| `calib_stale`        | `1` after a `RESTART`, until the roof next fully closes and re-zeroes                         |
| `percent`            | 0–100, or `-1` if not homed                                                                   |
| `pos`                | raw motor position counts                                                                     |
| `speed`              | motor speed, rpm                                                                              |
| `current`            | motor current, mA                                                                             |
| `comm_fail`          | consecutive failed Modbus polls (0..`MAX_COMM_FAILURES`); hits the limit -> `FAULT`           |
| `temp_mcu`/`temp_mosfet`/`temp_brake` | BLSD20 temperatures, °C                                                     |
| `led`                | status LED on/off                                                                             |
| `fan`                | case fan on/off                                                                               |
| `hardstop`           | `1` if the BLSD20's `HARD_STOP` input is currently asserted (hardware E-stop loop tripped)    |
| `dir_cmd`            | `FORWARD`/`BACKWARD` — direction last commanded by the software (`BACKWARD` = `DIR_OPEN`)     |
| `dir_motor`          | `STOPPED`/`FORWARD`/`BACKWARD` — what the BLSD20 itself reports it's doing right now; compare against `dir_cmd` to spot a reversed motor/wiring |
| `btn_open`/`btn_stop`/`btn_close` | live (debounced) button states                                                  |
| `sw_auto`/`sw_manual`| live 3-position rotary switch legs (both `0` = center/`Off`)                                  |
| `limit_open`/`limit_close` | live operational limit switch states                                                   |
| `slow_open`/`slow_close`   | live slowdown switch states                                                            |
| `fault` / `errflags` | only present when `state=FAULT`: human-readable reason and the raw BLSD20 error bitmask (hex) |
