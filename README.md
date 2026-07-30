# SkyHub BLSD20 Teensy Controller

Roof/blind controller on a Teensy 4.1, driving a BLSD20 BLDC motor controller over Modbus RTU (RS-485). The firmware is layered per `Teensy Controller Logik.pdf` (Firmware Specification v0.1):

```
USB / Buttons -> Command Layer -> State Machine -> Motion Controller -> BLSD20 Driver -> BLSD20Modbus -> Motor
```

| Layer             | File                                                              | Responsibility                                                                                   |
| ----------------- | ------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------- |
| Command Layer     | [src/CommandLayer.h](src/CommandLayer.h)                          | Buttons + mode switch → the same abstract requests USB issues                                     |
| (USB transport)    | [src/SerialLink.h](src/SerialLink.h)                               | Line protocol to the Pi; forwards movement commands to the Command Layer, prints telemetry        |
| State Machine     | [src/StateMachine.h](src/StateMachine.h)                           | The single source of truth: `RoofMode`/`Motion`/`Position`/`Zone`, safety rules, homing, faults    |
| Motion Controller | [src/MotionController.h](src/MotionController.h)                  | `Direction` + Zone → speed (Spec Abschnitt 7/8)                                                    |
| BLSD20 Driver     | [src/BLSD20Driver.h](src/BLSD20Driver.h)                           | Thin wrapper around `BLSD20Modbus`: config, motion primitives, telemetry caching                  |

Every tunable lives in [src/Config.h](src/Config.h) — this file documents the wiring and the resulting behavior.

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
| 14      | SLOW_CLOSE            | inductive, inboard of LIMIT_CLOSE — marks the boundary of the Close Slow Zone                                                                                                          |
| 15      | SLOW_OPEN             | inductive, inboard of LIMIT_OPEN — marks the boundary of the Open Slow Zone                                                                                                            |
| 16      | MODBUS_WATCHDOG_RELAY | output → relay module `IN`; energized (`HIGH`) only while Modbus comms are healthy                                                                                                      |
| 17      | CASE_FAN_RELAY        | output → relay module `IN`; fan wired to the relay's NC contact, so `LOW` = fan on (default at boot), `HIGH` = fan off. Toggled via the serial `FAN ON`/`FAN OFF`/`FAN TOGGLE` commands |
| 18      | RAIN_SENSOR           | rain contact plate via optocoupler; reported upstream as `rain=` in the `STATUS` line, never acted on by the firmware itself                                                            |

All digital inputs are debounced in software (see [src/DebouncedInput.h](src/DebouncedInput.h)) and active-low with internal pull-ups (`INPUT_PULLUP`). `DebouncedInput::begin()` takes an optional `inverted` flag that makes an input active-high; the pull-up stays on either way, so an inverted input works whether its source drives `HIGH` or just goes high-Z, and a broken wire reads active. Only `RAIN_SENSOR` uses it, via `RoofConfig::RAIN_SENSOR_INVERTED` (currently `true`, i.e. `HIGH` = rain — the plate measures `LOW` while dry).

## Zones and speed

`Zone` (`UNKNOWN`/`OPEN_SLOW`/`MAIN`/`CLOSE_SLOW`) is derived purely from the four physical switches (Spec Abschnitt 6):

```
LIMIT_OPEN
│
├──────── Open Slow Zone ────────┐
│                                │
SLOW_OPEN                        │
│                                │
├──────────── Main ──────────────┤
│                                │
SLOW_CLOSE                       │
│                                │
├────── Close Slow Zone ─────────┤
│                                │
LIMIT_CLOSE
```

`SLOW_OPEN`/`SLOW_CLOSE` are point sensors (inductive proximity switches), not continuous zone sensors, so `StateMachine` latches which side of each we're on whenever it's crossed (combined with the commanded `Direction` — never inferred from the Hall counter, see below), and that latched `Zone` survives a stop. Speed is then a pure function of `Zone` + `Direction` (Spec Abschnitt 7/8, see `MotionController::speedForZone`): slow only while heading *into* the slow zone that guards the limit ahead of you; `Main`, or heading back out toward `Main`, is always fast. A move that starts already inside the relevant slow zone starts slow immediately, no ramp.

`RoofConfig::AUTO_HAS_SLOW_SWITCHES = false` disables this and keeps every Auto move at `SPEED_AUTO_SLOW` for the whole travel (no `SLOW_OPEN`/`SLOW_CLOSE` wired up yet). `RoofConfig::MANUAL_RESPECTS_LIMIT_SWITCHES = false` (default) makes Manual hold-to-run moves ignore the operational limit switches entirely — only releasing the button or `STOP` stops them; Manual and Homing both always move at their own fixed speed (`SPEED_MANUAL`/`SPEED_HOMING`), independent of Zone. The hardware E-stop loop below is unaffected either way.

### Hardware E-stop (outermost switch per side, not wired to the Teensy)

Wired directly into a hardware series loop, independent of software/Modbus:

```
Endschalter-Open(outer) -> Endschalter-Close(outer) -> MODBUS_WATCHDOG_RELAY contact -> E-stop button -> BLSD20 HARD_STOP input
```

Any break in this chain (over-travel past the operational limit, E-stop pressed, or the watchdog relay dropping out) hard-stops the motor directly at the BLSD20, with no Teensy or Modbus involvement. The Teensy still polls `DI_HARD_STOP` (`StateMachine::hardStopActive()`) purely for status reporting upstream (see `hardstop=` in the serial `STATUS` line). The two outer E-stop limit switches use their **NC** contact (closed at rest, opens on trip or on a broken wire — fails safe either way).

## Homing

Triggered by the serial `HOME` command, or in Manual mode by holding `BTN_OPEN`+`BTN_CLOSE` together (from `Stopped`) for `HOME_COMBO_HOLD_MS` (3s) — no serial link needed.

1. **SeekClose**: drives `DIR_CLOSE` at `SPEED_HOMING` until `LIMIT_CLOSE` trips, then resets the BLSD20's Hall counter to 0 there (`closedPositionCounts = 0`).
2. Pauses `HOMING_LEG_PAUSE_MS` (500 ms) — gives the BLSD20 time to settle; issuing the next `start()` immediately after `stop()`+position-reset can otherwise get silently dropped.
3. **SeekOpen**: drives `DIR_OPEN` at `SPEED_HOMING` until `LIMIT_OPEN` trips, recording `openPositionCounts`. If the travel between the two ends is under `MIN_HOMING_RANGE_COUNTS`, the run is rejected as bogus (`homing range too small` fault) — e.g. a switch that never triggered.
4. On success: `homed=1`, and both counts are saved to EEPROM (survives Teensy reboots/reflashes).

The whole run is capped at `HOMING_TIMEOUT_MS`; timing out enters `Fault`. The hardware E-stop loop above still applies throughout, independent of any of this.

### Hall Counter — comfort only, never safety

Per Spec Abschnitt 8/16, the BLSD20's Hall-counter position is used **only** for `percent`/UI/Alpaca/SkyHub — never for `Position`, `Zone`, stopping, or slowdown, all of which come solely from the four physical switches above. It resets on every BLSD20 restart (`RESTART`, `RESETFAULT`, or the automatic post-boot one below), which can desync it from the EEPROM-stored `closedPositionCounts`/`openPositionCounts`. This is flagged as `calib_stale=1` (doesn't block `OPEN`/`CLOSE` — `percent`/`hall` may just be inaccurate until it clears) and self-heals the next time either operational limit switch is actually reached during normal use (`StateMachine::refreshCalibrationAtLimit`) — no dedicated re-`HOME` needed.

## Watchdog relay module

`MODBUS_WATCHDOG_RELAY` (pin 16) drives a 1-channel opto-isolated relay module (`SRD-DC03V-SL-C` relay, `EL817` opto, 3–3.3 V logic, high-level trigger — e.g. the common "3V relay module for ESP8266"):

- `VCC` → Teensy `3V3` (not 5V), `GND` → Teensy `GND`, `IN` → pin 16
- `JP1`/`JP2` jumpers left in place (single supply; no need for the logic/coil sides to be separately powered here)
- Module is high-level triggered, matching the code's `HIGH` = energized/healthy convention directly — no inversion needed
- Wired into the E-stop chain via its **COM + NO** contacts (closed only while energized/healthy), see above

### Boot recovery

The watchdog relay pin starts `LOW` at boot (fail-safe) until Modbus comms are proven healthy — this briefly breaks the E-stop loop above and latches `EmergencyStop` on the BLSD20, on *every* Teensy boot/reflash, not just a real fault. `BOOT_AUTO_RESTART_DELAY_MS` (2s) after boot, the controller automatically sends a `RESTART` to the BLSD20 to clear that — no manual `RESTART` needed after a flash or power cycle. This also sets `calib_stale=1`, same as a manual `RESTART`/`RESETFAULT` (see Hall Counter above). No Fahrbefehle (`OPEN`/`CLOSE`/`HOME`/manual hold) are accepted until this completes (Spec Abschnitt 12).

## Status LED

3-pin common-cathode RGB LED (e.g. AZDelivery KY-016) on pins 8/9/10, driven off `Motion` — see [src/StatusLed.cpp](src/StatusLed.cpp):

| Color           | Meaning                                                                  |
| --------------- | ------------------------------------------------------------------------ |
| Solid green     | Stopped, homed                                                          |
| Blinking yellow | Stopped but not yet homed, or homing in progress                        |
| Solid blue      | Opening                                                                  |
| Solid magenta   | Closing                                                                  |
| Solid cyan      | Disabled (motor lockout, see `ENABLE`/`DISABLE` below)                   |
| Blinking white  | Re-applying config after a `RESTART`/`RESETFAULT`/boot                   |
| Blinking red    | Fault — overrides everything else, even if the LED has been switched off |
| Off             | LED switched off (`LED_BUTTON` or `LED OFF`), and no fault               |

`LED_BUTTON` (pin 11) or the serial `LED ON`/`LED OFF`/`LED TOGGLE` commands toggle the LED off/on for cosmetic reasons (e.g. it shining into a room at night); a Fault always blinks red regardless.

## Motor controller (BLSD20)

- RS-485 Modbus RTU, slave ID `RoofConfig::MODBUS_SLAVE_ID`, 115200 baud, 8E1.
- Pole count, current limit, accel/decel etc. are re-applied every boot and after every restart — see `BLSD20Driver::applyConfig()`. `BLSD20Modbus` exposes no getters for most of these settings, so unlike the position/status telemetry, they're written unconditionally rather than compared against a read-back value first — the writes are idempotent and cheap, so this is safe.

## Host link

USB serial (115200 baud) to a Raspberry Pi 5, line-based, newline-terminated, case-insensitive. Every command gets exactly one reply line: `OK <CMD>` or `ERR <reason> <CMD>` — except the query commands `STATUS` and `INFO`, which answer with their own payload line instead. See [src/SerialLink.cpp](src/SerialLink.cpp). Movement/system commands are dispatched through the Command Layer, so USB and the physical buttons obey identical guards — SkyHub never implements its own movement logic (Spec Abschnitt 2).

### Commands

| Command                             | Effect                                                                        | Rejected when                                                                                           |
| ------------------------------------ | ------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------- |
| `OPEN`                               | Starts an Auto-mode opening move                                              | not in `RoofMode::Auto`, not `Stopped`, not homed (only if `RoofConfig::AUTO_REQUIRES_HOMING`), `Position=OPEN` already, or motor has an error flag |
| `CLOSE`                              | Starts an Auto-mode closing move                                              | same as `OPEN`, mirrored for the close side                                                              |
| `STOP`                               | If `Opening`/`Closing`: soft stop — drops to `SPEED_AUTO_SLOW`, then a real stop after `STOP_SLOWDOWN_DELAY_MS`. Otherwise: stops the motor immediately | never — always succeeds                                                                                  |
| `HOME`                               | Runs the homing sequence (see Homing section above; also triggerable in Manual by holding `BTN_OPEN`+`BTN_CLOSE`) | `RoofMode::Off`, not `Stopped`, or motor has an error flag                                                |
| `ENABLE`                             | Leaves `Disabled`, returns to `Stopped`                                        | not currently `Disabled`                                                                                 |
| `DISABLE`                            | Motor lockout: rejects `OPEN`/`CLOSE`/`HOME` until `ENABLE`d again             | not currently `Stopped`                                                                                  |
| `RESETFAULT`                         | Clears a `Fault` by rebooting the BLSD20 (clears its latched error register) and re-applying config | not currently `Fault`                                                                     |
| `RESTART`                            | Reboots the BLSD20 deliberately (e.g. after a wiring change), then re-applies config once it's back up | not `Stopped`, `Disabled`, or `Fault`                                                     |
| `FWUPDATE`                           | Replies `OK FWUPDATE`, waits `FWUPDATE_ACK_GRACE_MS` so the reply reaches the host, then drops the Teensy into its USB bootloader (Spec Abschnitt 14); flashing itself is done externally by SkyHub/`teensy_loader_cli` | currently `Opening`/`Closing`/`Homing`                                          |
| `PERCENT <0-100>`                    | Starts an Auto-mode move toward the given Hall-counter-derived `percent`; drops to `SPEED_AUTO_SLOW` once within `RoofConfig::PERCENT_APPROACH_BAND` points of the target. Best-effort/comfort only — see Hall Counter note below; the physical limit switches remain the real stop for `0`/`100`. A plain `OPEN`/`CLOSE` while a `PERCENT` move is in progress drops the target and continues to the full limit | not in `RoofMode::Auto`, not `Stopped`, not homed (always, regardless of `AUTO_REQUIRES_HOMING`), out-of-range value, `Position` already past the target's limit, or motor has an error flag |
| `LED ON` / `LED OFF` / `LED TOGGLE`  | Controls the status RGB LED                                                    | never                                                                                                    |
| `FAN ON` / `FAN OFF` / `FAN TOGGLE`  | Controls the case fan relay (see `CASE_FAN_RELAY`, pin 17)                     | never                                                                                                    |
| `STATUS`                             | Prints a `STATUS ...` line immediately (no `OK`/`ERR` reply)                   | never                                                                                                    |
| `INFO`                               | Prints the build metadata as one line of JSON (no `OK`/`ERR` reply) — see Firmware metadata below | never                                                                                 |
| _(anything else)_                    | —                                                                              | `ERR unknown_command <CMD>`                                                                              |

### Telemetry

A `STATUS ...` line is pushed unprompted every `STATUS_REPORT_MS` (500 ms) and immediately on any `Motion` change, in addition to being sent in reply to an explicit `STATUS` command:

```
STATUS mode=AUTO motion=STOPPED position=OPEN direction=NONE zone=OPEN_SLOW percent=100 target_percent=-1 hall=48213 speed=0 target_speed=0 current=0 motor_enabled=1 homed=1 calib_stale=0 modbus_connected=1 comm_fail=0 temp_mcu=32.5 temp_mosfet=30.1 temp_brake=28.4 led=1 fan=1 hardstop=0 btn_open=0 btn_stop=0 btn_close=0 sw_auto=1 sw_manual=0 limit_open=1 limit_close=0 slow_open=0 slow_close=0 rain=0 fw_version=0.1.0
```

| Field                        | Meaning                                                                                       |
| ------------------------------ | --------------------------------------------------------------------------------------------- |
| `mode`                        | `AUTO` / `MANUAL` / `OFF`                                                                     |
| `motion`                      | `STOPPED` / `OPENING` / `CLOSING` / `HOMING` / `FAULT` / `DISABLED`                           |
| `position`                    | `UNKNOWN` / `OPEN` / `CLOSED` — no intermediate value (Spec Abschnitt 5)                      |
| `direction`                   | `NONE` / `OPENING` / `CLOSING` — set solely by motor commands, never sensors                  |
| `zone`                        | `UNKNOWN` / `OPEN_SLOW` / `MAIN` / `CLOSE_SLOW`                                                |
| `percent`                     | 0–100, or `-1` if not homed — derived from the Hall counter; comfort/UI, and also the (best-effort) input `PERCENT` moves toward |
| `target_percent`              | 0–100 target of an in-progress `PERCENT` move, or `-1` if none is active                       |
| `hall`                        | raw Hall counter (motor position counts) — comfort only, see Hall Counter section above        |
| `speed`/`target_speed`        | actual / commanded motor speed, rpm                                                           |
| `current`                     | motor current, mA                                                                             |
| `motor_enabled`               | `0` while `Motion=DISABLED`                                                                   |
| `homed`                       | `1` once a valid homing run has completed                                                     |
| `calib_stale`                 | `1` after a `RESTART`/`RESETFAULT`, until the roof next fully closes and re-zeroes            |
| `modbus_connected`/`comm_fail`| Modbus link health; `comm_fail` hitting `MAX_COMM_FAILURES` -> `Fault`                        |
| `temp_mcu`/`temp_mosfet`/`temp_brake` | BLSD20 temperatures, °C                                                     |
| `led`                         | status LED on/off                                                                             |
| `fan`                         | case fan on/off                                                                               |
| `hardstop`                    | `1` if the BLSD20's `HARD_STOP` input is currently asserted (hardware E-stop loop tripped)    |
| `btn_open`/`btn_stop`/`btn_close` | live (debounced) button states                                                  |
| `sw_auto`/`sw_manual`         | live 3-position rotary switch legs (both `0` = center/`Off`)                                  |
| `limit_open`/`limit_close`    | live operational limit switch states                                                   |
| `slow_open`/`slow_close`      | live slowdown switch states                                                            |
| `rain`                        | `1` if the rain sensor reads wet; polarity per `RoofConfig::RAIN_SENSOR_INVERTED`              |
| `fw_version`                  | `FirmwareInfo::VERSION` — see Firmware metadata below                                         |
| `fault`/`errflags`            | only present when `motion=FAULT`: human-readable reason and the raw BLSD20 error bitmask (hex) |

## Firmware metadata and updates

Everything identifying a build and the physical controller lives in one place, [src/FirmwareInfo.h](src/FirmwareInfo.h). Nothing here is duplicated in `Config.h` or anywhere else. `INFO` returns it as a single JSON line (wrapped here for readability — on the wire it is one line):

```json
{"device":"SkyHub Roof Controller","model":"roof-controller","product":"skyhub-roof-controller",
 "board":"TEENSY41","serial":"12223410","hardware":"RevA","hardware_revision":1,
 "firmware":"0.1.0","protocol":1,"build":"2026-07-30T21:28:44Z","git":"7ff1716",
 "reset_reason":"power_on","reset_raw":1,
 "features":{"homing":true,"rain_sensor":true,"manual_mode":true,"percent_moves":true,
             "temperature":true,"watchdog_relay":true,"slow_switches":true,"requires_homing":false}}
```

### Field categories

Fields fall into four kinds, and SkyHub should treat them very differently:

| Kind                         | Fields                                                                  | Rule                                                                 |
| ------------------------------ | ------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| **Human-readable**           | `device`, `hardware`                                                    | display only — never branch on these, they are free to be reworded    |
| **Machine identifiers**       | `model`, `product`, `board`, `serial`                                   | stable, safe to compare exactly; changing one is a breaking change    |
| **Compatibility / version**   | `hardware_revision`, `firmware`, `protocol`                             | numeric or ordered — compare before flashing and before parsing       |
| **Diagnostics**               | `reset_reason`, `reset_raw`, `build`, `git`, `features`                 | observability and capability negotiation, not identity                |

### Fields

| Field               | Type    | Stability                                                          | Intended use by SkyHub Core                                                                                       |
| --------------------- | --------- | -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `device`            | string  | **unstable** — display name, may be reworded any time              | show in UIs and logs. Never match on it                                                                            |
| `model`             | string  | **stable** — identifies the hardware model across all revisions    | pick the right firmware image family                                                                              |
| `product`           | string  | **stable** — identifies this firmware product                      | primary key for "which kind of device is this"                                                                    |
| `board`             | string  | **stable** — equals the PlatformIO board id in `platformio.ini`     | hard gate before flashing: refuse an image built for another board                                                |
| `serial`            | string  | **stable per device** — unique per chip, survives reflashing        | identify the individual controller; equals the USB serial number, so it matches the `/dev/serial/by-id` path       |
| `hardware`          | string  | **unstable** — human label for the same thing as `hardware_revision` | display only                                                                                                      |
| `hardware_revision` | integer | **stable, monotonic** — RevA=1, RevB=2, …                          | compatibility check before flashing, without string parsing                                                       |
| `firmware`          | string  | changes every release; same value as `fw_version` in `STATUS`       | show the running version, decide whether an update is needed                                                      |
| `protocol`          | integer | **stable, monotonic** — bumped only on breaking protocol changes    | check before parsing anything else; refuse to talk to an unknown major                                            |
| `build`             | string  | changes every build — ISO 8601 UTC                                  | provenance; together with `git` the reliable way to confirm an update actually took effect                        |
| `git`               | string  | changes every commit — short hash, `-dirty` if the tree was modified | provenance; reject `-dirty` images in production                                                                  |
| `reset_reason`      | string  | **stable** — closed set, see below                                  | diagnostics after a reconnect                                                                                     |
| `reset_raw`         | integer | raw `SRC_SRSR` value                                                | diagnostics only, never branch on it — the decoded string is the contract. Useful when `reset_reason` is `unknown`, and it preserves several simultaneously-set bits that the single string cannot |
| `features`          | object  | **additive only** — keys are never removed or repurposed            | capability negotiation; treat a missing key as `false`                                                            |

`protocol` is bumped only for breaking changes. Adding a `STATUS` field, a command, or a `features` key is additive and leaves it alone — so **SkyHub's parser must tolerate unknown fields and unknown feature keys.**

### `features`

Each key answers "can I rely on this?" so SkyHub never has to infer capability from a version number. Keys are only ever added, never removed.

| Key               | Meaning                                                                                    |
| ------------------- | -------------------------------------------------------------------------------------------- |
| `homing`          | `HOME` command and the homing sequence exist                                                |
| `rain_sensor`     | `rain=` is present and meaningful in `STATUS`                                                |
| `manual_mode`     | hold-to-run Manual mode via the rotary switch exists                                        |
| `percent_moves`   | `PERCENT <0-100>` is accepted                                                                |
| `temperature`     | `temp_mcu`/`temp_mosfet`/`temp_brake` are present in `STATUS`                                 |
| `watchdog_relay`  | the Modbus watchdog relay driving the HARD_STOP chain is fitted                              |
| `slow_switches`   | slowdown switches are fitted (`RoofConfig::AUTO_HAS_SLOW_SWITCHES`); if `false` the roof always runs at `SPEED_AUTO_SLOW` |
| `requires_homing` | `OPEN`/`CLOSE` are refused until a successful `HOME` (`RoofConfig::AUTO_REQUIRES_HOMING`)     |

The last two mirror build-time policy and genuinely differ between builds. The others are compiled in unconditionally today and exist so a future controller variant can report `false` without a protocol change.

### `reset_reason`

Decoded from the i.MX RT1062 `SRC_SRSR` register. One of:

| Value         | Cause                                                                              |
| --------------- | ------------------------------------------------------------------------------------ |
| `power_on`    | plain reset — see the caveat below, this covers more than just power-on             |
| `software`    | software reset via `SCB_AIRCR` (this firmware never issues one) or a CPU lockup     |
| `fault`       | the core's automatic reboot after a fault or a bad interrupt vector                 |
| `watchdog`    | watchdog timer 1, 2 or 3 expired                                                    |
| `external`    | external reset input                                                                |
| `security`    | security monitor (CSU)                                                              |
| `jtag`        | JTAG boundary scan or JTAG debug reset                                              |
| `temperature` | on-chip temperature sensor tripped                                                  |
| `unknown`     | no cause bit set, or built for a non-Teensy target                                  |

> **Caveat, Teensy 4.1 specific.** The MKL02 bootloader chip manages 3.3 V supervision and the RESET line, so **power-on, brownout, an external reset pulse and the reboot after a `teensy_loader_cli` flash all arrive as the same plain reset** and are reported as `power_on`. Brownout is not separately detectable on this hardware, and `reset_reason` **cannot** confirm that a firmware update happened. To verify an update, compare `firmware`/`build`/`git` across the reconnect. Where `reset_reason` genuinely earns its keep is telling a clean restart apart from a `watchdog` or `fault` reboot.

### Build provenance

`build` and `git` are injected at compile time by [scripts/build_metadata.py](scripts/build_metadata.py) (wired up via `extra_scripts` in `platformio.ini`); both fall back to `"unknown"` if the firmware is built without it. Because the timestamp changes on every build, every build is a full rebuild — a few seconds for this project.

### Updates

Updates themselves are not part of this firmware: `FWUPDATE` only acknowledges and jumps into HalfKay, and SkyHub Core does the flashing with `teensy_loader_cli`. See the `FWUPDATE` row in the command table for the acknowledgement handshake. Note that HalfKay accepts any image — `board`, `model` and `hardware_revision` must be checked **before** sending `FWUPDATE`, since the firmware cannot refuse a mismatched image itself.
