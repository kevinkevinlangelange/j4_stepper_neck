# j4_stepper_neck

Stepper driver board for the Johnny 4 robot -- the sole stepper node, driving all five steppers. Runs on an ESP32-DEV (WROOM) module. The three neck axes run on STEPPERONLINE DM556TE digital step/dir drivers, and the two eye-pop motors run on TMC2209 drivers over a shared UART. Step pulses run on the ESP32 RMT peripheral via FastAccelStepper, so the CPU never blocks.

The former [j4_stepper_eyes](https://github.com/kevinkevinlangelange/j4_stepper_eyes) board is retired: swapping the neck drivers to the DM556TE freed the six SPI pins, which is exactly enough headroom to absorb the two eye channels onto this board and delete the downstream UART link entirely.

## Where it sits

```
  j4_controller --ESP-NOW--> j4_receiver (TTGO)
                                  |  UART1 (2-way, 250000)
                                  v
                          j4_stepper_neck   (3x DM556TE + 2x TMC2209)   <-- this board
```

## What this board does

- Receives `nL,nR,nP,eye` from j4_receiver over UART1 (2-way)
- Drives 3 neck steppers via STEPPERONLINE DM556TE drivers (neck left, neck right, neck pivot)
- Drives 2 eye-pop steppers via TMC2209 drivers, each on its own step/dir channel but commanded to identical targets, so the eyes pop together yet can be trimmed/homed independently
- Polls the TMC2209s over their shared UART for faults (overtemp, short, open-load) and link health
- Merges everything into a single status string reported back upstream
- Filters out tiny commanded changes (per-axis threshold) so the motors do not jitter at rest

The DM556TE is a standalone driver: motor current and microstep resolution are set by DIP switches on the driver itself, and there is no telemetry bus back to the ESP32. Neck faults are therefore reported from the limit switches only. The eye TMC2209s do have a telemetry bus (their shared UART), so they keep full driver-fault reporting and StallGuard4 stall detection during homing.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DEV (WROOM-32) |
| Neck drivers | 3x STEPPERONLINE DM556TE (digital step/dir, opto-isolated inputs) |
| Neck motors | STEPPERONLINE 17HE24-2104S (2.1A) |
| Neck motor supply | 24-50VDC to each DM556TE +Vdc |
| Eye drivers | 2x TMC2209 (shared UART, addresses 0b00 / 0b01 via MS1/MS2) |
| Eye motor supply | 12-24VDC to each TMC2209 VM |

## Link protocol

**Upstream (UART1, to/from j4_receiver):**

| Direction | Message | Meaning |
|-----------|---------|---------|
| in | `nL,nR,nP,eye\n` | neck L/R/pivot + eye-pop, all 0-3200 |
| in | `CAL\n` | run a homing/calibration pass on all five axes |
| out | `ST:<text>\n` | merged status, e.g. `ST:OK`, `ST:NL LIMIT`, `ST:EL OT`, `ST:EYES OFFLINE` |
| out | `EY:<0/1>\n` | eye driver state (1 = both TMC2209s respond over UART) |

The parser uses fixed char buffers (no Arduino `String`, so no heap fragmentation). A line that does not parse is ignored.

`EY:` used to mean "is the downstream eyes board alive" when the eyes lived on their own ESP32; it now reports whether the two local TMC2209s answer on their UART. j4_receiver and j4_controller read it exactly the same way, so they need no changes.

## Status reporting

Every 200 ms the board sends one status string upstream, chosen by priority:

1. `EYES OFFLINE` -- one or both TMC2209s stopped responding over UART
2. a neck limit fault -- `NL`/`NR`/`NP` + `LIMIT` (a limit switch is engaged or its wire is broken)
3. an eye fault -- `EL`/`ER` + `OT` (overtemp shutdown), `SHORT`, `OTPW` (overtemp prewarning), `OPEN` (open load), or `LIMIT`
4. `OK`

The TMC2209s and all five limit inputs are polled every 100 ms.

## Limit switches and calibration

Each axis has **two normally-closed (NC) switches wired in series** to one input, with an **external 10k pull-up to 3.3V**. Both closed (in range) reads LOW; either switch opening, or a broken wire, reads HIGH and is treated as a limit hit (fail-safe). The travel direction tells which end was reached.

- **In normal operation**, if a limit trips while an axis is moving, that axis is stopped immediately.
- **On a `CAL` command**, the board homes all five axes in sequence: each drives slowly toward the home end until its limit switch trips (eyes: or StallGuard4 detects a hard stop) or the per-axis timeout expires, backs off until the limit clears, and zeros the position.

The DM556TE has no stall detection, so a neck axis that starts jammed hard against the home stop cannot be sensed; it drives until the timeout and then continues. The eye axes keep StallGuard4 (read over the TMC2209 UART, no extra pins), which catches an eye already sitting against a stop the instant it tries to push further. The homing speed, back-off, direction, and the StallGuard thresholds (`SG_SGTHRS`, `SG_STALL_LEVEL`) are `#define`s at the top of `main.cpp` and **need tuning on the real mechanism**. The far-end range is then enforced in software once homed.

## DM556TE driver wiring (neck, three identical)

The DM556TE has four terminal groups: signal inputs (PUL/DIR/ENA), VDC power, and the motor phases.

### Signal inputs (opto-isolated)

The DM556TE inputs are optically isolated and rated for 5-24V logic at 7-20mA. Wire them **common-cathode / active-high** so the ESP32's normal 3.3V logic drives them directly and the pulse/direction polarity stays normal (no firmware inversion):

| DM556TE terminal | Connect to |
|------------------|------------|
| PUL+ | ESP32 STEP pin (26 / 33 / 27) |
| PUL- | ESP32 GND |
| DIR+ | ESP32 DIR pin (25 / 32 / 14) |
| DIR- | ESP32 GND |
| ENA+ / ENA- | leave unconnected (driver is enabled by default) |

> **3.3V note:** 3.3V is at the low edge of the opto input's 5V spec. It works on the bench, but if an axis drops pulses, add a 3.3V-to-5V level shifter on the PUL/DIR lines, or drive the inputs common-anode from a 5V rail (which inverts polarity and is not used here).

### Power and motor

| DM556TE terminal | Connect to |
|------------------|------------|
| +Vdc / GND | 24-50VDC motor supply (its GND common with the ESP32 GND) |
| A+ / A- | stepper phase A |
| B+ / B- | stepper phase B |

If an axis runs the wrong direction, swap that motor's A+/A- pair (or invert its DIR mapping in firmware). All grounds -- ESP32, driver signal, and motor supply -- must be common.

### DIP switch settings

Current and microstepping are set on the driver, not in firmware.

- **Current (SW1-SW3) + SW4:** set to your motor from the driver's printed table. For the 2.1A neck motors use the **2.1A-peak / 1.5A-RMS** row: **SW1=on, SW2=off, SW3=off**, with **SW4=on** (full current). Move up one row if you need more holding torque and the motor stays cool.
- **Microstepping (SW5-SW8):** set **3200 pulse/rev** -> **SW5=on, SW6=on, SW7=off, SW8=on**. This makes one commanded unit in firmware equal one step pulse, matching the old 16-microstep scale so all the motion numbers carry over unchanged.

## TMC2209 driver wiring (eyes, two on a shared UART)

| TMC2209 pin | Connect to |
|-------------|------------|
| STEP / DIR | ESP32 18 / 19 (eye left), 23 / 5 (eye right) |
| PDN_UART | ESP32 17 (TX, via 1k per driver) and 16 (RX), shared bus |
| MS1 / MS2 | address straps: eye-left = 0b00, eye-right = 0b01 |
| VIO | 3.3V |
| VM | 12-24VDC motor supply |
| EN | GND (always enabled) |

Driver current (800 mA RMS) and microstepping (16x) for the eyes are set in firmware over the UART, unlike the DM556TEs.

## Pin assignments

| GPIO | Function |
|------|----------|
| 26 / 25 | neck-left STEP / DIR -> DM556TE PUL+ / DIR+ |
| 33 / 32 | neck-right STEP / DIR -> DM556TE PUL+ / DIR+ |
| 27 / 14 | neck-pivot STEP / DIR -> DM556TE PUL+ / DIR+ |
| 18 / 19 | eye-left STEP / DIR -> TMC2209 |
| 23 / 5 | eye-right STEP / DIR -> TMC2209 |
| 17 / 16 | TMC2209 UART TX / RX (shared bus to both eye drivers) |
| 4 / 13 | Upstream UART1 RX / TX -- to j4_receiver: RX from TTGO GPIO 21, TX to TTGO GPIO 22 |
| 34 | neck-left limit switches (input, external pull-up) |
| 36 | neck-right limit switches (input, external pull-up) |
| 39 | neck-pivot limit switches (input, external pull-up) |
| 35 | eye-left limit switches (input, external pull-up) |
| 22 | eye-right limit switches (input, external pull-up; onboard LED = in-range indicator) |

The board is a bare ESP32-D0WD (LOLIN32-Lite style); it does not break out GPIO 21, and it has no battery-sense pin, so the input-only 34/35/36/39 are all safe to use -- they carry four of the five limit inputs. The fifth (eye-right) rides on the ordinary GPIO 22, whose onboard LED doubles as a free in-range indicator. The step/dir lines use plain bidirectional GPIOs (strapping pins 0/2/12/15 and flash pins 6-11 avoided).

Limit switches are not required for normal operation, so you can wire them whenever you add them.

## Pin diagram

This matches the real pin order on the board: hold it component-side up with the
USB-C connector at the bottom. The two header rails read top-to-bottom as shown.

```
          LEFT rail                                RIGHT rail
                          +--------------+
       limit NR -- 36 |   |              |   | 3V3   3.3V (TMC2209 VIO)
       limit NP -- 39 |   |              |   | 22    limit ER            (LED)
          reset -- EN |   |  ESP32-D0WD  |   | 19    DIR  eye-L  -> TMC2209
       limit NL -- 34 |   |  (LOLIN32-   |   | 23    STEP eye-R  -> TMC2209
       limit EL -- 35 |   |   Lite style)|   | 18    STEP eye-L  -> TMC2209
     DIR neck-R -> 32 |   |              |   | 5     DIR  eye-R  -> TMC2209
    STEP neck-R -> 33 |   |              |   | 17    TMC2209 UART TX
     DIR neck-L -> 25 |   |              |   | 16    TMC2209 UART RX
    STEP neck-L -> 26 |   |              |   | 4     UART1 RX <- TTGO 21
    STEP neck-P -> 27 |   |              |   | 0     (free / BOOT)
     DIR neck-P -> 14 |   |              |   | 2     (free)
         (free) -- 12 |   |              |   | 15    (free)
         ground -- GND|   |              |   | 13    UART1 TX -> TTGO 22
                          +---[ USB-C ]--+

   Neck STEP/DIR pins feed the DM556TE PUL+/DIR+ inputs (PUL-/DIR- to GND).
   Each limit input = two NC switches in series + a 10k pull-up to 3.3V.
   GPIO 22's onboard LED doubles as the eye-right in-range indicator.
```

## Motion settings

Neck values match the originals: `setSpeedInHz(3200)`, `setAcceleration(16000)`, and a per-axis change threshold of 20. With each DM556TE set to 3200 pulse/rev, one commanded unit equals one pulse, so these are the same numbers as the old 16-microstep setup. The neck-left axis is mapped inverted (`(3200 - nL) / 2`).

The eyes use the same speed/accel/threshold so they ramp exactly like the neck; only the driver current differs (800 mA RMS for the small eye motors, set over the TMC2209 UART).

## Building and uploading

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

## Related projects

- **[j4_receiver](https://github.com/kevinkevinlangelange/j4_receiver)** -- the TTGO that streams `nL,nR,nP,eye` to this board and relays its status to the controller
- **[j4_stepper_eyes](https://github.com/kevinkevinlangelange/j4_stepper_eyes)** -- retired; its two eye channels were absorbed into this board
