# j4_stepper_neck

Neck stepper driver board for the Johnny 4 robot, and the middle node of the stepper daisy chain. Runs on an ESP32-DEV (WROOM) module. It drives the three neck steppers via BIGTREETECH TMC5160T drivers, relays the eye-pop value down to [j4_stepper_eyes](https://github.com/kevinkevinlangelange/j4_stepper_eyes), and reports driver health back up to the receiver. Step pulses run on the ESP32 RMT peripheral via FastAccelStepper, so the CPU never blocks.

## Daisy chain

```
  j4_controller --ESP-NOW--> j4_receiver (TTGO)
                                  |  UART1 (2-way, 250000)
                                  v
                          j4_stepper_neck   (3x TMC5160T)   <-- this board
                                  |  UART2 (2-way, 250000)
                                  v
                          j4_stepper_eyes   (2x TMC2209)
```

The TTGO only has one free hardware UART, so the boards are chained rather than each wired to the TTGO. This board is the relay in the middle: it takes the full packet from the receiver, keeps the neck fields, and passes the eye field downstream; status flows back the other way.

## What this board does

- Receives `nL,nR,nP,eye` from j4_receiver over UART1 (2-way)
- Drives 3 neck steppers via BTT TMC5160T drivers (neck left, neck right, neck pivot)
- Forwards the `eye` value to j4_stepper_eyes over UART2
- Polls each TMC5160T's DRV_STATUS register for faults (overtemp prewarning/shutdown, short-to-ground, open-load)
- Merges its own driver status with the eyes board's status and a link-alive check, and reports a single status string back upstream
- Filters out tiny commanded changes (per-axis threshold) so the motors do not jitter at rest

Motion settings are copied from j4_motor_test_dev so the neck moves exactly as before.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DEV (WROOM-32) |
| Neck drivers | 3x BIGTREETECH TMC5160T V1.0 (software SPI) |
| Neck motors | STEPPERONLINE 17HE24-2104S (2.1A) |

## Link protocol

**Upstream (UART1, to/from j4_receiver):**

| Direction | Message | Meaning |
|-----------|---------|---------|
| in | `nL,nR,nP,eye\n` | neck L/R/pivot + eye-pop, all 0-3200 |
| in | `CAL\n` | run a homing/calibration pass (also forwarded to the eyes board) |
| out | `ST:<text>\n` | merged status, e.g. `ST:OK`, `ST:NL OT`, `ST:NL LIMIT`, `ST:EYES OFFLINE` |

**Downstream (UART2, to/from j4_stepper_eyes):**

| Direction | Message | Meaning |
|-----------|---------|---------|
| out | `<eye>\n` | the forwarded eye-pop value (0-3200) |
| out | `CAL\n` | tell the eyes board to calibrate too |
| in | `S:<text>\n` | the eyes board's status (`S:OK`, `S:EL OT`, ...) |

The parsers use fixed char buffers (no Arduino `String`, so no heap fragmentation). A line that does not parse is ignored.

## Status reporting

Every 200 ms the board sends one status string upstream, chosen by priority:

1. `EYES OFFLINE` -- no status from the eyes board within 800 ms
2. a neck driver fault -- `NL`/`NR`/`NP` + `OT` (overtemp shutdown), `SHORT`, `OTPW` (overtemp prewarning), `OPEN` (open load), or `LIMIT` (a limit switch is engaged or its wire is broken)
3. an eye driver fault relayed up from the eyes board (`EL`/`ER` + token)
4. `OK`

The DRV_STATUS registers are polled every 100 ms over the existing SPI bus. Reads do not disturb motion, since step pulses come from the RMT peripheral, not the CPU.

## Limit switches and calibration

Each axis has **two normally-closed (NC) switches wired in series** to one input (GPIO 34/35/36), with an **external 10k pull-up to 3.3V**. Both closed (in range) reads LOW; either switch opening, or a broken wire, reads HIGH and is treated as a limit hit (fail-safe). The travel direction tells which end was reached.

- **In normal operation**, if a limit trips while an axis is moving, that axis is stopped immediately.
- **On a `CAL` command**, the board homes each axis: it drives slowly toward the home end until the limit switch trips **or** StallGuard detects a hard stop, backs off, and zeros the position. StallGuard (read over SPI, no extra pins) is what catches an axis that is **already sitting against a stop** when calibration starts -- it stalls the instant it tries to push further, instead of grinding.

The StallGuard threshold (`SG_SGT`, `SG_STALL_LEVEL`), homing speed/back-off, and the home direction are `#define`s at the top of `main.cpp` and **need tuning on the real mechanism**. The far-end range is then enforced in software once homed.

## Pin assignments

| GPIO | Function |
|------|----------|
| 18 | SPI SCK (all 3 TMC5160T, shared) |
| 19 | SPI MISO (all 3 TMC5160T, shared) |
| 23 | SPI MOSI (all 3 TMC5160T, shared) |
| 5 | CS neck-left TMC5160T |
| 17 | CS neck-right TMC5160T |
| 16 | CS neck-pivot TMC5160T |
| 26 / 25 | neck-left STEP / DIR |
| 33 / 32 | neck-right STEP / DIR |
| 27 / 14 | neck-pivot STEP / DIR |
| 4 / 13 | Upstream UART1 RX / TX (to TTGO) |
| 21 / 22 | Downstream UART2 TX / RX (to eyes board) |
| 34 | neck-left limit switches (input, external pull-up) |
| 36 | neck-right limit switches (input, external pull-up) |
| 39 | neck-pivot limit switches (input, external pull-up) |

The links and step/dir use plain bidirectional GPIOs (strapping pins 0/2/12/15 and flash pins 6-11 are avoided). The three limit inputs use the input-only pins 34/36/39, each with an external 10k pull-up to 3.3V.

GPIO 35 is deliberately left unused, because some of these LiPo dev boards route battery sense to it. Limit switches are not required for normal operation, so you can wire them whenever you add them.

## Pin diagram

```
                   ESP32-DEV (WROOM)  -  j4_stepper_neck
                  +--------------------------------------+
   TTGO 21 (TX) ->| 4  (UART1 RX)        21 (UART2 TX) |-> eyes 4 (RX)
   TTGO 22 (RX) <-| 13 (UART1 TX)        22 (UART2 RX) |<- eyes 13 (TX)
                  |                                      |
   TMC5160T SPI:  | 18 SCK   19 MISO   23 MOSI  (shared) |
     CS L/R/P  <--| 5        17        16               |
   neck L STEP/DIR| 26 / 25                              |
   neck R STEP/DIR| 33 / 32          ---> 3x TMC5160T (neck L/R/pivot)
   neck P STEP/DIR| 27 / 14                              |
   limit inputs <-| 34 (NL)  36 (NR)  39 (NP)            |
                  |        3V3   GND   VIN   VM(12-24V)  |
                  +--------------------------------------+

   Each limit input = two NC switches in series + a 10k pull-up to 3.3V.
```

## Power and enable

VIO on every driver goes to 3.3V, VM to the 12-24V motor supply, and all grounds are common with the ESP32. Every driver's EN pin is tied to GND, so the drivers are always enabled.

## Motion settings

Neck values match the originals: 16 microsteps, `setSpeedInHz(3200)`, `setAcceleration(16000)`, `rms_current(2100)`. The neck-left axis is mapped inverted (`(3200 - nL) / 2`).

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
- **[j4_stepper_eyes](https://github.com/kevinkevinlangelange/j4_stepper_eyes)** -- the eye-pop stepper board downstream of this one
