# j4_stepper_neck

Neck stepper driver board for the Johnny 4 robot, and the middle node of the stepper daisy chain. Runs on an ESP32-DEV (WROOM) module. It drives the three neck steppers via STEPPERONLINE DM556TE digital step/dir drivers, relays the eye-pop value down to [j4_stepper_eyes](https://github.com/kevinkevinlangelange/j4_stepper_eyes), and reports neck/eye health back up to the receiver. Step pulses run on the ESP32 RMT peripheral via FastAccelStepper, so the CPU never blocks.

## Daisy chain

```
  j4_controller --ESP-NOW--> j4_receiver (TTGO)
                                  |  UART1 (2-way, 250000)
                                  v
                          j4_stepper_neck   (3x DM556TE)   <-- this board
                                  |  UART2 (2-way, 250000)
                                  v
                          j4_stepper_eyes   (2x TMC2209)
```

The TTGO only has one free hardware UART, so the boards are chained rather than each wired to the TTGO. This board is the relay in the middle: it takes the full packet from the receiver, keeps the neck fields, and passes the eye field downstream; status flows back the other way.

## What this board does

- Receives `nL,nR,nP,eye` from j4_receiver over UART1 (2-way)
- Drives 3 neck steppers via STEPPERONLINE DM556TE drivers (neck left, neck right, neck pivot)
- Forwards the `eye` value to j4_stepper_eyes over UART2
- Merges its own limit-switch status with the eyes board's status and a link-alive check, and reports a single status string back upstream
- Filters out tiny commanded changes (per-axis threshold) so the motors do not jitter at rest

The DM556TE is a standalone driver: motor current and microstep resolution are set by DIP switches on the driver itself, and there is no telemetry bus back to the ESP32. Neck faults are therefore reported from the limit switches only (the previous TMC5160T build could also report overtemp/short/open-load over SPI; the DM556TE cannot).

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DEV (WROOM-32) |
| Neck drivers | 3x STEPPERONLINE DM556TE (digital step/dir, opto-isolated inputs) |
| Neck motors | STEPPERONLINE 17HE24-2104S (2.1A) |
| Motor supply | 24-50VDC to each DM556TE +Vdc |

## Link protocol

**Upstream (UART1, to/from j4_receiver):**

| Direction | Message | Meaning |
|-----------|---------|---------|
| in | `nL,nR,nP,eye\n` | neck L/R/pivot + eye-pop, all 0-3200 |
| in | `CAL\n` | run a homing/calibration pass (also forwarded to the eyes board) |
| out | `ST:<text>\n` | merged status, e.g. `ST:OK`, `ST:NL LIMIT`, `ST:EYES OFFLINE` |
| out | `EY:<0/1>\n` | eyes-board link state (1 = alive) |

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
2. a neck limit fault -- `NL`/`NR`/`NP` + `LIMIT` (a limit switch is engaged or its wire is broken)
3. an eye driver fault relayed up from the eyes board (`EL`/`ER` + token)
4. `OK`

## Limit switches and calibration

Each axis has **two normally-closed (NC) switches wired in series** to one input (GPIO 34/36/39), with an **external 10k pull-up to 3.3V**. Both closed (in range) reads LOW; either switch opening, or a broken wire, reads HIGH and is treated as a limit hit (fail-safe). The travel direction tells which end was reached.

- **In normal operation**, if a limit trips while an axis is moving, that axis is stopped immediately.
- **On a `CAL` command**, the board homes each axis: it drives slowly toward the home end until the limit switch trips or the per-axis timeout expires, backs off until the limit clears, and zeros the position.

The DM556TE has no stall detection, so an axis that starts jammed hard against the home stop cannot be sensed; it drives until the timeout and then continues. The homing speed, back-off, and direction are `#define`s at the top of `main.cpp` and **need tuning on the real mechanism**. The far-end range is then enforced in software once homed.

## DM556TE driver wiring

Each of the three drivers is wired the same way. The DM556TE has four terminal groups: signal inputs (PUL/DIR/ENA), VDC power, and the motor phases.

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

## Pin assignments

| GPIO | Function |
|------|----------|
| 26 / 25 | neck-left STEP / DIR -> DM556TE PUL+ / DIR+ |
| 33 / 32 | neck-right STEP / DIR -> DM556TE PUL+ / DIR+ |
| 27 / 14 | neck-pivot STEP / DIR -> DM556TE PUL+ / DIR+ |
| 4 / 13 | Upstream UART1 RX / TX -- to j4_receiver: RX from TTGO GPIO 21, TX to TTGO GPIO 22 |
| 22 / 35 | Downstream UART2 TX / RX -- to j4_stepper_eyes: TX to eyes GPIO 4, RX from eyes GPIO 13 |
| 34 | neck-left limit switches (input, external pull-up) |
| 36 | neck-right limit switches (input, external pull-up) |
| 39 | neck-pivot limit switches (input, external pull-up) |

The board is a bare ESP32-D0WD (LOLIN32-Lite style). It does **not break out GPIO 21**, so the downstream UART TX is on **GPIO 22** (which also drives the onboard LED, giving a handy downstream-activity blink). The downstream RX is on the input-only **GPIO 35**; the board has **no battery-sense pin**, so 35 is safe to use. The step/dir lines use plain bidirectional GPIOs (strapping pins 0/2/12/15 and flash pins 6-11 avoided). The three limit inputs use the input-only pins 34/36/39, each with an external 10k pull-up to 3.3V.

Switching from the TMC5160T to the DM556TE frees the six SPI pins (SCK 18, MISO 19, MOSI 23, and CS 5/17/16); they are unused now.

Limit switches are not required for normal operation, so you can wire them whenever you add them.

## Pin diagram

This matches the real pin order on the board: hold it component-side up with the
USB-C connector at the bottom. The two header rails read top-to-bottom as shown.

```
          LEFT rail                                RIGHT rail
                          +--------------+
       limit NR -- 36 |   |              |   | 3V3   3.3V
       limit NP -- 39 |   |              |   | 22    DN UART2 TX -> eyes 4   (LED)
          reset -- EN |   |  ESP32-D0WD  |   | 19    (free)
       limit NL -- 34 |   |  (LOLIN32-   |   | 23    (free)
 DN RX <-eyes13 -- 35 |   |   Lite style)|   | 18    (free)
     DIR neck-R -> 32 |   |              |   | 5     (free)
    STEP neck-R -> 33 |   |              |   | 17    (free)
     DIR neck-L -> 25 |   |              |   | 16    (free)
    STEP neck-L -> 26 |   |              |   | 4     UART1 RX <- TTGO 21
    STEP neck-P -> 27 |   |              |   | 0     (free / BOOT)
     DIR neck-P -> 14 |   |              |   | 2     (free)
         (free) -- 12 |   |              |   | 15    (free)
         ground -- GND|   |              |   | 13    UART1 TX -> TTGO 22
                          +---[ USB-C ]--+

   GPIO 21 is NOT present on this board, which is why the downstream TX is on 22.
   STEP/DIR pins feed the DM556TE PUL+/DIR+ inputs (PUL-/DIR- to GND).
   Each limit input = two NC switches in series + a 10k pull-up to 3.3V.
   The old SPI pins (18/19/23, 5/17/16) are free now that the TMC5160Ts are gone.
```

## Motion settings

Neck values match the originals: `setSpeedInHz(3200)`, `setAcceleration(16000)`, and a per-axis change threshold of 20. With each DM556TE set to 3200 pulse/rev, one commanded unit equals one pulse, so these are the same numbers as the old 16-microstep setup. The neck-left axis is mapped inverted (`(3200 - nL) / 2`).

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
