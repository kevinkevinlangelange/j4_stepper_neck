# j4_stepper

Dedicated stepper driver board for the Johnny 4 robot. Runs on an ESP32-DEV (WROOM) module. It listens for position packets from the j4_receiver (TTGO) over a wired UART link and drives all five of Johnny 4's stepper motors. Pulse generation runs on the ESP32 RMT peripheral via FastAccelStepper, so the CPU never blocks and the board stays lightly loaded.

## What this board does

- Receives ASCII position packets from j4_receiver over UART1 (one-way, wired)
- Drives 3 neck steppers via BTT TMC5160T drivers (neck left, neck right, neck pivot)
- Drives 2 eye-pop steppers via TMC2209 drivers, ganged so they always move together
- Filters out tiny commanded changes (per-axis threshold) so the motors do not jitter at rest

The stepper library and motion settings are copied from j4_receiver and j4_motor_test_dev so the neck moves exactly as it did before the steppers were relocated to this board.

## Why a separate board

The TTGO receiver already runs the TFT, ESP-NOW, and two UART links. Moving the steppers onto their own ESP32 keeps each board lightly loaded and frees the TTGO's GPIO. Because FastAccelStepper offloads every step pulse to the RMT hardware, the ESP32-DEV has plenty of headroom for five motors.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DEV (WROOM) |
| Neck drivers | 3x BIGTREETECH TMC5160T V1.0 (software SPI) |
| Eye-pop drivers | 2x TMC2209 (shared single-wire UART) |
| Neck motors | STEPPERONLINE 17HE24-2104S (2.1A) |

## Incoming packet format

One newline-terminated line of four comma-separated integers:

```
nL,nR,nP,eye\n
```

| Field | Meaning | Range |
|-------|---------|-------|
| nL | neck left joystick value | 0-3200 |
| nR | neck right joystick value | 0-3200 |
| nP | neck pivot joystick value | 0-3200 |
| eye | eye-pop value | 0-255 |

The parser uses a fixed char buffer (no Arduino `String`, so no heap fragmentation). Any line that does not parse to exactly four integers is ignored, so a half-received line at power-up cannot move a motor.

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
| 13 / 4 | eye-pop STEP / DIR (both TMC2209, shared) |
| 21 / 22 | TMC2209 UART TX / RX (PDN_UART bus) |
| 35 | UART1 RX, from TTGO UART1 TX (input-only pin, fine for RX) |

Strapping pins (0, 2, 12, 15) are avoided for outputs and the flash pins (6-11) are left alone. On WROVER modules GPIO 16/17 are tied to PSRAM, so this build targets a plain WROOM ESP32-DEV where they are free.

## Pin diagram

```
                      ESP32-DEV (WROOM)  -  j4_stepper
                  +--------------------------------------+
   TTGO UART1 TX ->| 35 (UART1 RX, 250000)               |
                  |                                      |
   TMC5160T SPI:  | 18 SCK   19 MISO   23 MOSI  (shared) |
     CS L/R/P  <--| 5        17        16               |
   neck L STEP/DIR| 26 / 25                              |
   neck R STEP/DIR| 33 / 32          ---> 3x TMC5160T (neck L/R/pivot)
   neck P STEP/DIR| 27 / 14                              |
                  |                                      |
   eye STEP/DIR <-| 13 / 4           ---> 2x TMC2209 (eye-pop, ganged)
   TMC2209 UART  <| 21 TX   22 RX    (PDN_UART bus)      |
                  |        3V3   GND   VIN   VM(12-24V)  |
                  +--------------------------------------+
```

## Eye-pop wiring (two drivers, one step/dir pair)

The two eye-pop motors always move at the same speed and direction, so both TMC2209 STEP inputs connect to GPIO 13 and both DIR inputs connect to GPIO 4. The ESP32 pin drives both high-impedance driver inputs with no trouble. This is the same idea as a dual-Z setup on a 3D printer, and it means the pair uses a single FastAccelStepper channel.

The two drivers share one PDN_UART bus (GPIO 21/22, joined to each driver's PDN_UART pin through a 1k resistor). They are told apart by their MS1/MS2 address straps: driver 0 = 0b00, driver 1 = 0b01.

If you would rather skip the UART, the TMC2209s also run in standalone mode (current set by the VREF pot, microstep by MS1/MS2). In that case GPIO 21/22 are unused and the UART config in `setup()` can be removed.

## Power and enable

VIO on every driver goes to 3.3V, VM to the 12-24V motor supply, and all grounds are common with the ESP32. Every driver's EN pin is tied to GND, so the drivers are always enabled.

## Motion settings

Neck values match the originals: 16 microsteps, 3200 steps/rev, `setSpeedInHz(3200)`, `setAcceleration(16000)`, `rms_current(2100)`. The neck-left axis is mapped inverted (`(3200 - nL) / 2`) exactly as j4_receiver had it.

Eye-pop settings (`EYE_CURRENT`, `EYE_SPEED_HZ`, `EYE_ACCEL`, `EYE_TRAVEL`) are placeholders meant to be tuned to the eye-pop mechanism. `EYE_TRAVEL` sets how many microsteps the 0-255 eye value maps across.

## Link to j4_receiver

j4_receiver sends the packet on its UART1 TX at 250000 baud. Both boards are 3.3V, so no level shifter is needed. Only one wire plus ground is required (TTGO TX to this board's GPIO 35). `LINK_BAUD` here must match `STEPPER_LINK_BAUD` in j4_receiver.

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

- **[j4_receiver](https://github.com/kevinkevinlangelange/j4_receiver)** -- the TTGO receiver board that streams `nL,nR,nP,eye` packets to this board over UART1
