//******************************************************************************
//         file name:  j4_stepper.ino
//      v0_1 created:  2026-06-14 -- 0123 CDT -KL
//      last updated:  2026-06-15 -- CDT
//      last updated:  2026-06-17 -- CDT
//      last updated:  2026-06-18 -- CDT
//      last updated:  2026-06-19 -- CDT
//      last updated:  2026-07-02 -- CDT
//            author:  Kevin Lange
//       description:  Stepper driver board for Johnny 4, on an ESP32-DEV (WROOM)
//                     module. Drives ALL FIVE steppers: the three neck axes via
//                     STEPPERONLINE DM556TE digital step/dir drivers and the two
//                     eye-pop motors via TMC2209 drivers. Sole stepper node:
//
//                       j4_receiver (TTGO) <--UART1--> [ j4_stepper ]
//
//                     It parses "nL,nR,nP,eye" from the receiver, drives the neck
//                     with nL/nR/nP and the eye-pop pair with eye. The DM556TE
//                     drivers are standalone (current/microsteps on DIP switches,
//                     no telemetry), so neck health comes from the limit switches
//                     only. The TMC2209s are polled for faults over their shared
//                     UART, and everything is merged into a single status string
//                     sent back upstream.
//
//                     Step pulses come from the ESP32 RMT peripheral via
//                     FastAccelStepper (no CPU blocking).
//       update log:  v0_1 -- Initial build (combined neck + eye board).
//                    v0_2 -- Eye-pop bound to neck motion settings.
//                    v0_3 -- Split into two boards. This board is neck-only now;
//                            the eye-pop drivers moved to j4_stepper_eyes. Added a
//                            2-way upstream link, a downstream link to the eyes
//                            board (relays the eye value), DRV_STATUS fault polling
//                            on the three neck drivers, and a merged status report.
//                    v0_4 -- Added limit switches (two NC per axis in series, one
//                            input each: GPIO 34/35/36), StallGuard2 config + stall
//                            detection, a CAL homing routine that stops on a limit
//                            OR a stall (so an axis already at a stop is caught), and
//                            a safety stop if a limit trips mid-move. Limit state is
//                            reported in the status string. SG thresholds and homing
//                            params still need hardware tuning.
//                    v0_5 -- Also report the eyes-board link state upstream as an
//                            "EY:<0/1>" line, so j4_receiver / j4_controller can show
//                            j4_stepper_eyes connection on the status screen.
//                    v0_6 -- Moved the limit inputs off GPIO 35 (NR 35->36, NP
//                            36->39). Limits are now 34 (NL) / 36 (NR) / 39 (NP).
//                    v0_7 -- Confirmed the board is a bare ESP32-D0WD (LOLIN32-Lite
//                            style): onboard LED on GPIO 22, no battery-sense pin.
//                            Moved the downstream UART RX off the LED pin (GPIO 22
//                            -> GPIO 35, input-only). GPIO 35 is safe (no battery).
//                    v0_8 -- This board does NOT break out GPIO21, so moved the
//                            downstream UART TX 21 -> 22 (GPIO22 is exposed and
//                            carries the onboard LED, which becomes a TX-activity
//                            blink). Downstream is now TX 22 / RX 35. Redrew the
//                            README pin diagram to the real physical pin layout and
//                            added UART destination pins to the assignment table.
//                    v0_9 -- Swapped the three neck drivers from BTT TMC5160T to
//                            STEPPERONLINE DM556TE digital step/dir drivers. Removed
//                            all SPI / TMCStepper / StallGuard / DRV_STATUS code: the
//                            DM556TE is a standalone driver with no telemetry bus, so
//                            current and microstepping are set by its DIP switches and
//                            the step/dir pins now feed the driver's opto-isolated
//                            PUL/DIR inputs. The six SPI pins (18/19/23 + CS 5/17/16)
//                            are freed. Status now reports limit-switch faults only,
//                            and homing relies on the limit switch plus a timeout (no
//                            stall detection). Motion math is unchanged: set each
//                            DM556TE to 3200 pulse/rev so one commanded unit is one
//                            pulse, exactly like the old 16-microstep scale.
//                   v0_10 -- Absorbed j4_stepper_eyes: the two eye-pop TMC2209s now
//                            live on this board, on the pins the DM556TE swap freed
//                            (EL step/dir 18/19, ER step/dir 23/5, TMC2209 UART on
//                            17/16, limits EL 35 / ER 22). The downstream UART link,
//                            its protocol, and the eyes-status relay are gone; the
//                            eye value is driven locally and "EY:<0/1>" now reports
//                            whether the local TMC2209s respond over UART. One
//                            calibrate() homes all five axes (eyes keep StallGuard4
//                            stall detection). j4_stepper_eyes is retired.
//                   v0_11 -- Renamed the project j4_stepper_neck -> j4_stepper (it is
//                            the only stepper board now, so the _neck suffix was
//                            stale). Dropped the "EY:<0/1>" status line: with the
//                            eyes board gone there is no separate link to report,
//                            and an unreachable eye driver already surfaces as
//                            "ST:EYES OFFLINE". j4_receiver / j4_controller drop
//                            their eyes_ok plumbing to match.
//
//
//
//  ------------------------------------------------------------------
//  UPSTREAM PACKET (from j4_receiver, UART1)
//  ------------------------------------------------------------------
//   "nL,nR,nP,eye\n"   four comma-separated integers, newline terminated
//     nL  -- neck left   (0-3200)     nR  -- neck right (0-3200)
//     nP  -- neck pivot  (0-3200)     eye -- eye-pop    (0-3200)
//   "CAL\n"            run a homing/calibration pass on all five axes
//   A line that does not parse is ignored.
//
//  UPSTREAM STATUS (to j4_receiver, UART1)
//   "ST:<text>\n"   e.g. "ST:OK", "ST:NL LIMIT", "ST:EL OT", "ST:EYES OFFLINE"
//  ------------------------------------------------------------------
//
//
//  ESP32-DEV (WROOM) Pin Connections
//  ------------------------------------------------------------------
//  Neck step/dir (each pair feeds one DM556TE's PUL / DIR opto inputs):
//   26:  STEP -- neck left      25:  DIR -- neck left
//   33:  STEP -- neck right     32:  DIR -- neck right
//   27:  STEP -- neck pivot     14:  DIR -- neck pivot
//
//  Eye-pop step/dir (one TMC2209 per eye, independent channels):
//   18:  STEP -- eye left       19:  DIR -- eye left
//   23:  STEP -- eye right      05:  DIR -- eye right
//
//  TMC2209 single-wire UART (shared bus, addressed via MS1/MS2 straps):
//   17:  UART2 TX -- TMC2209 PDN_UART (via 1k to each driver)
//   16:  UART2 RX -- TMC2209 PDN_UART
//        addresses: eye-left = 0b00, eye-right = 0b01
//
//  Upstream link to j4_receiver (TTGO), UART1, 2-way:
//   04:  UART1 RX  <-- TTGO GPIO21 (TX)
//   13:  UART1 TX  --> TTGO GPIO22 (RX)
//
//  Limit switches (two NC switches per axis in SERIES -> one input each, with an
//  external 10k pull-up to 3.3V; LOW = in range, HIGH = limit hit / broken wire):
//   34:  neck-left  limits      36:  neck-right limits      39:  neck-pivot limits
//   35:  eye-left   limits      22:  eye-right  limits
//
//  GPIO 22 also carries the onboard LED, so the LED doubles as an eye-right limit
//  indicator (lit = in range, dark = limit hit, with the usual active-low wiring).
//
//  ------------------------------------------------------------------
//  DM556TE DRIVER WIRING (per neck driver -- three identical)
//  ------------------------------------------------------------------
//  The DM556TE inputs are optically isolated (rated 5-24V, 7-20mA). Wire them
//  COMMON-CATHODE / active-high so the ESP32's normal 3.3V logic drives them
//  directly and the pulse/direction polarity stays normal (no firmware inversion):
//
//   ESP32 STEP pin --> PUL+        ESP32 DIR pin --> DIR+
//   PUL- --> ESP32 GND            DIR- --> ESP32 GND
//   ENA+ / ENA-  left unconnected  (driver is enabled by default)
//
//  3.3V is at the low edge of the opto's 5V spec. It works on the bench, but if an
//  axis drops pulses, add a 3.3V->5V level shifter on PUL/DIR, or drive the inputs
//  common-anode from a 5V rail (which inverts polarity -- not used here).
//
//  DIP SWITCHES (set on each DM556TE, NOT in firmware):
//   Current (SW1-3) + SW4 full/half: set to your motor. For the 2.1A neck motors
//     use the 2.1A-peak / 1.5A-RMS row: SW1=on, SW2=off, SW3=off, SW4=on (full).
//   Microstep (SW5-8): set 3200 pulse/rev -> SW5=on, SW6=on, SW7=off, SW8=on.
//     This makes one commanded unit below equal one pulse (same as the old scale).
//
//  Motor: A+/A- to one phase, B+/B- to the other. If an axis runs the wrong way,
//  swap that motor's A+/A- pair (or invert its DIR mapping in firmware).
//
//  Power:
//   DM556TE +Vdc/GND -> 24-50VDC PSU  (PSU GND common with ESP32 GND)
//   TMC2209 VIO -> 3.3V, VM -> 12-24V PSU, EN tied to GND (always enabled)
//  ------------------------------------------------------------------
//
//  NOTE: avoids strapping pins (0/2/12/15) for outputs and flash pins (6-11).
//        The input-only pins 34/35/36/39 carry four of the five limit inputs;
//        the fifth (eye-right) is on the ordinary GPIO 22.
//
//******************************************************************************



#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "FastAccelStepper.h"


// ---------------------------------------------------------------------------
// PIN ASSIGNMENTS
// ---------------------------------------------------------------------------
// Neck step/dir (each pair feeds one DM556TE PUL/DIR input)
#define STEP_PIN_NL   26
#define DIR_PIN_NL    25
#define STEP_PIN_NR   33
#define DIR_PIN_NR    32
#define STEP_PIN_NP   27
#define DIR_PIN_NP    14

// Eye-pop step/dir (one TMC2209 per eye, on the pins the DM556TE swap freed)
#define STEP_PIN_EL   18
#define DIR_PIN_EL    19
#define STEP_PIN_ER   23
#define DIR_PIN_ER     5

// TMC2209 single-wire UART bus (both drivers share it, addressed via MS1/MS2)
#define TMC2209_TX    17
#define TMC2209_RX    16

// Upstream link to the TTGO receiver (UART1, 2-way)
#define UP_RX_PIN      4
#define UP_TX_PIN     13

#define LINK_BAUD 250000   // must match j4_receiver


// ---------------------------------------------------------------------------
// MOTION CONFIG -- NECK (DM556TE)
// ---------------------------------------------------------------------------
// Motor current and microstep resolution live on the DM556TE DIP switches, not
// here. Set each driver to 3200 pulse/rev (SW5=on, SW6=on, SW7=off, SW8=on) so one
// commanded unit below equals one step pulse -- identical to the old 16-microstep
// math, so all the neck travel/threshold numbers carry over unchanged.
#define NECK_SPEED_HZ     3200    // pulses/sec ~ 60 RPM at 3200 pulse/rev
#define NECK_ACCEL       16000    // pulses/sec^2
#define NECK_THRESHOLD      20    // min joystick-unit change (0-3200) to re-issue moveTo
#define NECK_CENTER       1600    // joystick midpoint (0-3200)


// ---------------------------------------------------------------------------
// MOTION CONFIG -- EYES (TMC2209)
// ---------------------------------------------------------------------------
// Eye-pop motion matches the neck so it has the same smooth ramp. Only the
// driver current is separate (small eye motors).
#define R_SENSE_2209    0.11f   // typical BTT/FYSETC TMC2209 sense resistor -- verify on your board
#define EYE_CURRENT      800    // RMS mA -- tune to your eye-pop motors
#define EYE_MICROSTEPS    16
#define EYE_SPEED_HZ    3200    // same as the neck
#define EYE_ACCEL      16000    // same as the neck
#define EYE_THRESHOLD     20    // min change (0-3200 scale) to re-issue moveTo


// ---------------------------------------------------------------------------
// LIMIT SWITCHES
// ---------------------------------------------------------------------------
// Two normally-closed switches per axis wired IN SERIES to one input, with an
// external 10k pull-up to 3.3V. LOW = both closed (in range); HIGH = a limit was
// hit OR a wire broke (fail-safe). The travel direction tells which end. Pins
// 34/35/36/39 are input-only (no internal pull-up -- hence the external ones);
// eye-right rides on the ordinary GPIO 22, whose onboard LED becomes a free
// in-range indicator.
#define LIMIT_PIN_NL  34
#define LIMIT_PIN_NR  36
#define LIMIT_PIN_NP  39
#define LIMIT_PIN_EL  35
#define LIMIT_PIN_ER  22
static inline bool limitHit(uint8_t pin) { return digitalRead(pin) == HIGH; }

// StallGuard4 on the eye TMC2209s (read over UART). SG_RESULT drops toward 0
// under load; near 0 is a stall. Used during calibration so an eye already jammed
// against a stop is caught the moment it tries to move. The DM556TE neck drivers
// have no stall detection -- neck homing trusts the limit switch plus the timeout.
// THESE VALUES NEED HARDWARE TUNING.
#define SG_SGTHRS         60       // StallGuard4 threshold (0-255); higher = more sensitive
#define SG_TCOOLTHRS  0xFFFFF      // enable StallGuard above near-standstill speed
#define SG_STALL_LEVEL    40       // SG_RESULT below this counts as a stall (tune)
static inline bool stalled2209(TMC2209Stepper &drv) { return drv.SG_RESULT() < SG_STALL_LEVEL; }

// Homing/calibration tunables (per-hardware -- adjust on the bench).
#define HOMING_SPEED_HZ   800      // slow approach speed
#define HOMING_ACCEL     8000
#define HOMING_BACKOFF    200      // pulses to back off after touching a limit
#define HOMING_TIMEOUT_MS 9000     // safety cap per axis


// ---------------------------------------------------------------------------
// EYE DRIVERS (shared UART, separate addresses)
// ---------------------------------------------------------------------------
HardwareSerial tmcSerial(2);  // UART2 -> TMC2209 PDN_UART bus
TMC2209Stepper tmc_EL(&tmcSerial, R_SENSE_2209, 0b00);
TMC2209Stepper tmc_ER(&tmcSerial, R_SENSE_2209, 0b01);


// ---------------------------------------------------------------------------
// STEP GENERATORS (FastAccelStepper -- RMT hardware, no CPU blocking).
// The two eye channels are independent so they can be trimmed/homed separately,
// but both are commanded to the same target.
// ---------------------------------------------------------------------------
FastAccelStepperEngine engine;
FastAccelStepper *stepper_NL = NULL;
FastAccelStepper *stepper_NR = NULL;
FastAccelStepper *stepper_NP = NULL;
FastAccelStepper *stepper_EL = NULL;
FastAccelStepper *stepper_ER = NULL;

int last_nL  = NECK_CENTER;
int last_nR  = NECK_CENTER;
int last_nP  = NECK_CENTER;
int last_eye = 0;


// ---------------------------------------------------------------------------
// UPSTREAM LINK (UART1) + fixed-buffer parser (no Arduino String -> no heap churn)
// ---------------------------------------------------------------------------
HardwareSerial upSerial(1);   // UART1 -> TTGO receiver
char    upBuf[48];  uint8_t upLen = 0;

// Status plumbing
char neckFault[24] = "";   // current neck limit fault ("" = none)
char eyeFault[24]  = "";   // current eye driver/limit fault ("" = none)
bool eyesAlive     = false; // both TMC2209s responding over UART?

#define POLL_INTERVAL_MS   100   // how often to poll the TMC2209s + limit inputs
#define STATUS_TX_MS       200   // how often to report status upstream

unsigned long lastPoll     = 0;
unsigned long lastStatusTx = 0;


// Neck travel is half a joystick range; neck-left runs inverted (as in j4_receiver).
static inline int32_t neckTarget(int joy)    { return (int32_t)joy / 2; }
static inline int32_t neckTargetInv(int joy) { return (int32_t)(3200 - joy) / 2; }

// Eye-pop travel maps the same way (half a 0-3200 range).
static inline int32_t eyeTarget(int eye)     { return (int32_t)eye / 2; }


void applyNeck(int nL, int nR, int nP) {
  if (stepper_NL && abs(nL - last_nL) >= NECK_THRESHOLD) {
    last_nL = nL;
    stepper_NL->moveTo(neckTargetInv(nL));
  }
  if (stepper_NR && abs(nR - last_nR) >= NECK_THRESHOLD) {
    last_nR = nR;
    stepper_NR->moveTo(neckTarget(nR));
  }
  if (stepper_NP && abs(nP - last_nP) >= NECK_THRESHOLD) {
    last_nP = nP;
    stepper_NP->moveTo(neckTarget(nP));
  }
}


// Drive both eyes to the same target (one move command per channel).
void applyEye(int eye) {
  if (abs(eye - last_eye) < EYE_THRESHOLD) return;
  last_eye = eye;
  int32_t tgt = eyeTarget(eye);
  if (stepper_EL) stepper_EL->moveTo(tgt);
  if (stepper_ER) stepper_ER->moveTo(tgt);
}


bool calRequested = false;

// Parse "nL,nR,nP,eye" (drive all five axes) or "CAL" (request calibration).
void parseUpstream(const char *line) {
  if (strcmp(line, "CAL") == 0) { calRequested = true; return; }
  int nL, nR, nP, eye;
  if (sscanf(line, "%d,%d,%d,%d", &nL, &nR, &nP, &eye) == 4) {
    applyNeck(nL, nR, nP);
    applyEye(eye);
  }
}


// Return a short fault token for one TMC2209, most-severe first ("" = healthy).
const char *faultToken2209(TMC2209Stepper &d) {
  if (d.ot())                                         return "OT";     // overtemp shutdown
  if (d.s2ga() || d.s2gb() || d.s2vsa() || d.s2vsb()) return "SHORT";  // short ground/supply
  if (d.otpw())                                       return "OTPW";   // overtemp prewarning
  if (d.ola()  || d.olb())                            return "OPEN";   // open load
  return "";
}


// Poll everything that can fault. The DM556TE has no telemetry bus, so a neck
// fault can only be a limit input reading HIGH (a switch is hit or its wire is
// broken). The eye TMC2209s are polled over UART for driver faults and checked
// for a working link (an unreachable driver reports as "EYES OFFLINE").
void pollFaults() {
  neckFault[0] = '\0';
  if      (limitHit(LIMIT_PIN_NL)) strncpy(neckFault, "NL LIMIT", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NR)) strncpy(neckFault, "NR LIMIT", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NP)) strncpy(neckFault, "NP LIMIT", sizeof(neckFault) - 1);

  eyesAlive = (tmc_EL.test_connection() == 0) && (tmc_ER.test_connection() == 0);

  const char *t;
  eyeFault[0] = '\0';
  if      ((t = faultToken2209(tmc_EL))[0]) snprintf(eyeFault, sizeof(eyeFault), "EL %s", t);
  else if ((t = faultToken2209(tmc_ER))[0]) snprintf(eyeFault, sizeof(eyeFault), "ER %s", t);
  else if (limitHit(LIMIT_PIN_EL))          strncpy(eyeFault, "EL LIMIT", sizeof(eyeFault) - 1);
  else if (limitHit(LIMIT_PIN_ER))          strncpy(eyeFault, "ER LIMIT", sizeof(eyeFault) - 1);
}


// Safety: if an axis is moving and its limit trips, stop it immediately.
void checkLimits() {
  if (stepper_NL && stepper_NL->isRunning() && limitHit(LIMIT_PIN_NL)) stepper_NL->forceStop();
  if (stepper_NR && stepper_NR->isRunning() && limitHit(LIMIT_PIN_NR)) stepper_NR->forceStop();
  if (stepper_NP && stepper_NP->isRunning() && limitHit(LIMIT_PIN_NP)) stepper_NP->forceStop();
  if (stepper_EL && stepper_EL->isRunning() && limitHit(LIMIT_PIN_EL)) stepper_EL->forceStop();
  if (stepper_ER && stepper_ER->isRunning() && limitHit(LIMIT_PIN_ER)) stepper_ER->forceStop();
}


// Home one axis: drive toward the home (negative) end until the limit switch
// trips, StallGuard detects a hard stop (eye axes only -- pass NULL for the neck,
// whose DM556TEs have no stall detection), or the timeout expires; then back off
// until the limit clears and zero the position. NEEDS HARDWARE VALIDATION -- the
// direction, speed, backoff, and SG level are all tunable above.
void homeAxis(FastAccelStepper *s, uint8_t limitPin, TMC2209Stepper *drv,
              uint32_t runSpeed, int32_t runAccel) {
  if (!s) return;
  s->setSpeedInHz(HOMING_SPEED_HZ);
  s->setAcceleration(HOMING_ACCEL);

  // Approach the home (negative) end. With one shared series input we can't tell
  // the two ends apart, so only trust the limit switch once we've seen it read
  // clear (LOW) at least once -- that way starting against the FAR limit just
  // drives us off it and onward to home. On the eye axes, StallGuard stops a real
  // jam immediately (this catches an eye already pinned against the HOME stop).
  s->move(-1000000);
  bool seenClear = !limitHit(limitPin);
  unsigned long t0 = millis();
  while (s->isRunning()) {
    if (!limitHit(limitPin))                  seenClear = true;
    if (drv && stalled2209(*drv))             { s->forceStop(); break; }
    if (seenClear && limitHit(limitPin))      { s->forceStop(); break; }
    if (millis() - t0 > HOMING_TIMEOUT_MS)    { s->forceStop(); break; }
    delay(1);
  }
  delay(50);

  // Back off until the limit clears, then zero here.
  s->setCurrentPosition(0);
  s->move(HOMING_BACKOFF);
  t0 = millis();
  while (s->isRunning() && (millis() - t0 < HOMING_TIMEOUT_MS)) delay(1);
  s->setCurrentPosition(0);

  // Restore normal motion settings.
  s->setSpeedInHz(runSpeed);
  s->setAcceleration(runAccel);
}


// Calibrate all five axes: the three neck axes (limit + timeout only), then the
// two eyes independently (limit, StallGuard, or timeout).
void calibrate() {
  homeAxis(stepper_NL, LIMIT_PIN_NL, NULL,    NECK_SPEED_HZ, NECK_ACCEL);
  homeAxis(stepper_NR, LIMIT_PIN_NR, NULL,    NECK_SPEED_HZ, NECK_ACCEL);
  homeAxis(stepper_NP, LIMIT_PIN_NP, NULL,    NECK_SPEED_HZ, NECK_ACCEL);
  homeAxis(stepper_EL, LIMIT_PIN_EL, &tmc_EL, EYE_SPEED_HZ,  EYE_ACCEL);
  homeAxis(stepper_ER, LIMIT_PIN_ER, &tmc_ER, EYE_SPEED_HZ,  EYE_ACCEL);
  last_nL = last_nR = last_nP = NECK_CENTER;   // force a fresh move on next packet
  last_eye = -1;
}


// Build the single status string sent upstream. Priority: unreachable eye
// drivers, then a neck limit fault, then an eye driver/limit fault, otherwise OK.
void buildStatus(char *out, size_t n) {
  if (!eyesAlive) {
    strncpy(out, "EYES OFFLINE", n);
  } else if (neckFault[0]) {
    strncpy(out, neckFault, n);
  } else if (eyeFault[0]) {
    strncpy(out, eyeFault, n);
  } else {
    strncpy(out, "OK", n);
  }
  out[n - 1] = '\0';
}


void configTmc2209(TMC2209Stepper &drv) {
  drv.begin();
  drv.toff(4);
  drv.blank_time(24);
  drv.rms_current(EYE_CURRENT);
  drv.microsteps(EYE_MICROSTEPS);
  drv.en_spreadCycle(false);  // StealthChop (quiet)
  drv.pwm_autoscale(true);
  // StallGuard4 (used during calibration; needs tuning on real hardware)
  drv.TCOOLTHRS(SG_TCOOLTHRS);
  drv.SGTHRS(SG_SGTHRS);
}


FastAccelStepper *connectStepper(uint8_t stepPin, uint8_t dirPin, int32_t startPos,
                                 uint32_t speed, int32_t accel) {
  FastAccelStepper *s = engine.stepperConnectToPin(stepPin);
  if (s) {
    s->setDirectionPin(dirPin);
    s->setSpeedInHz(speed);
    s->setAcceleration(accel);
    s->setCurrentPosition(startPos);
  }
  return s;
}


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("j4_stepper booting...");

  // --- Upstream link ---
  upSerial.begin(LINK_BAUD, SERIAL_8N1, UP_RX_PIN, UP_TX_PIN);   // to TTGO

  // --- Limit switches (series NC, external pull-up -> plain INPUT) ---
  pinMode(LIMIT_PIN_NL, INPUT);
  pinMode(LIMIT_PIN_NR, INPUT);
  pinMode(LIMIT_PIN_NP, INPUT);
  pinMode(LIMIT_PIN_EL, INPUT);
  pinMode(LIMIT_PIN_ER, INPUT);

  // --- Eye TMC2209 drivers over shared UART ---
  tmcSerial.begin(115200, SERIAL_8N1, TMC2209_RX, TMC2209_TX);
  configTmc2209(tmc_EL);
  configTmc2209(tmc_ER);

  // --- Step generators (neck DM556TEs get PUL/DIR straight from these) ---
  engine.init();
  stepper_NL = connectStepper(STEP_PIN_NL, DIR_PIN_NL, neckTargetInv(NECK_CENTER), NECK_SPEED_HZ, NECK_ACCEL);
  stepper_NR = connectStepper(STEP_PIN_NR, DIR_PIN_NR, neckTarget(NECK_CENTER),    NECK_SPEED_HZ, NECK_ACCEL);
  stepper_NP = connectStepper(STEP_PIN_NP, DIR_PIN_NP, neckTarget(NECK_CENTER),    NECK_SPEED_HZ, NECK_ACCEL);
  stepper_EL = connectStepper(STEP_PIN_EL, DIR_PIN_EL, 0,                          EYE_SPEED_HZ,  EYE_ACCEL);
  stepper_ER = connectStepper(STEP_PIN_ER, DIR_PIN_ER, 0,                          EYE_SPEED_HZ,  EYE_ACCEL);

  if (!stepper_NL || !stepper_NR || !stepper_NP || !stepper_EL || !stepper_ER) {
    Serial.println("WARNING: a stepperConnectToPin() failed -- check STEP pins");
  } else {
    Serial.println("j4_stepper ready -- 3 neck + 2 eye motors, waiting for packets");
  }
}


void loop() {
  // Upstream: commands from the TTGO.
  while (upSerial.available()) {
    char c = (char)upSerial.read();
    if (c == '\n') {
      upBuf[upLen] = '\0';
      if (upLen > 0) parseUpstream(upBuf);
      upLen = 0;
    } else if (c != '\r' && upLen < sizeof(upBuf) - 1) {
      upBuf[upLen++] = c;
    } else if (c != '\r') {
      upLen = 0;  // overflow -- drop and resync on next newline
    }
  }

  // Run a calibration pass if one was requested (blocks while homing).
  if (calRequested) {
    calRequested = false;
    calibrate();
  }

  // Safety: stop any axis whose limit trips mid-move.
  checkLimits();

  unsigned long now = millis();

  // Poll the eye drivers + all limit inputs.
  if (now - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = now;
    pollFaults();
  }

  // Report the merged status upstream.
  if (now - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = now;
    char status[24];
    buildStatus(status, sizeof(status));
    upSerial.printf("ST:%s\n", status);
  }
}
