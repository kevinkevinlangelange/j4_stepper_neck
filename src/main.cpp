//******************************************************************************
//         file name:  j4_stepper_neck.ino
//      v0_1 created:  2026-06-14 -- 0123 CDT -KL
//      last updated:  2026-06-15 -- CDT
//      last updated:  2026-06-17 -- CDT
//      last updated:  2026-06-18 -- CDT
//      last updated:  2026-06-19 -- CDT
//      last updated:  2026-07-02 -- CDT
//      last updated:  2026-07-03 -- CDT
//      last updated:  2026-10-14 -- CDT
//      last updated:  2026-10-15 -- CDT
//            author:  Kevin Lange
//       description:  Neck stepper driver board for Johnny 4, on a bare
//                     ESP32-D0WD board (LOLIN32-Lite style). Drives the three neck steppers via
//                     STEPPERONLINE DM556TE digital step/dir drivers and acts as
//                     the middle node of the stepper daisy chain:
//
//                       j4_receiver (TTGO) <--UART1--> [ j4_stepper_neck ]
//                                                              <--UART2--> j4_stepper_eyes
//
//                     It parses "nL,nR,nP,eye" from the receiver, drives the neck
//                     with nL/nR/nP, and forwards the eye value downstream to the
//                     eyes board. The DM556TE drivers are standalone (current and
//                     microsteps on DIP switches, no telemetry), so neck health
//                     comes from the limit switches: each axis has its OWN input
//                     per endpoint (six total), so faults name the exact switch
//                     ("NL MIN", "NP MAX", ...). That is merged with the eyes
//                     board's status (and a link-alive check) into a single status
//                     string sent back upstream.
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
//                   v0_10 -- Absorbed j4_stepper_eyes onto this board (the project
//                            was briefly renamed j4_stepper).
//                   v0_11 -- Dropped the "EY:" status line while consolidated.
//                   v0_13 -- Doc update only (no code change): the six STEP/DIR
//                            signals now pass through two SN74AHCT125N quad
//                            buffers, shifting them from 3.3V to a full 5V before
//                            the DM556TE PUL+/DIR+ opto inputs -- squarely inside
//                            the opto's 5-24V spec instead of at its low edge.
//                            The buffers are non-inverting, so the firmware and
//                            the ESP32 pin map are untouched. Chip pinout and
//                            per-buffer wiring are documented below.
//                   v0_12 -- Split back into two boards (j4_stepper_neck +
//                            j4_stepper_eyes) and renamed back. The pins freed by
//                            moving the eyes off-board now give every limit switch
//                            its OWN input: one switch per axis endpoint, six
//                            inputs total, so faults name the exact switch ("NL
//                            MIN", "NP MAX", ...). MIN (home) ends on the
//                            input-only pins 34/36/39 (external 10k pull-ups); MAX
//                            ends on 18/19/23 (internal pull-ups). Homing now
//                            detects "already sitting on the home switch" directly
//                            and backs off first -- no stall sensing needed. Mid-
//                            move safety is direction-aware: only a move TOWARD a
//                            tripped end is stopped, so an axis can always drive
//                            itself back off a limit. The downstream link (TX 22 /
//                            RX 35), eyes status relay, and "EY:<0/1>" are back.
//                   2026-10-14 -- Doc only (no code change): level shifters are now
//                            SN74ACT125N, pin-identical to the AHCT125 but rated
//                            24mA per output / 200mA per chip. Four DM556TE optos
//                            at up to 16mA each exceed the AHCT125's 50mA chip
//                            limit. Board named correctly (LOLIN32-Lite style,
//                            not ESP32-DEV WROOM); spare/strapping pins listed.
//                   2026-10-15 -- Doc only (no code change): back to SN74AHCT125N
//                            (the ACT125 is hard to buy), now THREE chips, one
//                            per DM556TE, each signal on two paralleled buffers
//                            of the same chip. Each buffer carries 5-8mA, inside
//                            the AHCT125's 8mA/output rating; each chip 32mA
//                            worst case, under its 50mA limit.
//
//
//
//  ------------------------------------------------------------------
//  UPSTREAM PACKET (from j4_receiver, UART1)
//  ------------------------------------------------------------------
//   "nL,nR,nP,eye\n"   four comma-separated integers, newline terminated
//     nL  -- neck left   (0-3200)     nR  -- neck right (0-3200)
//     nP  -- neck pivot  (0-3200)     eye -- eye-pop    (0-3200), forwarded down
//   "CAL\n"            run a homing/calibration pass (also forwarded downstream)
//   A line that does not parse is ignored.
//
//  UPSTREAM STATUS (to j4_receiver, UART1)
//   "ST:<text>\n"   e.g. "ST:OK", "ST:NL MIN", "ST:NP MAX", "ST:EYES OFFLINE"
//   "EY:<0/1>\n"    eyes-board link state (1 = alive)
//
//  DOWNSTREAM (to j4_stepper_eyes, UART2)
//   out: "<eye>\n"          the forwarded eye-pop value (0-3200)
//   out: "CAL\n"            tell the eyes board to calibrate too
//   in:  "S:<text>\n"       the eyes board's status ("S:OK", "S:EL MIN", ...)
//  ------------------------------------------------------------------
//
//
//  ESP32-D0WD (LOLIN32-Lite style) Pin Connections
//  ------------------------------------------------------------------
//  Neck step/dir (each pair feeds one DM556TE's PUL / DIR opto inputs):
//   26:  STEP -- neck left      25:  DIR -- neck left
//   33:  STEP -- neck right     32:  DIR -- neck right
//   27:  STEP -- neck pivot     14:  DIR -- neck pivot
//
//  Upstream link to j4_receiver (TTGO), UART1, 2-way:
//   04:  UART1 RX  <-- TTGO GPIO21 (TX)
//   13:  UART1 TX  --> TTGO GPIO22 (RX)
//
//  Downstream link to j4_stepper_eyes, UART2, 2-way:
//   22:  UART2 TX  --> eyes GPIO4 (RX)    (also drives the onboard LED = TX blink)
//   35:  UART2 RX  <-- eyes GPIO13 (TX)   (input-only)
//
//  NOTE: this board (ESP32-D0WD) does NOT break out GPIO21, so the downstream TX
//        is on GPIO22 (which carries the onboard LED -- a handy TX-activity blink).
//
//  Limit switches -- ONE NC switch per axis endpoint, six inputs total. Each
//  switch connects its input to GND; closed (in range) = LOW, open (limit hit
//  OR broken wire) = HIGH, fail-safe. MIN = the home end an axis drives toward
//  during CAL; MAX = the far end.
//   MIN (input-only pins, external 10k pull-up to 3.3V):
//    34:  neck-left MIN          36:  neck-right MIN         39:  neck-pivot MIN
//   MAX (ordinary pins, internal pull-ups -- no external resistor needed):
//    18:  neck-left MAX          19:  neck-right MAX         23:  neck-pivot MAX
//
//  ------------------------------------------------------------------
//  DM556TE DRIVER WIRING (per driver -- three identical)
//  ------------------------------------------------------------------
//  The DM556TE inputs are optically isolated (rated 5-24V, 7-16mA). The six
//  ESP32 STEP/DIR signals pass through three SN74AHCT125N quad buffers (below),
//  which shift them from 3.3V to a full 5V. The drivers stay wired
//  COMMON-CATHODE / active-high, so polarity stays normal (no firmware
//  inversion):
//
//   74AHCT125 1Y+2Y --> PUL+      74AHCT125 3Y+4Y --> DIR+
//   PUL- --> ESP32 GND            DIR- --> ESP32 GND
//   ENA+ / ENA-  left unconnected  (driver is enabled by default)
//
//  ------------------------------------------------------------------
//  SN74AHCT125N LEVEL SHIFTERS (3 chips, U1 + U2 + U3, one per driver)
//  ------------------------------------------------------------------
//  Quad bus buffer, non-inverting. AHCT inputs have TTL thresholds (VIH = 2.0V),
//  so 3.3V ESP32 logic drives them directly; outputs swing to VCC = 5V, putting
//  the DM556TE optos squarely inside their 5-24V spec. Every /OE (active-low
//  output enable) ties to GND so the buffers are always on. Put a 100nF
//  decoupling cap across VCC/GND of each chip, close to the pins.
//
//  Each opto input draws 7-16mA, more than one AHCT125 buffer is rated for
//  (8mA), so every signal drives TWO buffers of the same chip in parallel:
//  inputs tied together, outputs tied together. Each buffer then carries
//  5-8mA, and each chip (two optos) at most 32mA of its 50mA limit. Only
//  parallel buffers inside ONE chip, never across chips.
//
//                SN74AHCT125N (DIP-14, top view, notch at the top)
//
//                                 +---U---+
//               1/OE (to GND) -- 1|       |14 -- VCC (5V from the ESP32 board)
//               1A  (STEP in) -- 2|       |13 -- 4/OE (to GND)
//               1Y  (PUL+)    -- 3|       |12 -- 4A  (DIR in)
//               2/OE (to GND) -- 4|       |11 -- 4Y  (DIR+)
//               2A  (STEP in) -- 5|       |10 -- 3/OE (to GND)
//               2Y  (PUL+)    -- 6|       | 9 -- 3A  (DIR in)
//               GND           -- 7|       | 8 -- 3Y  (DIR+)
//                                 +-------+
//
//  Same wiring on all three chips:
//   pins 2 + 5  (1A+2A) <- that axis's STEP pin    pins 3 + 6  (1Y+2Y) -> its PUL+
//   pins 9 + 12 (3A+4A) <- that axis's DIR pin     pins 8 + 11 (3Y+4Y) -> its DIR+
//
//   U1 neck-left:   STEP 26, DIR 25 -> neck-left  DM556TE
//   U2 neck-right:  STEP 33, DIR 32 -> neck-right DM556TE
//   U3 neck-pivot:  STEP 27, DIR 14 -> neck-pivot DM556TE
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
//  ------------------------------------------------------------------
//
//  NOTE: avoids strapping pins (0/2/5/12/15) and flash pins (6-11). Spare pins:
//        16 and 17 are fully free; 0/2/5/12/15 are strapping pins (GPIO 12 must
//        never be held high at power-on or the board will not boot).
//
//******************************************************************************



#include <Arduino.h>
#include <HardwareSerial.h>
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

// Upstream link to the TTGO receiver (UART1, 2-way)
#define UP_RX_PIN      4
#define UP_TX_PIN     13

// Downstream link to the eyes board (UART2, 2-way).
// This board (ESP32-D0WD) does NOT break out GPIO21, so the downstream TX uses
// GPIO22 -- which also drives the onboard LED, giving a free TX-activity blink.
// RX is GPIO35 (input-only); this board has no battery-sense pin to conflict.
#define DN_TX_PIN     22
#define DN_RX_PIN     35

#define LINK_BAUD 250000   // both links; must match j4_receiver and j4_stepper_eyes


// ---------------------------------------------------------------------------
// MOTION CONFIG
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
// LIMIT SWITCHES -- one NC switch per axis endpoint (six inputs).
// ---------------------------------------------------------------------------
// Each switch shorts its input to GND while the axis is in range: LOW = in
// range, HIGH = that endpoint was hit OR its wire broke (fail-safe). Since every
// endpoint has its own input, faults can name the exact switch.
// MIN = the home end (CAL drives toward it); MAX = the far end.
// The MIN inputs are on input-only pins (34/36/39), which have no internal
// pull-up -- they need an external 10k to 3.3V. The MAX inputs (18/19/23) are
// ordinary GPIOs and use the internal pull-up: no external resistor needed.
#define LIMIT_PIN_NL_MIN  34
#define LIMIT_PIN_NR_MIN  36
#define LIMIT_PIN_NP_MIN  39
#define LIMIT_PIN_NL_MAX  18
#define LIMIT_PIN_NR_MAX  19
#define LIMIT_PIN_NP_MAX  23
static inline bool limitHit(uint8_t pin) { return digitalRead(pin) == HIGH; }

// Homing/calibration tunables (per-hardware -- adjust on the bench). Homing
// drives toward the MIN switch; an axis already sitting on it is detected
// directly (the switch reads pressed) and backs off first.
#define HOMING_SPEED_HZ   800      // slow approach speed
#define HOMING_ACCEL     8000
#define HOMING_BACKOFF    200      // pulses to back off after touching a limit
#define HOMING_TIMEOUT_MS 9000     // safety cap per axis


// ---------------------------------------------------------------------------
// STEP GENERATORS (FastAccelStepper -- RMT hardware, no CPU blocking)
// ---------------------------------------------------------------------------
FastAccelStepperEngine engine;
FastAccelStepper *stepper_NL = NULL;
FastAccelStepper *stepper_NR = NULL;
FastAccelStepper *stepper_NP = NULL;

int last_nL = NECK_CENTER;
int last_nR = NECK_CENTER;
int last_nP = NECK_CENTER;


// ---------------------------------------------------------------------------
// LINKS + parsers (fixed buffers, no Arduino String -> no heap churn)
// ---------------------------------------------------------------------------
HardwareSerial upSerial(1);   // UART1 -> TTGO receiver
HardwareSerial dnSerial(2);   // UART2 -> eyes board
char    upBuf[48];  uint8_t upLen = 0;
char    dnBuf[48];  uint8_t dnLen = 0;

// Status plumbing
char          eyesStatus[24] = "";      // last status text from the eyes board
bool          eyesSeen       = false;   // have we ever heard from the eyes board?
unsigned long eyesLastSeen   = 0;
char          neckFault[24]  = "";      // current neck limit fault ("" = none)

#define EYES_TIMEOUT_MS    800   // no eyes status for this long -> "EYES OFFLINE"
#define STATUS_TX_MS       200   // how often to report status upstream

unsigned long lastStatusTx = 0;


// Neck travel is half a joystick range; neck-left runs inverted (as in j4_receiver).
static inline int32_t neckTarget(int joy)    { return (int32_t)joy / 2; }
static inline int32_t neckTargetInv(int joy) { return (int32_t)(3200 - joy) / 2; }


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


bool calRequested = false;

// Parse "nL,nR,nP,eye" (drive neck + forward eye) or "CAL" (request calibration).
void parseUpstream(const char *line) {
  if (strcmp(line, "CAL") == 0) { calRequested = true; return; }
  int nL, nR, nP, eye;
  if (sscanf(line, "%d,%d,%d,%d", &nL, &nR, &nP, &eye) == 4) {
    applyNeck(nL, nR, nP);
    dnSerial.printf("%d\n", eye);   // relay to the eyes board
  }
}


// Parse "S:<text>" from the eyes board.
void parseDownstream(const char *line) {
  if (line[0] == 'S' && line[1] == ':') {
    strncpy(eyesStatus, line + 2, sizeof(eyesStatus) - 1);
    eyesStatus[sizeof(eyesStatus) - 1] = '\0';
    eyesSeen     = true;
    eyesLastSeen = millis();
  }
}


// With one input per endpoint the fault can name the exact switch. Record the
// first one found (a HIGH input = switch hit or its wire broke).
void updateNeckFault() {
  neckFault[0] = '\0';
  if      (limitHit(LIMIT_PIN_NL_MIN)) strncpy(neckFault, "NL MIN", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NL_MAX)) strncpy(neckFault, "NL MAX", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NR_MIN)) strncpy(neckFault, "NR MIN", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NR_MAX)) strncpy(neckFault, "NR MAX", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NP_MIN)) strncpy(neckFault, "NP MIN", sizeof(neckFault) - 1);
  else if (limitHit(LIMIT_PIN_NP_MAX)) strncpy(neckFault, "NP MAX", sizeof(neckFault) - 1);
}


// Safety: if an axis is moving TOWARD a tripped endpoint, stop it. Knowing which
// end tripped means a move away from the switch is still allowed, so an axis can
// always drive itself back off a limit.
void checkAxisLimits(FastAccelStepper *s, uint8_t minPin, uint8_t maxPin) {
  if (!s || !s->isRunning()) return;
  bool movingDown = s->targetPos() < s->getCurrentPosition();
  if (movingDown  && limitHit(minPin)) s->forceStop();
  if (!movingDown && limitHit(maxPin)) s->forceStop();
}

void checkLimits() {
  checkAxisLimits(stepper_NL, LIMIT_PIN_NL_MIN, LIMIT_PIN_NL_MAX);
  checkAxisLimits(stepper_NR, LIMIT_PIN_NR_MIN, LIMIT_PIN_NR_MAX);
  checkAxisLimits(stepper_NP, LIMIT_PIN_NP_MIN, LIMIT_PIN_NP_MAX);
}


// Home one axis toward its MIN switch. With a dedicated input per endpoint an
// axis already sitting on the home switch is detected immediately (no stall
// sensing needed): it just backs off until the switch releases, then zeros.
// NEEDS HARDWARE VALIDATION -- direction, speed, and backoff are tunable above.
void homeAxis(FastAccelStepper *s, uint8_t minPin, uint8_t maxPin) {
  if (!s) return;
  s->setSpeedInHz(HOMING_SPEED_HZ);
  s->setAcceleration(HOMING_ACCEL);
  unsigned long t0 = millis();

  // Approach the MIN end unless we are already pressing its switch.
  if (!limitHit(minPin)) {
    s->move(-1000000);
    while (s->isRunning()) {
      if (limitHit(minPin))                  { s->forceStop(); break; }
      if (limitHit(maxPin))                  { s->forceStop(); break; }  // wrong end / wiring fault
      if (millis() - t0 > HOMING_TIMEOUT_MS) { s->forceStop(); break; }
      delay(1);
    }
    delay(50);
  }

  // Back off until the MIN switch releases (retry in fixed nudges), then zero.
  s->setCurrentPosition(0);
  t0 = millis();
  while (limitHit(minPin) && (millis() - t0 < HOMING_TIMEOUT_MS)) {
    s->move(HOMING_BACKOFF);
    while (s->isRunning() && limitHit(minPin) &&
           (millis() - t0 < HOMING_TIMEOUT_MS)) delay(1);
  }
  s->forceStop();
  delay(50);
  s->setCurrentPosition(0);

  // Restore normal motion settings.
  s->setSpeedInHz(NECK_SPEED_HZ);
  s->setAcceleration(NECK_ACCEL);
}


// Calibrate all neck axes, then ask the eyes board to calibrate too.
void calibrate() {
  dnSerial.println("CAL");           // tell the eyes board to home as well
  homeAxis(stepper_NL, LIMIT_PIN_NL_MIN, LIMIT_PIN_NL_MAX);
  homeAxis(stepper_NR, LIMIT_PIN_NR_MIN, LIMIT_PIN_NR_MAX);
  homeAxis(stepper_NP, LIMIT_PIN_NP_MIN, LIMIT_PIN_NP_MAX);
  last_nL = last_nR = last_nP = NECK_CENTER;   // force a fresh move on next packet
}


// Build the single status string sent upstream. Priority: dead eyes link, then a
// neck limit fault, then an eye fault relayed from the eyes board, otherwise OK.
void buildStatus(char *out, size_t n) {
  if (!eyesSeen || (millis() - eyesLastSeen) > EYES_TIMEOUT_MS) {
    strncpy(out, "EYES OFFLINE", n);
  } else if (neckFault[0]) {
    strncpy(out, neckFault, n);
  } else if (eyesStatus[0] && strcmp(eyesStatus, "OK") != 0) {
    strncpy(out, eyesStatus, n);
  } else {
    strncpy(out, "OK", n);
  }
  out[n - 1] = '\0';
}


FastAccelStepper *connectStepper(uint8_t stepPin, uint8_t dirPin, int32_t startPos) {
  FastAccelStepper *s = engine.stepperConnectToPin(stepPin);
  if (s) {
    s->setDirectionPin(dirPin);
    s->setSpeedInHz(NECK_SPEED_HZ);
    s->setAcceleration(NECK_ACCEL);
    s->setCurrentPosition(startPos);
  }
  return s;
}


void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("j4_stepper_neck booting...");

  // --- Links ---
  upSerial.begin(LINK_BAUD, SERIAL_8N1, UP_RX_PIN, UP_TX_PIN);   // to TTGO
  dnSerial.begin(LINK_BAUD, SERIAL_8N1, DN_RX_PIN, DN_TX_PIN);   // to eyes board

  // --- Limit switches, one per endpoint ---
  // MIN ends: input-only pins, external 10k pull-up -> plain INPUT.
  pinMode(LIMIT_PIN_NL_MIN, INPUT);
  pinMode(LIMIT_PIN_NR_MIN, INPUT);
  pinMode(LIMIT_PIN_NP_MIN, INPUT);
  // MAX ends: ordinary pins, internal pull-up.
  pinMode(LIMIT_PIN_NL_MAX, INPUT_PULLUP);
  pinMode(LIMIT_PIN_NR_MAX, INPUT_PULLUP);
  pinMode(LIMIT_PIN_NP_MAX, INPUT_PULLUP);

  // --- Step generators (DM556TE PUL/DIR are driven straight from these) ---
  engine.init();
  stepper_NL = connectStepper(STEP_PIN_NL, DIR_PIN_NL, neckTargetInv(NECK_CENTER));
  stepper_NR = connectStepper(STEP_PIN_NR, DIR_PIN_NR, neckTarget(NECK_CENTER));
  stepper_NP = connectStepper(STEP_PIN_NP, DIR_PIN_NP, neckTarget(NECK_CENTER));

  if (!stepper_NL || !stepper_NR || !stepper_NP) {
    Serial.println("WARNING: a stepperConnectToPin() failed -- check STEP pins");
  } else {
    Serial.println("j4_stepper_neck ready -- 3 neck motors (DM556TE), waiting for packets");
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

  // Downstream: status from the eyes board.
  while (dnSerial.available()) {
    char c = (char)dnSerial.read();
    if (c == '\n') {
      dnBuf[dnLen] = '\0';
      if (dnLen > 0) parseDownstream(dnBuf);
      dnLen = 0;
    } else if (c != '\r' && dnLen < sizeof(dnBuf) - 1) {
      dnBuf[dnLen++] = c;
    } else if (c != '\r') {
      dnLen = 0;
    }
  }

  // Run a calibration pass if one was requested (blocks while homing).
  if (calRequested) {
    calRequested = false;
    calibrate();
  }

  // Safety: stop any axis moving toward a tripped endpoint.
  checkLimits();

  unsigned long now = millis();

  // Report merged status + the eyes-board link state upstream.
  if (now - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = now;
    updateNeckFault();
    char status[24];
    buildStatus(status, sizeof(status));
    upSerial.printf("ST:%s\n", status);
    bool eyesLinkOk = eyesSeen && (now - eyesLastSeen <= EYES_TIMEOUT_MS);
    upSerial.printf("EY:%d\n", eyesLinkOk ? 1 : 0);
  }
}
