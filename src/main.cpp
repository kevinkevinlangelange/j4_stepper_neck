//******************************************************************************
//         file name:  j4_stepper_neck.ino
//      v0_1 created:  2026-06-14 -- 0123 CDT -KL
//      last updated:  2026-06-15 -- CDT
//      last updated:  2026-06-17 -- CDT
//            author:  Kevin Lange
//       description:  Neck stepper driver board for Johnny 4, on an ESP32-DEV
//                     (WROOM) module. Drives the three neck steppers via BTT
//                     TMC5160T drivers and acts as the middle node of the stepper
//                     daisy chain:
//
//                       j4_receiver (TTGO) <--UART1--> [ j4_stepper_neck ]
//                                                              <--UART2--> j4_stepper_eyes
//
//                     It parses "nL,nR,nP,eye" from the receiver, drives the neck
//                     with nL/nR/nP, and forwards the eye value downstream to the
//                     eyes board. For status it polls each TMC5160T's DRV_STATUS
//                     register (overtemp, short, open-load), merges that with the
//                     eyes board's status (and a link-alive check), and reports a
//                     single status string back upstream to the receiver.
//
//                     Step pulses come from the ESP32 RMT peripheral via
//                     FastAccelStepper (no CPU blocking); the TMC5160T drivers are
//                     configured over software SPI.
//       update log:  v0_1 -- Initial build (combined neck + eye board).
//                    v0_2 -- Eye-pop bound to neck motion settings.
//                    v0_3 -- Split into two boards. This board is neck-only now;
//                            the eye-pop drivers moved to j4_stepper_eyes. Added a
//                            2-way upstream link, a downstream link to the eyes
//                            board (relays the eye value), DRV_STATUS fault polling
//                            on the three neck drivers, and a merged status report.
//
//
//
//  ------------------------------------------------------------------
//  UPSTREAM PACKET (from j4_receiver, UART1)
//  ------------------------------------------------------------------
//   "nL,nR,nP,eye\n"   four comma-separated integers, newline terminated
//     nL  -- neck left   (0-3200)     nR  -- neck right (0-3200)
//     nP  -- neck pivot  (0-3200)     eye -- eye-pop    (0-3200), forwarded down
//   A line that does not parse to four ints is ignored.
//
//  UPSTREAM STATUS (to j4_receiver, UART1)
//   "ST:<text>\n"   e.g. "ST:OK", "ST:NL OT", "ST:EYES OFFLINE"
//
//  DOWNSTREAM (to j4_stepper_eyes, UART2)
//   out: "<eye>\n"          the forwarded eye-pop value (0-3200)
//   in:  "S:<text>\n"       the eyes board's status ("S:OK", "S:EL OT", ...)
//  ------------------------------------------------------------------
//
//
//  ESP32-DEV (WROOM) Pin Connections
//  ------------------------------------------------------------------
//  TMC5160T software SPI (shared SCK/MISO/MOSI, one CS per driver):
//   18:  SPI SCK   -- all three TMC5160T (shared)
//   19:  SPI MISO  -- all three TMC5160T SDO (shared)
//   23:  SPI MOSI  -- all three TMC5160T SDI (shared)
//   05:  CS -- neck-left  TMC5160T
//   17:  CS -- neck-right TMC5160T
//   16:  CS -- neck-pivot TMC5160T
//
//  Neck step/dir:
//   26:  STEP -- neck left      25:  DIR -- neck left
//   33:  STEP -- neck right     32:  DIR -- neck right
//   27:  STEP -- neck pivot     14:  DIR -- neck pivot
//
//  Upstream link to j4_receiver (TTGO), UART1, 2-way:
//   04:  UART1 RX  <-- TTGO GPIO21 (TX)
//   13:  UART1 TX  --> TTGO GPIO22 (RX)
//
//  Downstream link to j4_stepper_eyes, UART2, 2-way:
//   21:  UART2 TX  --> eyes GPIO4 (RX)
//   22:  UART2 RX  <-- eyes GPIO13 (TX)
//
//  Power / enable:
//   3.3V -> all driver VIO       GND -> all driver GND (common with ESP32)
//   12-24V PSU -> all driver VM  (PSU GND common with ESP32 GND)
//   All driver EN pins tied to GND -- drivers always enabled.
//  ------------------------------------------------------------------
//
//  NOTE: avoids strapping pins (0/2/12/15) for outputs and flash pins (6-11).
//        On WROVER modules GPIO 16/17 are used by PSRAM -- this build targets
//        a plain WROOM ESP32-DEV where 16/17 are free.
//
//******************************************************************************



#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include "FastAccelStepper.h"


// ---------------------------------------------------------------------------
// PIN ASSIGNMENTS
// ---------------------------------------------------------------------------
// TMC5160T software SPI (shared bus)
#define SPI_SCK_PIN   18
#define SPI_MISO_PIN  19
#define SPI_MOSI_PIN  23
#define CS_PIN_NL      5   // neck left
#define CS_PIN_NR     17   // neck right
#define CS_PIN_NP     16   // neck pivot

// Neck step/dir
#define STEP_PIN_NL   26
#define DIR_PIN_NL    25
#define STEP_PIN_NR   33
#define DIR_PIN_NR    32
#define STEP_PIN_NP   27
#define DIR_PIN_NP    14

// Upstream link to the TTGO receiver (UART1, 2-way)
#define UP_RX_PIN      4
#define UP_TX_PIN     13

// Downstream link to the eyes board (UART2, 2-way)
#define DN_TX_PIN     21
#define DN_RX_PIN     22

#define LINK_BAUD 250000   // both links; must match j4_receiver and j4_stepper_eyes


// ---------------------------------------------------------------------------
// DRIVER + MOTION CONFIG  (matched to j4_motor_test_dev so the neck feels right)
// ---------------------------------------------------------------------------
#define R_SENSE_5160      0.075f  // BTT TMC5160T V1.0 sense resistor
#define NECK_CURRENT      2100    // RMS mA -- motor rated current
#define NECK_MICROSTEPS     16
#define NECK_SPEED_HZ     3200    // microsteps/sec (16x) ~ 60 RPM on a 1.8deg motor
#define NECK_ACCEL       16000    // microsteps/sec^2
#define NECK_THRESHOLD      20    // min joystick-unit change (0-3200) to re-issue moveTo
#define NECK_CENTER       1600    // joystick midpoint (0-3200)


// ---------------------------------------------------------------------------
// DRIVERS  (software SPI: TMC5160Stepper(CS, RS, MOSI, MISO, SCK))
// ---------------------------------------------------------------------------
TMC5160Stepper tmc_NL(CS_PIN_NL, R_SENSE_5160, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);
TMC5160Stepper tmc_NR(CS_PIN_NR, R_SENSE_5160, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);
TMC5160Stepper tmc_NP(CS_PIN_NP, R_SENSE_5160, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);


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
char          neckFault[24]  = "";      // current neck driver fault ("" = none)

#define EYES_TIMEOUT_MS    800   // no eyes status for this long -> "EYES OFFLINE"
#define POLL_INTERVAL_MS   100   // how often to read the neck DRV_STATUS registers
#define STATUS_TX_MS       200   // how often to report status upstream

unsigned long lastPoll     = 0;
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


// Parse "nL,nR,nP,eye": drive the neck and forward the eye value downstream.
void parseUpstream(const char *line) {
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


// Return a short fault token for one TMC5160T, most-severe first ("" = healthy).
const char *faultToken5160(TMC5160Stepper &d) {
  if (d.ot())                return "OT";     // overtemp shutdown
  if (d.s2ga() || d.s2gb())  return "SHORT";  // short to ground
  if (d.otpw())              return "OTPW";   // overtemp prewarning
  if (d.ola()  || d.olb())   return "OPEN";   // open load / disconnected
  return "";
}


// Poll the three neck drivers; record the first fault found (NL, then NR, then NP).
void pollNeckFaults() {
  const char *t;
  neckFault[0] = '\0';
  if ((t = faultToken5160(tmc_NL))[0]) { snprintf(neckFault, sizeof(neckFault), "NL %s", t); return; }
  if ((t = faultToken5160(tmc_NR))[0]) { snprintf(neckFault, sizeof(neckFault), "NR %s", t); return; }
  if ((t = faultToken5160(tmc_NP))[0]) { snprintf(neckFault, sizeof(neckFault), "NP %s", t); return; }
}


// Build the single status string sent upstream. Priority: dead eyes link, then a
// neck driver fault, then an eye driver fault, otherwise OK.
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


void configTmc5160(TMC5160Stepper &drv) {
  drv.begin();
  drv.GSTAT(0);          // clear latched fault flags
  drv.toff(4);           // chopper off time (0 disables the driver)
  drv.blank_time(24);
  drv.rms_current(NECK_CURRENT);
  drv.microsteps(NECK_MICROSTEPS);
  drv.en_pwm_mode(true); // StealthChop
  drv.pwm_autoscale(true);
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

  // --- TMC5160T neck drivers over software SPI ---
  configTmc5160(tmc_NL);
  configTmc5160(tmc_NR);
  configTmc5160(tmc_NP);

  // --- Step generators ---
  engine.init();
  stepper_NL = connectStepper(STEP_PIN_NL, DIR_PIN_NL, neckTargetInv(NECK_CENTER));
  stepper_NR = connectStepper(STEP_PIN_NR, DIR_PIN_NR, neckTarget(NECK_CENTER));
  stepper_NP = connectStepper(STEP_PIN_NP, DIR_PIN_NP, neckTarget(NECK_CENTER));

  if (!stepper_NL || !stepper_NR || !stepper_NP) {
    Serial.println("WARNING: a stepperConnectToPin() failed -- check STEP pins");
  } else {
    Serial.println("j4_stepper_neck ready -- 3 neck motors, waiting for packets");
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

  unsigned long now = millis();

  // Poll the neck drivers' fault registers.
  if (now - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = now;
    pollNeckFaults();
  }

  // Report merged status upstream.
  if (now - lastStatusTx >= STATUS_TX_MS) {
    lastStatusTx = now;
    char status[24];
    buildStatus(status, sizeof(status));
    upSerial.printf("ST:%s\n", status);
  }
}
