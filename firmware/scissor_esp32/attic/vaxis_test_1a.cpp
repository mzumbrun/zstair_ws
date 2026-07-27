/*
 * HARP V-Axis — Test 1A: Standalone RoboClaw serial bring-up (NO micro-ROS)
 * Target : ESP32 WROOM-32, Basicmicro library (basicmicro_arduino)
 * Link   : UART2  TX2=GPIO17 -> RoboClaw S1,  RX2=GPIO16 <- RoboClaw S2,
 *          common GND, 115200 baud, packet-serial addr 0x80.
 *
 * Goal   : prove ReadVersion + ReadEncM1 + a small bidirectional
 *          SpeedAccelDeccelPositionM1 move, triggered from the USB console.
 *          Isolates "is the link alive" from any micro-ROS question.
 *
 * INIT NOTE (verified from library + arduino-esp32 core source):
 *   Basicmicro::begin(speed) only calls hserial->begin(speed) with NO pin args,
 *   so the library does NOT set the UART pins. On the current esp32 core (3.x)
 *   classic-ESP32 Serial2 defaults are GPIO4/GPIO25, NOT 16/17. We therefore set
 *   the pins EXPLICITLY below and do NOT call motors.begin() (the object only
 *   needs the Stream up). This is correct on both core 2.x and 3.x.
 */

#include <Arduino.h>
#include <Basicmicro.h>

// ---------------- Link config ----------------
static const uint8_t   RC_ADDR   = 0x80;
static const uint32_t  RC_BAUD   = 115200;
static const int       RC_RX_PIN = 16;   // ESP32 RX2 <- RoboClaw S2 (its TX)
static const int       RC_TX_PIN = 17;   // ESP32 TX2 -> RoboClaw S1 (its RX)

// Basicmicro timeout is in MICROSECONDS. 10000 us = 10 ms (fine for standalone;
// drop to ~3000-5000 us later so a stalled read can't eat a 100 Hz micro-ROS cycle).
Basicmicro motors(&Serial2, 10000);

// ---------------- Motion config (encoder counts; 796.1 counts/mm screw) ----------------
static const float    COUNTS_PER_MM = 796.1f;
static const float    NUDGE_MM      = 3.0f;
static const long     NUDGE_COUNTS  = (long)(NUDGE_MM * COUNTS_PER_MM + 0.5f); // ~2388
static const uint32_t MV_SPEED      = 1000;   // counts/s   (~1.26 mm/s; QPPS max 2640)
static const uint32_t MV_ACCEL      = 2000;   // counts/s^2 (also used for deccel)
static const long     SOFT_MIN      = 200;    // stay off the 0 / 10000 bench limits
static const long     SOFT_MAX      = 9800;

// ---------------- Helpers ----------------
void printVersion() {
  char ver[64] = {0};
  if (motors.ReadVersion(RC_ADDR, ver)) {
    Serial.print("ReadVersion OK: ");
    Serial.print(ver);   // firmware string already ends in newline
  } else {
    Serial.println("ReadVersion FAILED (no reply) -> pins? baud? addr? TX/RX swap? ground? power?");
  }
}

long readEnc(bool *okOut = nullptr) {
  uint8_t status; bool valid = false;
  uint32_t enc = motors.ReadEncM1(RC_ADDR, &status, &valid);
  if (okOut) *okOut = valid;
  return (long)enc;
}

void printEnc() {
  bool ok;
  long e = readEnc(&ok);
  Serial.print("ReadEncM1: "); Serial.print(e);
  Serial.print("  ("); Serial.print(e / COUNTS_PER_MM, 3); Serial.print(" mm)  valid=");
  Serial.println(ok ? "true" : "false");
}

void printCurrent() {
  int16_t c1 = 0, c2 = 0;
  if (motors.ReadCurrents(RC_ADDR, c1, c2)) {
    Serial.print("M1 current: "); Serial.print(c1 / 100.0f, 2); Serial.println(" A");
  } else {
    Serial.println("ReadCurrents FAILED");
  }
}

void waitArrival(long target) {
  const long tol = 15;                 // counts
  const uint32_t t0 = millis();
  while (millis() - t0 < 6000) {       // 6 s cap
    bool ok; long e = readEnc(&ok);
    if (ok && labs(e - target) <= tol) {
      Serial.print("  arrived @ "); Serial.println(e);
      return;
    }
    delay(50);
  }
  Serial.println("  (arrival timeout -> check limits / PID / current clip)");
}

void doNudge() {
  bool ok;
  long start = readEnc(&ok);
  if (!ok) { Serial.println("Abort move: encoder read invalid."); return; }
  if (start < SOFT_MIN || start > SOFT_MAX) {
    Serial.print("Abort move: start "); Serial.print(start);
    Serial.println(" outside soft window -> position it mid-stroke first.");
    return;
  }

  // Prefer moving positive; flip if that would exceed the soft window.
  long target = (start + NUDGE_COUNTS <= SOFT_MAX) ? (start + NUDGE_COUNTS)
                                                   : (start - NUDGE_COUNTS);
  target = constrain(target, SOFT_MIN, SOFT_MAX);

  Serial.print("Move: "); Serial.print(start); Serial.print(" -> ");
  Serial.print(target);   Serial.print(" -> "); Serial.println(start);

  // Out  (accel, speed, deccel, position, flag=1 execute-now)
  motors.SpeedAccelDeccelPositionM1(RC_ADDR, MV_ACCEL, MV_SPEED, MV_ACCEL, (uint32_t)target, 1);
  waitArrival(target);
  printCurrent();
  delay(300);

  // Back
  motors.SpeedAccelDeccelPositionM1(RC_ADDR, MV_ACCEL, MV_SPEED, MV_ACCEL, (uint32_t)start, 1);
  waitArrival(start);
  Serial.print("Done. Final "); printEnc();
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);            // USB console
  delay(400);

  // EXPLICIT Serial2 pins -- do NOT rely on the core default (3.x = GPIO4/25).
  Serial2.begin(RC_BAUD, SERIAL_8N1, RC_RX_PIN, RC_TX_PIN);
  // motors.begin(RC_BAUD) intentionally omitted (see header note).
  delay(50);

  Serial.println("\n=== HARP V-axis 1A: RoboClaw standalone (no micro-ROS) ===");
  Serial.print("UART2 RX="); Serial.print(RC_RX_PIN);
  Serial.print(" TX=");      Serial.print(RC_TX_PIN);
  Serial.print(" @ ");       Serial.print(RC_BAUD);
  Serial.print(" addr 0x");  Serial.println(RC_ADDR, HEX);

  printVersion();
  printEnc();
  Serial.println("Commands: v=version  e=encoder  c=current  m=nudge(3mm out+back)");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'v': printVersion(); break;
      case 'e': printEnc();     break;
      case 'c': printCurrent(); break;
      case 'm': doNudge();      break;
      case '\n': case '\r':     break;
      default:  Serial.println("keys: v e c m"); break;
    }
  }
}