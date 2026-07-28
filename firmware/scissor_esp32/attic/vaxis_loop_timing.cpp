/*
 * HARP V-Axis 1C — Serial-Load / Loop-Timing Measurement Stub
 * ----------------------------------------------------------------------------
 * PURPOSE (feasibility gate BEFORE the /v_axis/state publisher exists):
 *   Measure the real, on-hardware duration of the three RoboClaw transactions
 *   that step 1C will add to the control loop:
 *       1) SpeedAccelDeccelPositionM1   (write, 21 TX + 1 ACK  = 22 B)
 *       2) ReadEncM1                    (read,   2 TX + 7 RX   =  9 B)
 *       3) ReadCurrents                 (read,   2 TX + 6 RX   =  8 B)
 *   ...and confirm write+2reads fits a control cycle at 100 Hz / 50 Hz.
 *
 * WHY STANDALONE:
 *   The measured call duration = serial wire time + RoboClaw command-to-response
 *   turnaround. Both are independent of WiFi / micro-ROS. Isolating this from the
 *   full node keeps agent/WiFi jitter out of the numbers. Combine the result with
 *   the known micro-ROS overhead afterward.
 *
 * SAFETY:
 *   Captures the current encoder count at startup and commands THAT SAME target
 *   every cycle (absolute-position write). The 21-byte write + ACK executes in
 *   full (true timing) but the mechanism does not move. Set MOVE_DELTA_COUNTS
 *   nonzero ONLY on a hand-verified span if you want to confirm motion happens.
 *
 * CONFIG MATCHES 1B AS-RUN (do not drift):
 *   Serial2 115200 8N1 on RX=16 / TX=17, motors.begin() omitted (manual init),
 *   constructor timeout 3000 us, accel/deccel 2000, speed 1000 cts/s, flag 1.
 *
 * OUTPUT: USB Serial (115200). No ROS, no publisher. Read the block report.
 * ============================================================================
 */

#include <Arduino.h>
#include <Basicmicro.h>

// ---- Link / device config (must match 1B as-run) ---------------------------
static const uint32_t ROBOCLAW_TIMEOUT_US = 3000;      // 1B standing value
static const uint8_t  ROBOCLAW_ADDR       = 0x80;      // CONFIRM in Motion Studio
static const int      S2_RX_PIN           = 16;
static const int      S2_TX_PIN           = 17;
static const uint32_t S2_BAUD             = 115200;

// ---- Move command params (match 1B) ----------------------------------------
static const uint32_t MOVE_ACCEL   = 2000;
static const uint32_t MOVE_SPEED   = 1000;   // cts/s (~1.26 mm/s)
static const uint32_t MOVE_DECCEL  = 2000;
static const uint8_t  MOVE_FLAG    = 1;      // execute immediately (no buffer)

// Set to 0 for zero-motion timing (SAFE default). Nonzero alternates +/- this
// many counts around the captured home ONLY within a hand-verified span.
static const int32_t  MOVE_DELTA_COUNTS = 0;

// ---- Measurement config ----------------------------------------------------
static const uint32_t WARMUP_CYCLES = 20;    // discard first-after-idle anomalies
static const uint32_t BLOCK_CYCLES  = 500;   // cycles per printed report block

// ---- Wire-time floors at 115200 8N1 (86.81 us/byte), for derived turnaround -
static const float US_PER_BYTE     = 10.0f / 115200.0f * 1e6f;  // 86.81 us
static const float WIRE_WRITE_US   = 22.0f * US_PER_BYTE;       // 1910 us
static const float WIRE_ENC_US     =  9.0f * US_PER_BYTE;       //  781 us
static const float WIRE_CUR_US     =  8.0f * US_PER_BYTE;       //  694 us

// ---- Budget targets --------------------------------------------------------
static const uint32_t PERIOD_100HZ_US = 10000;
static const uint32_t PERIOD_50HZ_US  = 20000;

Basicmicro motors(&Serial2, ROBOCLAW_TIMEOUT_US);

// Running stats for one timed call
struct Stat {
  uint32_t n    = 0;
  uint32_t vmin = 0xFFFFFFFF;
  uint32_t vmax = 0;
  uint64_t vsum = 0;
  uint32_t fail = 0;   // valid==false / bool==false
  void add(uint32_t dt, bool ok) {
    n++;
    if (dt < vmin) vmin = dt;
    if (dt > vmax) vmax = dt;
    vsum += dt;
    if (!ok) fail++;
  }
  float mean() const { return n ? (float)vsum / (float)n : 0.0f; }
  void reset() { *this = Stat(); }
};

Stat sWrite, sEnc, sCur, sCycle;
int32_t home_counts = 0;
bool    home_ok     = false;
uint32_t moveToggle = 0;

// One-time home capture with retry
static bool captureHome() {
  for (int i = 0; i < 10; i++) {
    uint8_t status = 0; bool valid = false;
    uint32_t c = motors.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
    if (valid) { home_counts = (int32_t)c; return true; }
    delay(20);
  }
  return false;
}

static int32_t currentTarget() {
  if (MOVE_DELTA_COUNTS == 0) return home_counts;      // zero-motion (safe)
  return home_counts + ((moveToggle++ & 1) ? MOVE_DELTA_COUNTS : -MOVE_DELTA_COUNTS);
}

// Run one full transaction cycle, timing each call individually.
static void runCycle(bool record) {
  uint32_t t0, dt;

  // --- write ---
  int32_t tgt = currentTarget();
  t0 = micros();
  bool wok = motors.SpeedAccelDeccelPositionM1(
               ROBOCLAW_ADDR, MOVE_ACCEL, MOVE_SPEED, MOVE_DECCEL,
               (uint32_t)tgt, MOVE_FLAG);
  dt = micros() - t0;
  uint32_t dtWrite = dt;

  // --- ReadEncM1 ---
  uint8_t status = 0; bool encValid = false;
  t0 = micros();
  (void)motors.ReadEncM1(ROBOCLAW_ADDR, &status, &encValid);
  dt = micros() - t0;
  uint32_t dtEnc = dt;

  // --- ReadCurrents ---
  int16_t m1 = 0, m2 = 0;
  t0 = micros();
  bool curOk = motors.ReadCurrents(ROBOCLAW_ADDR, m1, m2);
  dt = micros() - t0;
  uint32_t dtCur = dt;

  if (record) {
    sWrite.add(dtWrite, wok);
    sEnc.add(dtEnc,   encValid);
    sCur.add(dtCur,   curOk);
    sCycle.add(dtWrite + dtEnc + dtCur, wok && encValid && curOk);
  }
}

static void printLine(const char* name, float wireFloor, const Stat& s) {
  float turn = s.mean() - wireFloor;   // derived RoboClaw turnaround (mean)
  Serial.printf("  %-16s %7u %8.1f %7u   %7.0f      %+7.1f     %u/%u\n",
                name, s.vmin, s.mean(), s.vmax, wireFloor, turn, s.fail, s.n);
}

static void printReport() {
  Serial.println("\n===== V-Axis 1C serial-load report =====");
  Serial.printf("115200 8N1 | %.2f us/byte | wire floor: write %.0f  enc %.0f  cur %.0f (us)\n",
                US_PER_BYTE, WIRE_WRITE_US, WIRE_ENC_US, WIRE_CUR_US);
  Serial.printf("home_counts = %ld   move_delta = %ld (%s)\n",
                (long)home_counts, (long)MOVE_DELTA_COUNTS,
                MOVE_DELTA_COUNTS == 0 ? "zero-motion" : "MOTION - verify span!");
  Serial.println("  call             min(us) mean(us) max(us)  wireFloor  meanTurn(us)  fails");
  printLine("write 22B",       WIRE_WRITE_US, sWrite);
  printLine("ReadEncM1 9B",    WIRE_ENC_US,   sEnc);
  printLine("ReadCurrents 8B", WIRE_CUR_US,   sCur);

  Serial.println("  ---- full cycle (write + enc + cur) ----");
  Serial.printf("  cycle sum        %7u %8.1f %7u\n",
                sCycle.vmin, sCycle.mean(), sCycle.vmax);

  // Budget verdicts against WORST-case cycle (the honest number)
  float hr100 = (float)PERIOD_100HZ_US - (float)sCycle.vmax;
  float hr50  = (float)PERIOD_50HZ_US  - (float)sCycle.vmax;
  Serial.printf("  100Hz (10000us): worst-case headroom = %+.0f us  -> %s\n",
                hr100, hr100 > 0 ? "FITS" : "OVER");
  Serial.printf("   50Hz (20000us): worst-case headroom = %+.0f us  -> %s\n",
                hr50,  hr50  > 0 ? "FITS" : "OVER");
  Serial.printf("  (leave headroom for micro-ROS spin + publish, ~0.3-1 ms)\n");
  Serial.println("========================================");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\nHARP V-Axis 1C serial-load stub — no publisher, standalone.");

  // Manual Serial2 init, motors.begin() omitted (1B as-run)
  Serial2.begin(S2_BAUD, SERIAL_8N1, S2_RX_PIN, S2_TX_PIN);
  delay(200);

  if (!captureHome()) {
    Serial.println("FATAL: ReadEncM1 never returned valid. Check wiring/addr/baud. Halting.");
    while (true) delay(1000);
  }
  Serial.printf("Home captured: %ld counts (%.3f mm @ 796.1). Warming up...\n",
                (long)home_counts, home_counts / 796.1f);

  for (uint32_t i = 0; i < WARMUP_CYCLES; i++) runCycle(false);
}

void loop() {
  sWrite.reset(); sEnc.reset(); sCur.reset(); sCycle.reset();
  for (uint32_t i = 0; i < BLOCK_CYCLES; i++) runCycle(true);
  printReport();
  delay(1000);   // pause between report blocks so you can read them
}