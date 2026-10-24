// HARP wheel scales — raw HX711 streamer, fw v1 (2026-10-04)
// ESP32 WROOM-32 DevKitC (esp32dev) · USB serial 115200 · no libraries
//
// Streams one line whenever any present scale has a new conversion:
//     t_ms,raw1,raw2,raw3,raw4
//   raw = signed 24-bit HX711 count, channel A, gain 128. An empty field means
//   no new sample from that scale since the last line (or scale not present).
// Info lines start with '#'. Serial commands: 'i' = reprint info,
//   'p' = re-probe which scales are present (use after plugging one in).
// No tare or calibration here: all of that lives in the host script.
//
// Scale  DT       SCK   (GPIO34-39 are input-only, no internal pull-ups)
//   1    34       25    left drive  (left front)
//   2    35       26    right drive
//   3    36 (VP)  27    left caster
//   4    39 (VN)  32    right caster

#include <Arduino.h>

static const int N = 4;
static const uint8_t DT_PIN[N]  = {34, 35, 36, 39};
static const uint8_t SCK_PIN[N] = {25, 26, 27, 32};
static const char* FW = "harp_scales_fw v1";

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bool present[N];
static int32_t last[N];
static bool fresh[N];

// Shift out one conversion. SCK must stay high < 60 us or the HX711 powers
// down, hence the critical section. 25 pulses total selects channel A,
// gain 128 for the next conversion (B+/B- are never selected).
static int32_t hxRead(int i) {
  uint32_t v = 0;
  portENTER_CRITICAL(&mux);
  for (int b = 0; b < 25; b++) {
    digitalWrite(SCK_PIN[i], HIGH);
    delayMicroseconds(1);
    if (b < 24) v = (v << 1) | (uint32_t)digitalRead(DT_PIN[i]);
    digitalWrite(SCK_PIN[i], LOW);
    delayMicroseconds(1);
  }
  portEXIT_CRITICAL(&mux);
  if (v & 0x800000UL) v |= 0xFF000000UL;  // sign-extend 24 -> 32 bit
  return (int32_t)v;
}

// A real HX711 pulls DT low when a conversion is ready (within ~400 ms of
// power-up at 10 SPS) and drives it high again right after the read.
// A floating DT pin (nothing connected) fails one of these checks.
static bool probe(int i) {
  uint32_t t0 = millis();
  while (digitalRead(DT_PIN[i]) == HIGH) {
    if (millis() - t0 > 1000) return false;
    delay(1);
  }
  int32_t v = hxRead(i);
  delayMicroseconds(20);
  bool highAfter = (digitalRead(DT_PIN[i]) == HIGH);
  return highAfter && v != 0 && v != -1;
}

static void printInfo() {
  Serial.printf("# %s\n", FW);
  Serial.print("# present:");
  for (int i = 0; i < N; i++) {
    Serial.print(present[i] ? 1 : 0);
    if (i < N - 1) Serial.print(',');
  }
  Serial.println();
  Serial.println("# format: t_ms,raw1,raw2,raw3,raw4");
}

static void probeAll() {
  for (int i = 0; i < N; i++) { present[i] = probe(i); fresh[i] = false; }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < N; i++) {
    pinMode(DT_PIN[i], INPUT);
    pinMode(SCK_PIN[i], OUTPUT);
    digitalWrite(SCK_PIN[i], LOW);  // SCK low = HX711 powered up
  }
  delay(500);  // HX711 power-up settle
  probeAll();
  printInfo();
}

void loop() {
  bool any = false;
  for (int i = 0; i < N; i++) {
    if (present[i] && digitalRead(DT_PIN[i]) == LOW) {
      last[i] = hxRead(i);
      fresh[i] = true;
      any = true;
    }
  }
  if (any) {
    char buf[96];
    int n = snprintf(buf, sizeof buf, "%lu", (unsigned long)millis());
    for (int i = 0; i < N; i++) {
      if (fresh[i]) n += snprintf(buf + n, sizeof buf - n, ",%ld", (long)last[i]);
      else          n += snprintf(buf + n, sizeof buf - n, ",");
      fresh[i] = false;
    }
    Serial.println(buf);
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') printInfo();
    else if (c == 'p') { probeAll(); printInfo(); }
  }
}
