// HARP / zstair — scissor module node: H-axis servo + V-axis RoboClaw (1B) + V telemetry (1C) + V fail-safe (1D)
// ESP32 WROOM-32 (esp32dev) · micro-ROS over WiFi (UDP) · Engineering Manual v13 §8.4
//
// Subscribes: /h_axis/cmd_us   std_msgs/msg/Int32              servo pulse width (us)
//             /v_axis/cmd_mm   std_msgs/msg/Float32            screw travel (mm)   [1B]
//             /v_axis/reset    std_msgs/msg/Bool               re-arm the V fail-safe latch (true) [1D]
// Publishes:  /v_axis/state    std_msgs/msg/Float32MultiArray  [pos_mm, current_A, fs_state] [1C/1D]
// Drives:     GPIO18 (LEDC ch 0, 50 Hz, 16-bit) -> Stingray-2 servo signal, 3.3 V
//             UART2 GPIO17->S1 / GPIO16<-S2 -> RoboClaw 2x15A (addr 0x80), 115200
// Indicator:  GPIO2 onboard LED toggles on every received H command
//             GPIO4 solid ON when the V axis is tripped (distinct from the H indicator) [1D]
//
// V-axis 1B scope: /v_axis/cmd_mm carries SCREW-mm. Layer A only (mm->counts,
//   796.1). No kinematic map (Layer B, deferred). Sign: +mm -> +counts ->
//   flanges INWARD (1A: toward stored/closed).
//
// V-axis 1C scope: adds /v_axis/state telemetry at 20 Hz. pos_mm = ReadEncM1 /
//   796.1; current_A = ReadCurrents M1 / 100 (RoboClaw returns 10 mA units).
//   Reads are loop-driven and sequential AFTER the write (§4.4/§6): the RoboClaw
//   services one Serial2 transaction at a time, so write and reads never overlap.
//   Bench budget (measured): worst-case write+enc+cur = 5198 us, FITS 100 Hz.
//   Publish only when BOTH reads are valid; a bad frame bumps a diag counter.
//   Publisher adds NO executor handle.
//
// V-axis 1D scope: fail-safe layer. A command-liveness watchdog (V_WATCHDOG_MS)
//   trips the V axis on loss of the /v_axis/cmd_mm stream; a consecutive-write-
//   fault counter (V_FAULT_TRIP) trips on a persistent serial fault. On trip:
//   DutyM1(0) RELEASES M1 and the self-locking screw holds — the motor does NOT
//   fight to hold position. The trip LATCHES (GPIO4 solid, /v_axis/state[2] =
//   1 watchdog / 2 fault) and re-arms ONLY on an explicit /v_axis/reset (Bool
//   true) with the stream live. Telemetry grew to 3 fields. Executor 2 -> 3
//   (adds the reset subscriber). NOTE: the ESP32<->RoboClaw link-loss domain is
//   NOT covered here (a down link cannot transmit DutyM1(0)) — that belongs to
//   the RoboClaw's own serial timeout, still TBD. Reset uses Bool, not Empty:
//   a zero-field Empty produced a null DDS type hash under CycloneDDS and stalled
//   discovery (3-5 s re-arm lag); Bool discovers cleanly.
//
// Bench proof order (H):
//   1. Scope GPIO18 with NO servo connected. Verify 1500 us +/- 10 us at 50 Hz.
//   2. Then BEC 8.4 V + >=1000 uF bulk cap at servo input + BEC GND -> ESP32 GND jumper.
//   3. Then servo, unloaded, off the rack.
// Bench proof order (V, 1B): agent up -> publish 3.0 mm -> flanges step inward
//   2388 counts (identical to the 1A console move) and hold. Keep commands small
//   until GPIO34/35 limit-switch homing exists; V_MAX is a code net, not a limit.
// Bench proof order (V, 1C): agent up -> `ros2 topic echo /v_axis/state`.
//   At rest (home 4777 counts) pos_mm should read ~6.00 mm (4777/796.1) and
//   current_A ~0.00-0.01 A (validates the /100 scale vs 1A idle). Then publish a
//   /v_axis/cmd_mm setpoint and watch pos_mm track+settle while current ticks up.
// Bench proof order (V, 1D): stream cmd_mm live FIRST, then arm. Test A: kill the
//   stream -> state[2] -> 1 within ~500 ms, GPIO4 solid, pos static, current idle.
//   Test B: drop RoboClaw TX with stream live -> v_fault_count -> 5 -> state[2] = 2.
//   Test C: soak -> state[2] stays 0. Re-arm each time via /v_axis/reset {data: true}
//   WITH the stream live (reset stamps liveness; no stream -> re-trips in 500 ms).

#include <Arduino.h>
#include <math.h>                       // lroundf (V-axis)
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>               // V-axis cmd
#include <std_msgs/msg/float32_multi_array.h>   // V-axis state (1C)
#include <std_msgs/msg/bool.h>                  // V-axis fail-safe re-arm (1D)
#include <Basicmicro.h>                 // RoboClaw packet serial (V-axis)
#include <secrets.h>

// ---------------------------------------------------------------- config

#define NODE_NAME       "harp_scissor"
#define TOPIC_NAME      "h_axis/cmd_us"

#define LED_PIN         2
#define H_SERVO_PIN     18
#define H_PWM_FREQ      50        // Hz  -> 20000 us period
#define H_PWM_BITS      16        // 65536 counts / period -> 0.305 us per count
#define H_PWM_CHAN      0
#define H_PWM_PERIOD_US 20000UL

#define H_US_MIN        1020    // Stingray-2 full travel, one end
#define H_US_MAX        2000     // Stingray-2 full travel, other end
#define H_US_CENTER     1500      // power-on / safe default

// --- V-axis (RoboClaw) config -------------------------------------------
#define V_TOPIC_NAME    "v_axis/cmd_mm"   // resolves to /v_axis/cmd_mm
#define V_STATE_TOPIC   "v_axis/state"    // resolves to /v_axis/state   (1C)
#define V_RESET_TOPIC   "v_axis/reset"    // resolves to /v_axis/reset    (1D)
#define RC_ADDRESS      0x80
#define RC_TIMEOUT_US   3000              // 3 ms: one lost-ACK retry ~4.8 ms; MAXRETRY=3
                                          // storm (~19 ms) is a fail-safe event, not cadence
#define V_UART_RX_PIN   16                // RX2 <- RoboClaw S2
#define V_UART_TX_PIN   17                // TX2 -> RoboClaw S1

// -- V-axis safety (1D) --------------------------------------------------
#define V_WATCHDOG_MS   500
#define V_FAILSAFE_LED  4        // free output pin (NOT 2=H; NOT 34/35 input-only)

enum VState : uint8_t { V_ARMED = 0, V_TRIP_WD = 1, V_TRIP_FAULT = 2 };
static volatile uint32_t v_last_cmd_ms = 0;
static VState            v_fs_state     = V_ARMED;   // named to avoid v_state_* collision

// Layer A: screw-mm <-> counts (datasheet-grounded 796.1, carried from 1A)
static const float   V_COUNTS_PER_MM = 796.1f;
static const int32_t V_MIN_COUNTS    = 0;       // = storage / homed encoder reference
static const int32_t V_MAX_COUNTS    = 22000;   // measured, just short of inner dead zone

// 1A-proven gentle move params (speed 1000 cts/s ~= 1.26 mm/s screw)
static const uint32_t V_ACCEL  = 2000;
static const uint32_t V_SPEED  = 1000;
static const uint32_t V_DECCEL = 2000;
static const uint8_t  V_FLAG   = 1;             // execute now

static const uint8_t  V_FAULT_TRIP = 5;         // consecutive fails -> 1D fail-safe

// --- V-axis telemetry (1C) ----------------------------------------------
static const uint32_t V_TELEM_PERIOD_MS = 50;   // 20 Hz publish cadence

// ---------------------------------------------------------------- state

rclc_support_t          support;
rcl_allocator_t         allocator;
rcl_node_t              node;
rcl_subscription_t      h_cmd_sub;
rclc_executor_t         executor;
std_msgs__msg__Int32    h_cmd_msg;

static int32_t h_cmd_us = H_US_CENTER;

// --- V-axis state --------------------------------------------------------
Basicmicro              motors(&Serial2, RC_TIMEOUT_US);   // begin() NOT used (1A 4.1)
rcl_subscription_t      v_cmd_sub;
std_msgs__msg__Float32  v_cmd_msg;

volatile float  v_target_mm   = 0.0f;   // written in callback, read in loop
volatile bool   v_new_cmd     = false;
static uint8_t  v_fault_count = 0;

// --- V-axis telemetry publisher (1C) ------------------------------------
rcl_publisher_t                  v_state_pub;
std_msgs__msg__Float32MultiArray v_state_msg;         // zero-init at file scope
static float    v_state_buf[3];                       // [0]=pos_mm, [1]=current_A, [2]=fs_state
static uint32_t v_telem_last_ms    = 0;
static uint32_t v_telem_fault_count = 0;              // read valid/ok failures (diag)

// --- V-axis fail-safe re-arm subscriber (1D) ----------------------------
rcl_subscription_t     v_reset_sub;
std_msgs__msg__Bool    v_reset_msg;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFT(fn)  { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  // Fail visibly and loudly rather than silently running a half-initialized node.
  // The servo is unaffected: LEDC is hardware and holds its last duty. The V axis
  // is safe too: no command is sent before init completes, and the self-locking
  // screw holds whatever position it is in.
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ---------------------------------------------------------------- LEDC
// Arduino-ESP32 core 3.x merged ledcSetup + ledcAttachPin into ledcAttach*,
// and ledcWrite now takes the GPIO rather than the channel. Compile against
// whichever core micro_ros_platformio resolved, without editing this file.

static inline void h_pwm_init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(H_SERVO_PIN, H_PWM_FREQ, H_PWM_BITS, H_PWM_CHAN);
#else
  ledcSetup(H_PWM_CHAN, H_PWM_FREQ, H_PWM_BITS);
  ledcAttachPin(H_SERVO_PIN, H_PWM_CHAN);
#endif
}

static inline void h_pwm_write_us(uint32_t us) {
  // duty = us * 65536 / 20000 = us * 3.2768
  //   500 us -> 1638 counts | 1500 us -> 4915 | 2500 us -> 8192
  uint32_t duty = (us * (1UL << H_PWM_BITS)) / H_PWM_PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(H_SERVO_PIN, duty);
#else
  ledcWrite(H_PWM_CHAN, duty);
#endif
}

static inline int32_t clamp_us(int32_t us) {
  if (us < H_US_MIN) return H_US_MIN;
  if (us > H_US_MAX) return H_US_MAX;
  return us;
}

// ---------------------------------------------------------------- callbacks

void h_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;

  h_cmd_us = clamp_us(m->data);
  h_pwm_write_us((uint32_t)h_cmd_us);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// V: keep the callback trivial — latch and return. The blocking RoboClaw write
// must NOT run here, or a slow ACK stalls the executor spin (starves the agent).
void v_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
    v_target_mm   = m->data;      // latch target
    v_new_cmd     = true;         // latch "new"
    v_last_cmd_ms = millis();     // liveness stamp — every arrival (1D watchdog feed)
}

// V: runs in loop(), OUTSIDE the executor spin. Convert -> clamp -> send.
static void service_v_axis() {
  if (!v_new_cmd) return;
  v_new_cmd = false;

  const float mm = v_target_mm;
  int32_t counts = (int32_t)lroundf(mm * V_COUNTS_PER_MM);  // +mm -> +counts -> INWARD
  if (counts < V_MIN_COUNTS) counts = V_MIN_COUNTS;
  if (counts > V_MAX_COUNTS) counts = V_MAX_COUNTS;

  // Absolute position + execute-now => a retry re-sends the same target (idempotent).
  const bool ok = motors.SpeedAccelDeccelPositionM1(
      RC_ADDRESS, V_ACCEL, V_SPEED, V_DECCEL, (uint32_t)counts, V_FLAG);

  if (ok) {
    v_fault_count = 0;
  } else {
    if (v_fault_count < 255) v_fault_count++;
  }
}

// V (1D): re-arm the fail-safe latch. Bool, gated on true so a stray false is a
// no-op. Callback stays trivial (flags only) — same rule as the command path.
// Re-arm does NOT auto-resume a stale move: after DutyM1(0) the next streamed
// setpoint re-establishes position mode. Must be published WITH the stream live,
// or the watchdog re-trips within V_WATCHDOG_MS.
void v_reset_callback(const void *msgin) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    if (!m->data) return;          // re-arm on true only

    v_fs_state    = V_ARMED;        // clear the latch
    v_fault_count = 0;              // clear the fault accumulator
    v_last_cmd_ms = millis();       // grace: don't immediately re-trip
    digitalWrite(V_FAILSAFE_LED, LOW);
}

// V telemetry (1C): runs in loop(), OUTSIDE the executor spin, and AFTER
// service_v_axis() so the two RoboClaw reads never overlap the write on Serial2.
// Blocking reads live here, never in a data callback — same rule as the write.
static void service_telemetry() {
  const uint32_t now = millis();
  if (now - v_telem_last_ms < V_TELEM_PERIOD_MS) return;   // 20 Hz gate
  v_telem_last_ms = now;

  uint8_t  enc_status = 0;
  bool     enc_valid  = false;
  uint32_t raw = motors.ReadEncM1(RC_ADDRESS, &enc_status, &enc_valid);

  int16_t i_m1 = 0, i_m2 = 0;
  const bool cur_ok = motors.ReadCurrents(RC_ADDRESS, i_m1, i_m2);

  if (!enc_valid || !cur_ok) {            // §6: check valid before publishing
    if (v_telem_fault_count < 0xFFFFFFFF) v_telem_fault_count++;
    return;                               // skip this frame, retry next cadence
  }

  v_state_buf[0] = (int32_t)raw / V_COUNTS_PER_MM;   // screw mm (cast before divide)
  v_state_buf[1] = i_m1 / 100.0f;                    // A (RoboClaw = 10 mA units)
  v_state_buf[2] = (float)v_fs_state;                // 0 armed, 1 wd-trip, 2 fault-trip
  RCSOFT(rcl_publish(&v_state_pub, &v_state_msg, NULL));   // non-fatal on transient fail
}

// ---------------------------------------------------------------- fail-safe (1D)

void v_trip(VState reason) {
    if (v_fs_state != V_ARMED) return;       // latch: first trip wins
    v_fs_state = reason;
    motors.DutyM1(RC_ADDRESS, 0);            // release M1 -> self-locking screw holds
    digitalWrite(V_FAILSAFE_LED, HIGH);
}

void v_check_failsafe() {
    if (v_fs_state != V_ARMED) return;
    if ((uint32_t)(millis() - v_last_cmd_ms) > V_WATCHDOG_MS) v_trip(V_TRIP_WD);
    else if (v_fault_count >= V_FAULT_TRIP)                   v_trip(V_TRIP_FAULT);
}

// ---------------------------------------------------------------- setup

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Fail-safe indicator + liveness baseline (1D). Boots ARMED with a fresh
  // stamp: if the cmd_mm stream is not up within V_WATCHDOG_MS it trips safe.
  pinMode(V_FAILSAFE_LED, OUTPUT);
  digitalWrite(V_FAILSAFE_LED, LOW);
  v_last_cmd_ms = millis();

  // PWM comes up BEFORE the network. If WiFi or the agent never arrives, the
  // servo still sees a valid 1500 us pulse train rather than a floating pin.
  h_pwm_init();
  h_pwm_write_us(H_US_CENTER);

  // RoboClaw UART2. Explicit pins (1A 4.1): the lib does NOT set them and the
  // core default is version-dependent. motors.begin() intentionally omitted.
  // Bringing the link up before the network is harmless: no command is sent
  // until a /v_axis/cmd_mm message arrives.
  Serial2.begin(115200, SERIAL_8N1, V_UART_RX_PIN, V_UART_TX_PIN);

  IPAddress agent_ip(AGENT_IP_0, AGENT_IP_1, AGENT_IP_2, AGENT_IP_3);
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // --- subscriptions (init all before the executor adds) ---
  RCCHECK(rclc_subscription_init_default(
      &h_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
      &v_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      V_TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
      &v_reset_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      V_RESET_TOPIC));

  // V telemetry publisher (1C). Needs node + support only; a publisher has no
  // callback, so it is NOT an executor handle (executor sizing below is unaffected).
  RCCHECK(rclc_publisher_init_default(
      &v_state_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      V_STATE_TOPIC));

  // Pre-allocate the Float32MultiArray data sequence. micro-ROS will NOT
  // allocate this: point .data at a static buffer and set size == capacity,
  // or rcl_publish corrupts memory. layout is unused -> leave it empty.
  v_state_msg.data.data           = v_state_buf;
  v_state_msg.data.size           = 3;
  v_state_msg.data.capacity       = 3;
  v_state_msg.layout.dim.data     = NULL;
  v_state_msg.layout.dim.size     = 0;
  v_state_msg.layout.dim.capacity = 0;
  v_state_msg.layout.data_offset  = 0;

  // 3 handles: H command + V command + V reset subscribers. The telemetry
  // publisher is NOT a handle. This count MUST match the number of adds below,
  // or the last-added callback (v_reset) is silently never serviced (1B 4.5).
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &h_cmd_sub,
      &h_cmd_msg,
      &h_cmd_callback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &v_cmd_sub,
      &v_cmd_msg,
      &v_cmd_callback,
      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &v_reset_sub,
      &v_reset_msg,
      &v_reset_callback,
      ON_NEW_DATA));
}

// ---------------------------------------------------------------- loop
void loop() {
  // Fires h_cmd_callback / v_cmd_callback / v_reset_callback on new data.
  // v_cmd_callback only latches; the blocking RoboClaw write happens below,
  // off the executor.
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Evaluate fail-safe before the write, so a trip suppresses this cycle's command.
  v_check_failsafe();

  // Apply any latched V command outside the spin -- only while armed.
  if (v_fs_state == V_ARMED) service_v_axis();

  // Then publish V telemetry (20 Hz gate). Sequential after the write so the
  // two reads never overlap it on Serial2.
  service_telemetry();

  // delay(5) keeps this loop well below 100 Hz for bring-up (start slow).
  delay(5);
}
