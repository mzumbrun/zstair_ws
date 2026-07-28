// HARP / zstair — scissor module node: H-axis servo + V-axis RoboClaw (1B)
// ESP32 WROOM-32 (esp32dev) · micro-ROS over WiFi (UDP) · Engineering Manual v13 §8.4
//
// Subscribes: /h_axis/cmd_us   std_msgs/msg/Int32     servo pulse width (us)
//             /v_axis/cmd_mm   std_msgs/msg/Float32   screw travel (mm)   [1B]
// Drives:     GPIO18 (LEDC ch 0, 50 Hz, 16-bit) -> Stingray-2 servo signal, 3.3 V
//             UART2 GPIO17->S1 / GPIO16<-S2 -> RoboClaw 2x15A (addr 0x80), 115200
// Indicator:  GPIO2 onboard LED toggles on every received H command
//
// V-axis 1B scope: /v_axis/cmd_mm carries SCREW-mm. Layer A only (mm->counts,
//   796.1). No kinematic map (Layer B, deferred). No telemetry reads (1C).
//   Sign: +mm -> +counts -> flanges INWARD (1A: toward stored/closed).
//
// Bench proof order (H):
//   1. Scope GPIO18 with NO servo connected. Verify 1500 us +/- 10 us at 50 Hz.
//   2. Then BEC 8.4 V + >=1000 uF bulk cap at servo input + BEC GND -> ESP32 GND jumper.
//   3. Then servo, unloaded, off the rack.
// Bench proof order (V, 1B): agent up -> publish 3.0 mm -> flanges step inward
//   2388 counts (identical to the 1A console move) and hold. Keep commands small
//   until GPIO34/35 limit-switch homing exists; V_MAX is a code net, not a limit.

#include <Arduino.h>
#include <math.h>                       // lroundf (V-axis)
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>       // V-axis cmd
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
#define RC_ADDRESS      0x80
#define RC_TIMEOUT_US   3000              // 3 ms: one lost-ACK retry ~4.8 ms; MAXRETRY=3
                                          // storm (~19 ms) is a fail-safe event, not cadence
#define V_UART_RX_PIN   16                // RX2 <- RoboClaw S2
#define V_UART_TX_PIN   17                // TX2 -> RoboClaw S1

// Layer A: screw-mm <-> counts (datasheet-grounded 796.1, carried from 1A)
static const float   V_COUNTS_PER_MM = 796.1f;
static const int32_t V_MIN_COUNTS    = 0;       // = power-up encoder reference
static const int32_t V_MAX_COUNTS    = 23600;   // full-stroke CODE clamp only

// 1A-proven gentle move params (speed 1000 cts/s ~= 1.26 mm/s screw)
static const uint32_t V_ACCEL  = 2000;
static const uint32_t V_SPEED  = 1000;
static const uint32_t V_DECCEL = 2000;
static const uint8_t  V_FLAG   = 1;             // execute now

static const uint8_t  V_FAULT_TRIP = 5;         // consecutive fails -> 1D fail-safe

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
void v_cmd_callback(const void * msgin) {
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
  v_target_mm = m->data;
  v_new_cmd   = true;
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
    // TODO 1D: if (v_fault_count >= V_FAULT_TRIP) v_axis_failsafe();
    //          -> stop M1; self-locking screw holds. Do NOT fight position.
  }
}

// ---------------------------------------------------------------- setup

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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

  // 2 handles: H subscriber + V subscriber. Must match the count added below,
  // or the last-added callback is silently never serviced.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
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
}

// ---------------------------------------------------------------- loop

void loop() {
  // Fires h_cmd_callback / v_cmd_callback on new data. v_cmd_callback only
  // latches; the blocking RoboClaw write happens below, off the executor.
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Apply any latched V command outside the spin.
  service_v_axis();

  // delay(5) keeps this loop well below 100 Hz for bring-up (start slow).
  delay(5);
}
