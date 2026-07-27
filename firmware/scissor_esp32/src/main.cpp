// HARP / zstair — scissor module H-axis servo node
// ESP32 WROOM-32 (esp32dev) · micro-ROS over WiFi (UDP) · Engineering Manual v13 §8.4
//
// Subscribes: /h_axis/cmd_us   std_msgs/msg/Int32   servo pulse width in microseconds
// Drives:     GPIO18 (LEDC ch 0, 50 Hz, 16-bit) -> Stingray-2 servo signal, 3.3 V
// Indicator:  GPIO2 onboard LED toggles on every received command
//
// Bench proof order:
//   1. Scope GPIO18 with NO servo connected. Verify 1500 us +/- 10 us at 50 Hz.
//   2. Then BEC 8.4 V + >=1000 uF bulk cap at servo input + BEC GND -> ESP32 GND jumper.
//   3. Then servo, unloaded, off the rack.

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
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

// ---------------------------------------------------------------- state

rclc_support_t          support;
rcl_allocator_t         allocator;
rcl_node_t              node;
rcl_subscription_t      h_cmd_sub;
rclc_executor_t         executor;
std_msgs__msg__Int32    h_cmd_msg;

static int32_t h_cmd_us = H_US_CENTER;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFT(fn)  { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  // Fail visibly and loudly rather than silently running a half-initialized node.
  // The servo is unaffected: LEDC is hardware and holds its last duty.
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

// ---------------------------------------------------------------- callback

void h_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;

  h_cmd_us = clamp_us(m->data);
  h_pwm_write_us((uint32_t)h_cmd_us);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ---------------------------------------------------------------- setup

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
 

  // PWM comes up BEFORE the network. If WiFi or the agent never arrives, the
  // servo still sees a valid 1500 us pulse train rather than a floating pin.
  h_pwm_init();
  h_pwm_write_us(H_US_CENTER);

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

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &h_cmd_sub,
      &h_cmd_msg,
      &h_cmd_callback,
      ON_NEW_DATA));
}

// ---------------------------------------------------------------- loop

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(5);
}