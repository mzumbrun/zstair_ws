#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <secrets.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

  IPAddress agent_ip(AGENT_IP_0, AGENT_IP_1, AGENT_IP_2, AGENT_IP_3);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agent_ip, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "scissor_node", "", &support);

  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "scissor_counter");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
} 
