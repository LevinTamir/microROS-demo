#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// ---------- LED pin ----------
#define LED_PIN 4   // your external LED on GPIO4

// ---------- micro-ROS handles ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t led_sub;
rcl_publisher_t state_pub;
rclc_executor_t executor;

// ---------- messages ----------
std_msgs__msg__Bool led_cmd_msg;
std_msgs__msg__Bool led_state_msg;

// ---------- callback ----------
static void led_cmd_callback(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, m->data ? HIGH : LOW);

  led_state_msg.data = m->data;
  (void) rcl_publish(&state_pub, &led_state_msg, NULL);

  Serial.printf("[ESP32] LED set to: %s\n", m->data ? "ON" : "OFF");
}

void setup() {
  // USB serial for logging
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- micro-ROS over USB serial ---
  // Use the default USB/UART bridge 'Serial'
  set_microros_serial_transports(Serial);

  // rcl init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_led_node", "", &support);

  // publisher: /led_state
  rclc_publisher_init_default(
      &state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/led_state");

  // subscription: /led_cmd
  rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/led_cmd");

  // executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &led_sub, &led_cmd_msg,
                                 &led_cmd_callback, ON_NEW_DATA);

  // initial state publish (OFF)
  led_state_msg.data = false;
  (void) rcl_publish(&state_pub, &led_state_msg, NULL);

  Serial.println("[Setup] Ready over USB serial: /led_cmd <-> /led_state");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
