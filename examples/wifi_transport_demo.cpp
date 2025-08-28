#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// ---------- Wi-Fi + Agent ----------
char ssid[] = "WIFI";                // << replace with your WiFi SSID
char psk[]  = "100200300";           // << replace with your WiFi password
IPAddress agent_ip(192,168,1,173);   // << replace with your PC/agent IP
uint16_t agent_port = 8888;          // << agent port

// ---------- LED pin ----------
#define LED_PIN 4   // external LED connected to GPIO4

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

// ---------- helpers ----------
static bool connectWiFi(unsigned long timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, psk);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) return false;
    delay(250);
  }
  return true;
}

static void waitForAgent() {
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) {
    delay(500);
  }
}

// ---------- callback ----------
static void led_cmd_callback(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, m->data ? HIGH : LOW);

  // publish current state back
  led_state_msg.data = m->data;
  (void) rcl_publish(&state_pub, &led_state_msg, NULL);

  Serial.printf("[ESP32] LED set to: %s\n", m->data ? "ON" : "OFF");
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Wi-Fi
  if (!connectWiFi()) {
    Serial.println("[WiFi] Failed to connect, retrying forever...");
    while (!connectWiFi()) delay(1000);
  }
  Serial.print("[WiFi] OK. IP: "); Serial.println(WiFi.localIP());

  // micro-ROS transport + agent
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  Serial.print("[micro-ROS] Waiting for agent "); Serial.println(agent_ip);
  waitForAgent();
  Serial.println("[micro-ROS] Agent found");

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

  // publish initial state (OFF)
  led_state_msg.data = false;
  (void) rcl_publish(&state_pub, &led_state_msg, NULL);

  Serial.println("[Setup] Ready: subscribe /led_cmd, publish /led_state");
}

// ---------- loop ----------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // lightweight Wi-Fi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(ssid, psk);
  }

  delay(10);
}
