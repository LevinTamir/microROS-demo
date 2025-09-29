#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
// --- changed includes ---
#include <rosidl_runtime_c/string_functions.h>
// (you no longer need micro_ros_utilities or primitives_sequence_functions here)

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =================== Wi-Fi + agent ===================
char ssid[] = "WIFI";                // << your WiFi SSID
char psk[]  = "100200300";           // << your WiFi password
IPAddress agent_ip(192,168,1,215);   // << micro-ROS agent IP
uint16_t agent_port = 8888;

// =================== Motor pins ======================
#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 32
#define IN4 33
#define ENB 14

// PWM (LEDC)
static const int PWM_CH_A = 0;
static const int PWM_CH_B = 1;
static const int PWM_FREQ = 3000;      // 3 kHz (quiet on L298N)
static const int PWM_RES  = 8;         // 0..255 duty

// =================== Kinematics ======================
static const float WHEEL_RADIUS_M   = 0.033f; // adjust to your wheels
static const float BASE_WIDTH_M     = 0.135f; // wheel separation (m)
static const float MAX_RPM          = 150.0f; // adjust to your motors
static const float MAX_WHEEL_MS     = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

// Safety: how long to keep last command before stopping
static const uint32_t CMD_TIMEOUT_MS = 500;

// =================== micro-ROS handles ================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t cmd_sub;
rcl_publisher_t joint_pub;

rclc_executor_t executor;
rcl_timer_t joint_timer;

geometry_msgs__msg__Twist cmd_msg;
sensor_msgs__msg__JointState joint_msg;

// ---- Static storage for JointState sequences (fix) ----
static rosidl_runtime_c__String joint_names_buf[2];
static double pos_buf[2];
static double vel_buf[2];
static double eff_buf[2];

// =================== State ============================
volatile float v_left_ms  = 0.0f;  // commanded wheel linear speed (m/s)
volatile float v_right_ms = 0.0f;

double pos_left_rad  = 0.0;        // integrated wheel angle (rad)
double pos_right_rad = 0.0;
uint32_t last_cmd_ms = 0;

// ---------------- Wi-Fi helpers -----------------------
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

// ---------------- Motor driver ------------------------
static inline uint8_t duty_from_speed(float v_ms) {
  float m = fabsf(v_ms) / MAX_WHEEL_MS;
  if (m > 1.0f) m = 1.0f;
  return (uint8_t)lrintf(m * 255.0f);
}

static void set_motor_left(float v_ms) {
  if (v_ms >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(PWM_CH_A, duty_from_speed(v_ms));
}

static void set_motor_right(float v_ms) {
  if (v_ms >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(PWM_CH_B, duty_from_speed(v_ms));
}

static void stop_motors() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

// --------------- ROS callbacks ------------------------
static void cmd_vel_cb(const void * msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

  float v  = (float)m->linear.x;     // m/s
  float wz = (float)m->angular.z;    // rad/s

  // differential kinematics (unicycle → wheels)
  float vL = v - 0.5f * wz * BASE_WIDTH_M;
  float vR = v + 0.5f * wz * BASE_WIDTH_M;

  // clamp to achievable wheel speeds
  if (vL >  MAX_WHEEL_MS) vL =  MAX_WHEEL_MS;
  if (vL < -MAX_WHEEL_MS) vL = -MAX_WHEEL_MS;
  if (vR >  MAX_WHEEL_MS) vR =  MAX_WHEEL_MS;
  if (vR < -MAX_WHEEL_MS) vR = -MAX_WHEEL_MS;

  v_left_ms  = vL;
  v_right_ms = vR;

  set_motor_left(v_left_ms);
  set_motor_right(v_right_ms);

  last_cmd_ms = millis();
}

// Publish JointState at fixed rate, integrate commanded velocity
static void joint_timer_cb(rcl_timer_t * timer, int64_t /*last_call_time*/) {
  (void) timer;

  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_left_ms = 0.0f;
    v_right_ms = 0.0f;
    stop_motors();
  }

  static uint32_t prev_ms = millis();
  uint32_t now_ms = millis();
  float dt = (now_ms - prev_ms) / 1000.0f;
  prev_ms = now_ms;

  double wL = (double)(v_left_ms  / WHEEL_RADIUS_M);
  double wR = (double)(v_right_ms / WHEEL_RADIUS_M);

  pos_left_rad  += wL * dt;
  pos_right_rad += wR * dt;

  joint_msg.header.stamp.sec     = now_ms / 1000;
  joint_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;

  joint_msg.position.data[0] = pos_left_rad;
  joint_msg.position.data[1] = pos_right_rad;
  joint_msg.velocity.data[0] = wL;
  joint_msg.velocity.data[1] = wR;

  rcl_ret_t rc = rcl_publish(&joint_pub, &joint_msg, NULL);
  (void)rc;
}

// =================== setup ============================
void setup() {
  Serial.begin(115200);
  delay(200);

  // --- pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_B);

  stop_motors();

  // --- Wi-Fi
  while (!connectWiFi()) {
    Serial.println("[WiFi] retry...");
    delay(1000);
  }
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());

  // --- micro-ROS transport
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  Serial.println("[micro-ROS] waiting for agent...");
  waitForAgent();
  Serial.println("[micro-ROS] agent found");

  // --- init RCL
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support);

  // subscriber: /cmd_vel
  rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  // publisher: /joint_states
  rclc_publisher_init_default(
      &joint_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states");

  // ---- prepare JointState message memory (fixed) ----
  // names
  joint_msg.name.data     = joint_names_buf;
  joint_msg.name.size     = 2;
  joint_msg.name.capacity = 2;
  rosidl_runtime_c__String__init(&joint_msg.name.data[0]);
  rosidl_runtime_c__String__init(&joint_msg.name.data[1]);
  rosidl_runtime_c__String__assign(&joint_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_msg.name.data[1], "right_wheel_joint");

  // numeric arrays
  joint_msg.position.data     = pos_buf;
  joint_msg.position.size     = 2;
  joint_msg.position.capacity = 2;
  joint_msg.velocity.data     = vel_buf;
  joint_msg.velocity.size     = 2;
  joint_msg.velocity.capacity = 2;
  joint_msg.effort.data       = eff_buf;
  joint_msg.effort.size       = 2;
  joint_msg.effort.capacity   = 2;

  pos_buf[0] = pos_buf[1] = 0.0;
  vel_buf[0] = vel_buf[1] = 0.0;
  eff_buf[0] = eff_buf[1] = 0.0;

  // executor: 1 sub + 1 timer
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA);

  // publish joint_states at 50 Hz
  rclc_timer_init_default(
      &joint_timer, &support, RCL_MS_TO_NS(20), joint_timer_cb);
  rclc_executor_add_timer(&executor, &joint_timer);

  last_cmd_ms = millis();

  Serial.println("[Setup] Ready: sub /cmd_vel, pub /joint_states");
}

// =================== loop =============================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // simple Wi-Fi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(ssid, psk);
  }

  delay(2);
}
