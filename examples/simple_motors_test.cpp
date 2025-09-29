#include <Arduino.h>   // needed for Serial, pinMode, ledc*, etc.

// ---------------- Pins (your wiring) ----------------
#define ENA 25      // PWM for left motor
#define IN1 26
#define IN2 27

#define ENB 14      // PWM for right motor
#define IN3 32
#define IN4 33

// ---------------- PWM config ------------------------
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;
const int PWM_FREQ = 3000;   // Hz
const int PWM_RES  = 8;      // 8-bit (0–255 duty)

// ---- Forward declarations (fix for stopMotors not declared) ----
void stopMotors();
void setMotorA(bool forward, uint8_t duty);
void setMotorB(bool forward, uint8_t duty);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Motor wiring test");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_A);

  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_B);

  stopMotors();  // now declared above
}

// ---------------- Helpers ---------------------------
void setMotorA(bool forward, uint8_t duty) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  ledcWrite(PWM_CH_A, duty);
}

void setMotorB(bool forward, uint8_t duty) {
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  ledcWrite(PWM_CH_B, duty);
}

void stopMotors() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

// ---------------- Main loop -------------------------
void loop() {
  Serial.println("Forward slow");
  setMotorA(true, 80);
  setMotorB(true, 80);
  delay(2000);

  Serial.println("Forward fast");
  setMotorA(true, 200);
  setMotorB(true, 200);
  delay(2000);

  Serial.println("Stop");
  stopMotors();
  delay(1500);

  Serial.println("Reverse slow");
  setMotorA(false, 80);
  setMotorB(false, 80);
  delay(2000);

  Serial.println("Reverse fast");
  setMotorA(false, 200);
  setMotorB(false, 200);
  delay(2000);

  Serial.println("Stop");
  stopMotors();
  delay(3000);
}
