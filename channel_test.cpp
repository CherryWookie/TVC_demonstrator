// ============================================================
// CHANNEL IDENTIFICATION TEST - no sensor, no PID, no math.
// Moves ONE channel at a time, clearly labeled, so you can watch
// which physical servo actually responds to which channel number.
// This isolates hardware wiring/channel mapping from everything else.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22
#define SERVO_MIN 150
#define SERVO_MAX 600

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Center both first
  pwm.setPWM(0, 0, angleToPulse(90));
  pwm.setPWM(1, 0, angleToPulse(90));
  Serial.println("Both centered at 90. Starting channel identification.");
  delay(2000);
}

void loop() {
  Serial.println();
  Serial.println("=== Moving CHANNEL 0 to 100, then back to 90 ===");
  Serial.println("Watch which physical servo moves.");
  pwm.setPWM(0, 0, angleToPulse(100));
  delay(1500);
  pwm.setPWM(0, 0, angleToPulse(90));
  delay(1500);

  Serial.println("=== Moving CHANNEL 1 to 100, then back to 90 ===");
  Serial.println("Watch which physical servo moves.");
  pwm.setPWM(1, 0, angleToPulse(100));
  delay(1500);
  pwm.setPWM(1, 0, angleToPulse(90));
  delay(1500);
}