// ============================================================
// CENTERING/ALIGNMENT CHECK ONLY - no PID, no sweep, no motion
// after the initial center command. Run this FIRST after installing
// linkages, before any closed-loop testing.
//
// SAFETY: Have your hand near the power switch/USB cable. Watch the
// linkage closely during the first setPWM call - if anything binds,
// grinds, or strains visibly, cut power immediately.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22
#define PITCH_CH 0
#define YAW_CH   1
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50
#define SERVO_CENTER 90

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== CENTERING CHECK ===");
  Serial.println("About to command both servos to 90 degrees (center).");
  Serial.println("WATCH THE LINKAGE. If it binds or strains, cut power now.");
  delay(3000); // gives you 3 seconds to get your hand near the power switch

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Centering pitch...");
  pwm.setPWM(PITCH_CH, 0, angleToPulse(SERVO_CENTER));
  delay(1000);

  Serial.println("Centering yaw...");
  pwm.setPWM(YAW_CH, 0, angleToPulse(SERVO_CENTER));
  delay(1000);

  Serial.println("Both servos at commanded center (90 deg). Holding.");
  Serial.println("Check visually: does this match the tube's true neutral position?");
  Serial.println("If NOT aligned, loosen the servo horn screw, rotate the horn");
  Serial.println("on its spline to match this held position to true neutral,");
  Serial.println("then re-tighten. Do NOT power off while doing this - the");
  Serial.println("servo is actively holding 90 deg, which is your reference.");
}

void loop() {
  // Intentionally empty - servos just hold their last commanded position.
  delay(1000);
}