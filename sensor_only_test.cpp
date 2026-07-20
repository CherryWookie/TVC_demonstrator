// ============================================================
// SENSOR-ONLY DIAGNOSTIC - servos just center and hold. No PID,
// no correction, no feedback loop. Pure sensor math validation.
// This is a fresh, distinct sketch - no ambiguity about what's
// actually running.
//
// Watch pitchAngle and yawAngle while you tilt each axis by hand
// (or however you can safely move it with linkages attached) and
// confirm each one only responds to its own axis.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22
#define PITCH_CH 0
#define YAW_CH   1
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_CENTER 90

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

Adafruit_MPU6050 mpu;

#define X_OFFSET 0.1151
#define Y_OFFSET 0.9027
#define Z_OFFSET 1.8392
#define GYRO_X_OFFSET 0.0149
#define GYRO_Y_OFFSET 0.0135
#define GYRO_Z_OFFSET -0.0346

float pitchAngle = 0;
float yawAngle = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SENSOR-ONLY DIAGNOSTIC (build marker: SO-1) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  pwm.setPWM(PITCH_CH, 0, angleToPulse(SERVO_CENTER));
  pwm.setPWM(YAW_CH,   0, angleToPulse(SERVO_CENTER));
  Serial.println("Servos centered and holding. No PID running.");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = millis();
  Serial.println("Now tilt PITCH only, watch which value changes.");
  Serial.println("Then tilt YAW only, watch which value changes.");
  delay(1000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x - X_OFFSET;
  float ay = a.acceleration.y - Y_OFFSET;
  float az = a.acceleration.z - Z_OFFSET;

  float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelYaw   = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroPitchRate = (g.gyro.z - GYRO_Z_OFFSET) * 180.0 / PI;
  float gyroYawRate   = (g.gyro.y - GYRO_Y_OFFSET) * 180.0 / PI;

  pitchAngle = 0.98 * (pitchAngle + gyroPitchRate * dt) + 0.02 * accelPitch;
  yawAngle   = 0.98 * (yawAngle   + gyroYawRate   * dt) + 0.02 * accelYaw;

  Serial.print("raw ax:"); Serial.print(ax, 2);
  Serial.print(" ay:"); Serial.print(ay, 2);
  Serial.print(" az:"); Serial.print(az, 2);
  Serial.print("  |  pitchAngle:"); Serial.print(pitchAngle, 2);
  Serial.print("  yawAngle:"); Serial.println(yawAngle, 2);

  delay(50);
}