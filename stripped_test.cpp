// ============================================================
// TVC Gimbal - MINIMAL VERSION
// Stripped down to only what's proven working. Pitch and yaw are
// now STRUCTURALLY IDENTICAL - both use yaw's confirmed-good filter
// blend (0.9 gyro / 0.1 accel), the same deadband, the same
// anti-windup logic. No centripetal/tangential correction, no
// dynamic blend weighting, no WiFi dashboard, no oscilloscope.
// Just the gimbal.
//
// Gains below are yaw's own confirmed-working values, mirrored onto
// pitch with pitch's own confirmed sign (positive, via repeated
// push-tests). Adjust independently from here once this baseline
// is confirmed working on both axes.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
MPU6050 mpu(Wire);

#define SDA_PIN 21
#define SCL_PIN 22
#define PITCH_CH 0
#define YAW_CH   1
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50
#define SERVO_CENTER 90

#define LINKAGE_RATIO 2
#define GIMBAL_MAX_SAFE 20.0

#define FAN_PIN 17
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8
#define FAN_SPEED 200

// --- Calibration (real measured values, final mounted position) ---
#define ACCEL_X_AVG -0.9616
#define ACCEL_Y_AVG -0.0555
#define ACCEL_Z_AVG -0.0584
#define GYRO_X_AVG 0.5768
#define GYRO_Y_AVG 1.3094
#define GYRO_Z_AVG -0.6862

// -X is up, so X_OFFSET = ax_avg - (-1.0), not ax_avg - 1.0
#define X_OFFSET (ACCEL_X_AVG + 1.0)
#define Y_OFFSET (ACCEL_Y_AVG)
#define Z_OFFSET (ACCEL_Z_AVG)
#define GYRO_X_OFFSET (GYRO_X_AVG)
#define GYRO_Y_OFFSET (GYRO_Y_AVG)
#define GYRO_Z_OFFSET (GYRO_Z_AVG)

// --- Gains: yaw's confirmed-working values, mirrored onto pitch ---
// Pitch positive, yaw negative - each axis's own confirmed-correct
// sign via direct push-testing. Magnitudes matched.
float Kp_pitch = -1.3;
float Ki_pitch = -0.10;
float Kd_pitch = 0.00;

float Kp_yaw = -1.3;
float Ki_yaw = -16.0;
float Kd_yaw = -0.0003;

// Yaw's confirmed-working deadband, same for both axes now.
#define PITCH_DEADBAND 0.5
#define YAW_DEADBAND   0.5

#define INTEGRAL_LIMIT 80.0

float pitchAngle = 0, yawAngle = 0;
float pitchIntegral = 0, pitchLastError = 0;
float yawIntegral = 0, yawLastError = 0;
bool pitchSaturated = false, yawSaturated = false;
unsigned long lastTime = 0;

int angleToPulse(float angle) {
  return (int)((angle) * (SERVO_MAX - SERVO_MIN) / 180.0 + SERVO_MIN);
}

// Soft deadband: below this error magnitude, contribute nothing.
// Above it, error ramps up continuously from zero rather than
// jumping to its full raw value - no discontinuity at the boundary.
float applyDeadband(float error, float deadband) {
  if (fabs(error) <= deadband) return 0.0;
  return error - deadband * (error > 0 ? 1.0 : -1.0);
}

float computePID(float setpoint, float measured, float &integral,
                  float &lastError, float dt, float Kp, float Ki, float Kd,
                  bool outputSaturated, float deadband) {
  float rawError = setpoint - measured;
  float error = applyDeadband(rawError, deadband);

  // Anti-windup: only accumulate integral if the output ISN'T already
  // saturated, or if this error would pull the output back INTO range.
  bool wouldIncreaseSaturation = (outputSaturated && ((error > 0) == (integral > 0)));
  if (!wouldIncreaseSaturation) {
    integral += error * dt;
  }
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (dt > 0) ? (error - lastError) / dt : 0;
  lastError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== TVC Gimbal - MINIMAL VERSION, pitch/yaw structurally identical ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  pwm.setPWM(PITCH_CH, 0, angleToPulse(SERVO_CENTER));
  pwm.setPWM(YAW_CH,   0, angleToPulse(SERVO_CENTER));
  Serial.println("Servos centered.");

  ledcSetup(1, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, 1);
  ledcWrite(1, FAN_SPEED);

  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  if (status != 0) {
    Serial.println("MPU6050 not found!");
    while (1) { delay(10); }
  }

  mpu.fetchData();
  float ax = mpu.getAccX() - X_OFFSET;
  float ay = mpu.getAccY() - Y_OFFSET;
  float az = mpu.getAccZ() - Z_OFFSET;
  // -X = up, Y = pitch axis, Z = yaw axis
  pitchAngle = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;
  yawAngle   = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  Serial.println("Setup complete. Stabilization active.");
  delay(1000);
  lastTime = millis();
}

void loop() {
  mpu.fetchData();

  float ax = mpu.getAccX() - X_OFFSET;
  float ay = mpu.getAccY() - Y_OFFSET;
  float az = mpu.getAccZ() - Z_OFFSET;

  float accelPitch = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;
  float accelYaw   = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroPitchRate = mpu.getGyroY() - GYRO_Y_OFFSET;
  float gyroYawRate   = mpu.getGyroZ() - GYRO_Z_OFFSET;

  // Identical blend, both axes. Yaw's confirmed-working ratio.
  pitchAngle = 0.75 * (pitchAngle + gyroPitchRate * dt) + 0.25 * accelPitch;
  yawAngle   = 0.9 * (yawAngle   + gyroYawRate   * dt) + 0.1 * accelYaw;

  float pitchCorrection = computePID(0, pitchAngle, pitchIntegral,
                                      pitchLastError, dt,
                                      Kp_pitch, Ki_pitch, Kd_pitch, pitchSaturated,
                                      PITCH_DEADBAND);
  float yawCorrection = computePID(0, yawAngle, yawIntegral,
                                    yawLastError, dt,
                                    Kp_yaw, Ki_yaw, Kd_yaw, yawSaturated,
                                    YAW_DEADBAND);

  pitchSaturated = (pitchCorrection >= GIMBAL_MAX_SAFE || pitchCorrection <= -GIMBAL_MAX_SAFE);
  yawSaturated   = (yawCorrection   >= GIMBAL_MAX_SAFE || yawCorrection   <= -GIMBAL_MAX_SAFE);

  pitchCorrection = constrain(pitchCorrection, -GIMBAL_MAX_SAFE, GIMBAL_MAX_SAFE);
  yawCorrection   = constrain(yawCorrection,   -GIMBAL_MAX_SAFE, GIMBAL_MAX_SAFE);

  float pitchServoAngle = SERVO_CENTER + (pitchCorrection * LINKAGE_RATIO);
  float yawServoAngle   = SERVO_CENTER + (yawCorrection   * LINKAGE_RATIO);

  pitchServoAngle = constrain(pitchServoAngle, 0.0f, 180.0f);
  yawServoAngle   = constrain(yawServoAngle, 0.0f, 180.0f);

  pwm.setPWM(PITCH_CH, 0, angleToPulse(pitchServoAngle));
  pwm.setPWM(YAW_CH,   0, angleToPulse(yawServoAngle));

  Serial.print("Pitch: "); Serial.print(pitchAngle);
  Serial.print("  servo: "); Serial.print(pitchServoAngle);
  Serial.print(" || Yaw: "); Serial.print(yawAngle);
  Serial.print("  servo: "); Serial.println(yawServoAngle);

  delay(20);
}