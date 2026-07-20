// ============================================================
// CALIBRATION SKETCH - run with the IMU in its real, final,
// mounted position (linkages attached, at rest / centered).
// Do NOT move the mechanism during this run.
//
// This averages many raw readings per axis and prints suggested
// offsets. Write down all six numbers, then bake them into the
// main PID sketch.
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MPU6050 mpu;

const int SAMPLES = 300;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== MPU6050 6-axis calibration ===");
  Serial.println("Keep the mechanism perfectly still until this finishes.");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500); // let the sensor settle after configuration

  double ax_sum = 0, ay_sum = 0, az_sum = 0;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.print("Sampling");
  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;

    if (i % 30 == 0) Serial.print(".");
    delay(10);
  }
  Serial.println(" done.");

  double ax_avg = ax_sum / SAMPLES;
  double ay_avg = ay_sum / SAMPLES;
  double az_avg = az_sum / SAMPLES;
  double gx_avg = gx_sum / SAMPLES;
  double gy_avg = gy_sum / SAMPLES;
  double gz_avg = gz_sum / SAMPLES;

  Serial.println();
  Serial.println("=== RAW AVERAGES (write these down) ===");
  Serial.print("Accel X avg: "); Serial.println(ax_avg, 4);
  Serial.print("Accel Y avg: "); Serial.println(ay_avg, 4);
  Serial.print("Accel Z avg: "); Serial.println(az_avg, 4);
  Serial.print("Gyro  X avg: "); Serial.println(gx_avg, 4);
  Serial.print("Gyro  Y avg: "); Serial.println(gy_avg, 4);
  Serial.print("Gyro  Z avg: "); Serial.println(gz_avg, 4);

  Serial.println();
  Serial.println("=== SUGGESTED OFFSETS ===");
  Serial.println("For the axis that's vertical (up) in this exact resting");
  Serial.println("position, expected = +9.81. For the other two horizontal");
  Serial.println("axes, expected = 0. For ALL THREE gyro axes, expected = 0.");
  Serial.println();
  Serial.print("If Y is up right now: X_OFFSET = "); Serial.println(ax_avg, 4);
  Serial.print("                      Y_OFFSET = "); Serial.println(ay_avg - 9.81, 4);
  Serial.print("                      Z_OFFSET = "); Serial.println(az_avg, 4);
  Serial.println();
  Serial.print("GYRO_X_OFFSET = "); Serial.println(gx_avg, 4);
  Serial.print("GYRO_Y_OFFSET = "); Serial.println(gy_avg, 4);
  Serial.print("GYRO_Z_OFFSET = "); Serial.println(gz_avg, 4);

  Serial.println();
  Serial.println("Copy the 6 offset values above into the PID sketch,");
  Serial.println("then re-upload that file. Calibration complete - halting.");
}

void loop() {
  delay(1000);
}