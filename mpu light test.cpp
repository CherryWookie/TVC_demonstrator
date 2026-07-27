// ============================================================
// MPU6050_light test + calibration - confirms the sensor is now
// detected, then averages raw readings so we can get fresh offsets
// in this library's units (accel in g's, gyro in deg/s).
// ============================================================

#include <Wire.h>
#include <MPU6050_light.h>

#define SDA_PIN 21
#define SCL_PIN 22

MPU6050 mpu(Wire);

const int SAMPLES = 300;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== MPU6050_light detection + calibration test ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  if (status != 0) {
    Serial.println("Still not detected - this points to wiring/power, not the library.");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 detected successfully!");

  Serial.println("Keep it perfectly still, in its real final mounted position...");
  delay(2000);

  double ax_sum = 0, ay_sum = 0, az_sum = 0;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.print("Sampling");
  for (int i = 0; i < SAMPLES; i++) {
    mpu.fetchData(); // raw read, no offsets applied yet - that's what we're solving for
    ax_sum += mpu.getAccX();
    ay_sum += mpu.getAccY();
    az_sum += mpu.getAccZ();
    gx_sum += mpu.getGyroX();
    gy_sum += mpu.getGyroY();
    gz_sum += mpu.getGyroZ();
    if (i % 30 == 0) Serial.print(".");
    delay(10);
  }
  Serial.println(" done.");

  Serial.println();
  Serial.println("=== RAW AVERAGES (units: accel in g, gyro in deg/s) ===");
  Serial.print("Accel X avg: "); Serial.println(ax_sum / SAMPLES, 4);
  Serial.print("Accel Y avg: "); Serial.println(ay_sum / SAMPLES, 4);
  Serial.print("Accel Z avg: "); Serial.println(az_sum / SAMPLES, 4);
  Serial.print("Gyro  X avg: "); Serial.println(gx_sum / SAMPLES, 4);
  Serial.print("Gyro  Y avg: "); Serial.println(gy_sum / SAMPLES, 4);
  Serial.print("Gyro  Z avg: "); Serial.println(gz_sum / SAMPLES, 4);
  Serial.println();
  Serial.println("Whichever axis is 'up' right now should read close to 1.0 (not 9.81 this time).");
  Serial.println("The other two accel axes should be close to 0. All three gyro axes should be close to 0.");
}

void loop() {
  delay(1000);
}