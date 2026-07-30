
// ============================================================
// TVC Gimbal - Closed-loop PID stabilization + WiFi live tuning
// Now using MPU6050_light instead of Adafruit_MPU6050 (generic
// board wasn't passing the Adafruit library's identity check).
//
// STATUS: integration test only - sensor is sitting flat on the
// desk, NOT yet in its final mounted position. Axis assignment
// below (pitch=X, yaw=Y) is a PLACEHOLDER. Once mounted, redo the
// raw-axis rotation test and update accordingly, same process as
// every previous remount.
//
// Units: this library returns accel in g (not m/s^2) and gyro
// directly in deg/s (not rad/s) - motion gate threshold updated
// to compare against 1.0, not 9.81.
// ============================================================
 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
 

// --- WiFi credentials - fill these in ---
const char* WIFI_SSID = "Slower 2.4";
const char* WIFI_PASSWORD = "6a9e5b839d";


AsyncWebServer server(80);
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
 
#define SDA_PIN 21
#define SCL_PIN 22
#define PITCH_CH 0
#define YAW_CH   1
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50
#define SERVO_CENTER 90
 
#define LINKAGE_RATIO 2
#define GIMBAL_MAX_SAFE 17.0
 
#define FAN_PIN 17
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8
#define FAN_SPEED 200
 
int angleToPulse(float angle) {
  return (int)((angle) * (SERVO_MAX - SERVO_MIN) / 180.0 + SERVO_MIN);
}
 
MPU6050 mpu(Wire);
 
// --- Accelerometer offsets (units: g) - PLACEHOLDER, flat on desk ---
#define X_OFFSET 0.0221
#define Y_OFFSET 0.0322
#define Z_OFFSET 0.1244
 
// --- Gyroscope zero-rate offsets (units: deg/s) ---
#define GYRO_X_OFFSET 0.4163
#define GYRO_Y_OFFSET 1.7458
#define GYRO_Z_OFFSET 0.7683
 
float pitchAngle = 0;
float yawAngle = 0;
unsigned long lastTime = 0;
 
float Kp_pitch = -1.8;
float Ki_pitch = 0.02;
float Kd_pitch = 0.0005;
 
float Kp_yaw = 1.8;
float Ki_yaw = 0.02;
float Kd_yaw = 0.0003;
 
float pitchIntegral = 0, pitchLastError = 0;
float yawIntegral = 0, yawLastError = 0;
 
#define INTEGRAL_LIMIT 20.0
 
float telemetry_pitch = 0, telemetry_yaw = 0;
float telemetry_pitchCorrection = 0, telemetry_yawCorrection = 0;
float telemetry_pitchServo = 90, telemetry_yawServo = 90;
 
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>TVC Gimbal Tuner</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; max-width: 480px; margin: 20px auto; padding: 0 16px; }
    h2 { margin-top: 24px; }
    label { display: block; margin-top: 10px; font-weight: bold; }
    input { width: 100%; padding: 6px; font-size: 16px; box-sizing: border-box; }
    button { margin-top: 16px; padding: 10px; width: 100%; font-size: 16px; }
    #telemetry { background: #f0f0f0; padding: 12px; border-radius: 6px; font-family: monospace; }
  </style>
</head>
<body>
  <h1>TVC Gimbal Tuner</h1>
  <div id="telemetry">Loading telemetry...</div>
 
  <h2>Pitch gains</h2>
  <label>Kp</label><input type="number" step="0.01" id="kp_pitch">
  <label>Ki</label><input type="number" step="0.001" id="ki_pitch">
  <label>Kd</label><input type="number" step="0.01" id="kd_pitch">
 
  <h2>Yaw gains</h2>
  <label>Kp</label><input type="number" step="0.01" id="kp_yaw">
  <label>Ki</label><input type="number" step="0.001" id="ki_yaw">
  <label>Kd</label><input type="number" step="0.01" id="kd_yaw">
 
  <button onclick="updateGains()">Apply</button>
 
  <script>
    async function loadGains() {
      const r = await fetch('/gains');
      const g = await r.json();
      kp_pitch.value = g.kp_pitch; ki_pitch.value = g.ki_pitch; kd_pitch.value = g.kd_pitch;
      kp_yaw.value = g.kp_yaw; ki_yaw.value = g.ki_yaw; kd_yaw.value = g.kd_yaw;
    }
    async function updateGains() {
      const params = new URLSearchParams({
        kp_pitch: kp_pitch.value, ki_pitch: ki_pitch.value, kd_pitch: kd_pitch.value,
        kp_yaw: kp_yaw.value, ki_yaw: ki_yaw.value, kd_yaw: kd_yaw.value
      });
      await fetch('/update?' + params.toString());
    }
    async function pollTelemetry() {
      try {
        const r = await fetch('/data');
        const d = await r.json();
        document.getElementById('telemetry').innerHTML =
          'Pitch: ' + d.pitch.toFixed(2) + ' deg  (servo ' + d.pitchServo.toFixed(1) + ')<br>' +
          'Yaw: ' + d.yaw.toFixed(2) + ' deg  (servo ' + d.yawServo.toFixed(1) + ')';
      } catch (e) {}
      setTimeout(pollTelemetry, 200);
    }
    loadGains();
    pollTelemetry();
  </script>
</body>
</html>
)rawliteral";
 
void setup() {
  Serial.begin(115200);
  Serial.println("=== TVC Gimbal PID + WiFi dashboard (build: MAIN-2, MPU6050_light) ===");
 
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
  Serial.println("Fan initialized.");
 
  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  if (status != 0) {
    Serial.println("MPU6050 not found!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 found and initialized.");
 
  mpu.fetchData();
  float ax = mpu.getAccX() - X_OFFSET;
  float ay = mpu.getAccY() - Y_OFFSET;
  float az = mpu.getAccZ() - Z_OFFSET;
  // PLACEHOLDER axis assignment - pitch=X, yaw=Y - re-derive once mounted.
  pitchAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  yawAngle   = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! Open this in a browser: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connect failed - continuing without dashboard.");
  }
 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });
 
  server.on("/gains", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"kp_pitch\":" + String(Kp_pitch) +
                  ",\"ki_pitch\":" + String(Ki_pitch) +
                  ",\"kd_pitch\":" + String(Kd_pitch) +
                  ",\"kp_yaw\":" + String(Kp_yaw) +
                  ",\"ki_yaw\":" + String(Ki_yaw) +
                  ",\"kd_yaw\":" + String(Kd_yaw) + "}";
    request->send(200, "application/json", json);
  });
 
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp_pitch")) Kp_pitch = request->getParam("kp_pitch")->value().toFloat();
    if (request->hasParam("ki_pitch")) Ki_pitch = request->getParam("ki_pitch")->value().toFloat();
    if (request->hasParam("kd_pitch")) Kd_pitch = request->getParam("kd_pitch")->value().toFloat();
    if (request->hasParam("kp_yaw"))   Kp_yaw   = request->getParam("kp_yaw")->value().toFloat();
    if (request->hasParam("ki_yaw"))   Ki_yaw   = request->getParam("ki_yaw")->value().toFloat();
    if (request->hasParam("kd_yaw"))   Kd_yaw   = request->getParam("kd_yaw")->value().toFloat();
    Serial.println("Gains updated via web dashboard.");
    request->send(200, "text/plain", "OK");
  });
 
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"pitch\":" + String(telemetry_pitch) +
                  ",\"yaw\":" + String(telemetry_yaw) +
                  ",\"pitchCorrection\":" + String(telemetry_pitchCorrection) +
                  ",\"yawCorrection\":" + String(telemetry_yawCorrection) +
                  ",\"pitchServo\":" + String(telemetry_pitchServo) +
                  ",\"yawServo\":" + String(telemetry_yawServo) + "}";
    request->send(200, "application/json", json);
  });
 
  server.begin();
 
  Serial.println("Setup complete. Stabilization active.");
  delay(1000);
  lastTime = millis();
}
 
float computePID(float setpoint, float measured, float &integral,
                  float &lastError, float dt, float Kp, float Ki, float Kd) {
  float error = setpoint - measured;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = (dt > 0) ? (error - lastError) / dt : 0;
  lastError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}
 
void loop() {
  mpu.fetchData();
 
  float ax = mpu.getAccX() - X_OFFSET;
  float ay = mpu.getAccY() - Y_OFFSET;
  float az = mpu.getAccZ() - Z_OFFSET;
 
  // PLACEHOLDER axis assignment - pitch=X, yaw=Y - re-derive once mounted.
  float accelPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float accelYaw   = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
 
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
 
  // Gyro already in deg/s from this library - no *180/PI conversion needed.
  float gyroPitchRate = mpu.getGyroX() - GYRO_X_OFFSET;
  float gyroYawRate   = mpu.getGyroY() - GYRO_Y_OFFSET;
 
  // --- Motion gate --- units are g now, so compare against 1.0, not 9.81.
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  bool isStationary = (fabs(accelMagnitude - 1.0) < 0.05); // tune if needed
 
  if (isStationary) {
    pitchAngle = 0.95 * (pitchAngle + gyroPitchRate * dt) + 0.05 * accelPitch;
  } else {
    pitchAngle = pitchAngle + gyroPitchRate * dt;
  }
  yawAngle = 0.98 * (yawAngle + gyroYawRate * dt) + 0.02 * accelYaw;
 
  float pitchCorrection = computePID(0, pitchAngle, pitchIntegral,
                                      pitchLastError, dt,
                                      Kp_pitch, Ki_pitch, Kd_pitch);
  float yawCorrection = computePID(0, yawAngle, yawIntegral,
                                    yawLastError, dt,
                                    Kp_yaw, Ki_yaw, Kd_yaw);
 
  pitchCorrection = constrain(pitchCorrection, -GIMBAL_MAX_SAFE, GIMBAL_MAX_SAFE);
  yawCorrection   = constrain(yawCorrection,   -GIMBAL_MAX_SAFE, GIMBAL_MAX_SAFE);
 
  float pitchServoAngle = SERVO_CENTER + (pitchCorrection * LINKAGE_RATIO);
  float yawServoAngle   = SERVO_CENTER + (yawCorrection   * LINKAGE_RATIO);
 
  pitchServoAngle = constrain(pitchServoAngle, 0.0f, 180.0f);
  yawServoAngle   = constrain(yawServoAngle, 0.0f, 180.0f);
 
  pwm.setPWM(PITCH_CH, 0, angleToPulse(pitchServoAngle));
  pwm.setPWM(YAW_CH,   0, angleToPulse(yawServoAngle));
 
  telemetry_pitch = pitchAngle;
  telemetry_yaw = yawAngle;
  telemetry_pitchCorrection = pitchCorrection;
  telemetry_yawCorrection = yawCorrection;
  telemetry_pitchServo = pitchServoAngle;
  telemetry_yawServo = yawServoAngle;
 
  Serial.print("Pitch: "); Serial.print(pitchAngle);
  Serial.print("  servo: "); Serial.print(pitchServoAngle);
  Serial.print(" || Yaw: "); Serial.print(yawAngle);
  Serial.print("  servo: "); Serial.println(yawServoAngle);
 
  delay(20);
}
 
