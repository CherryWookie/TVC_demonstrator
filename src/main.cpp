// ============================================================
// TVC Gimbal - MINIMAL control loop + WiFi dashboard
// Control logic is the confirmed-working minimal version (exact
// gains/blends preserved). Dashboard adds live gain tuning and a
// side-by-side oscilloscope for both axes, without reintroducing
// any of the centripetal/tangential/dynamic-blend experiments.
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

// --- WiFi credentials - fill these in ---
const char* WIFI_SSID = "Slower 2.4";
const char* WIFI_PASSWORD = "6a9e5b839d";

AsyncWebServer server(80);

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

// --- Gains: confirmed-working values, live-tunable via dashboard ---
float Kp_pitch = -0.6;
float Ki_pitch = -6;
float Kd_pitch = -0.0001;

float Kp_yaw = -0.6;
float Ki_yaw = -15.0;
float Kd_yaw = -0.001;

#define PITCH_DEADBAND 0.35
#define YAW_DEADBAND   0.35

#define INTEGRAL_LIMIT 60.0

float pitchAngle = 0, yawAngle = 0;
float pitchIntegral = 0, pitchLastError = 0;
float yawIntegral = 0, yawLastError = 0;
bool pitchSaturated = false, yawSaturated = false;
unsigned long lastTime = 0;

float telemetry_pitch = 0, telemetry_yaw = 0;
float telemetry_pitchServo = 90, telemetry_yawServo = 90;

// --- Oscilloscope trace buffers, both axes ---
// No centripetal/tangential correction anymore, so channels are just
// the core signals: raw accel component driving each axis, the
// accel-only angle estimate, gyro rate, the final blended angle, and
// the servo output - completing the picture from raw sensor through
// to actuator command, useful for report step-response graphs.
#define TRACE_LEN 1000   // ~24 seconds of history at 24ms/sample
float trace_paz[TRACE_LEN], trace_paccel[TRACE_LEN], trace_pgyro[TRACE_LEN], trace_pangle[TRACE_LEN], trace_pservo[TRACE_LEN];
float trace_yay[TRACE_LEN], trace_yaccel[TRACE_LEN], trace_ygyro[TRACE_LEN], trace_yangle[TRACE_LEN], trace_yservo[TRACE_LEN];
unsigned long trace_millis[TRACE_LEN];
int traceIndex = 0;
bool traceFull = false;
bool recording = false;

int angleToPulse(float angle) {
  return (int)((angle) * (SERVO_MAX - SERVO_MIN) / 180.0 + SERVO_MIN);
}

float applyDeadband(float error, float deadband) {
  if (fabs(error) <= deadband) return 0.0;
  return error - deadband * (error > 0 ? 1.0 : -1.0);
}

float computePID(float setpoint, float measured, float &integral,
                  float &lastError, float dt, float Kp, float Ki, float Kd,
                  bool outputSaturated, float deadband) {
  float rawError = setpoint - measured;
  float error = applyDeadband(rawError, deadband);

  bool wouldIncreaseSaturation = (outputSaturated && ((error > 0) == (integral > 0)));
  if (!wouldIncreaseSaturation) {
    integral += error * dt;
  }
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (dt > 0) ? (error - lastError) / dt : 0;
  lastError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

String traceArrayToJson(float* arr, int len, int startIdx, int count) {
  String s = "[";
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % len;
    s += String(arr[idx], 4);
    if (i < count - 1) s += ",";
  }
  s += "]";
  return s;
}

String traceMillisToJson(unsigned long* arr, int len, int startIdx, int count) {
  String s = "[";
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % len;
    s += String(arr[idx]);
    if (i < count - 1) s += ",";
  }
  s += "]";
  return s;
}

void writeTraceArrayToFile(File &f, float* arr, int len, int startIdx, int count) {
  f.print("[");
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % len;
    f.print(arr[idx], 4);
    if (i < count - 1) f.print(",");
  }
  f.print("]");
}

void writeCaptureToFile() {
  int start = traceFull ? traceIndex : 0;
  int count = traceFull ? TRACE_LEN : traceIndex;

  File f = LittleFS.open("/capture.json", "w");
  if (!f) {
    Serial.println("Failed to open /capture.json for writing!");
    return;
  }

  // Stream directly to the file, one piece at a time - never build the
  // whole JSON as one giant String in RAM. At full buffer length the
  // complete JSON is on the order of 150-200KB, which reliably
  // exhausted heap when built as a single String (this is what caused
  // truncated/corrupted capture files before this fix).
  f.print("{\"time\":[");
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % TRACE_LEN;
    f.print(trace_millis[idx]);
    if (i < count - 1) f.print(",");
  }
  f.print("],");

  f.print("\"paz\":");    writeTraceArrayToFile(f, trace_paz, TRACE_LEN, start, count);    f.print(",");
  f.print("\"paccel\":"); writeTraceArrayToFile(f, trace_paccel, TRACE_LEN, start, count); f.print(",");
  f.print("\"pgyro\":");  writeTraceArrayToFile(f, trace_pgyro, TRACE_LEN, start, count);  f.print(",");
  f.print("\"pangle\":"); writeTraceArrayToFile(f, trace_pangle, TRACE_LEN, start, count); f.print(",");
  f.print("\"pservo\":"); writeTraceArrayToFile(f, trace_pservo, TRACE_LEN, start, count); f.print(",");
  f.print("\"yay\":");    writeTraceArrayToFile(f, trace_yay, TRACE_LEN, start, count);    f.print(",");
  f.print("\"yaccel\":"); writeTraceArrayToFile(f, trace_yaccel, TRACE_LEN, start, count); f.print(",");
  f.print("\"ygyro\":");  writeTraceArrayToFile(f, trace_ygyro, TRACE_LEN, start, count);  f.print(",");
  f.print("\"yangle\":"); writeTraceArrayToFile(f, trace_yangle, TRACE_LEN, start, count); f.print(",");
  f.print("\"yservo\":"); writeTraceArrayToFile(f, trace_yservo, TRACE_LEN, start, count);

  f.print("}");
  size_t bytesWritten = f.size();
  f.close();
  Serial.println("Capture written to /capture.json (" + String(bytesWritten) + " bytes, " + String(count) + " samples).");
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>TVC Gimbal Tuner</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; max-width: 780px; margin: 20px auto; padding: 0 16px; }
    h2 { margin-top: 24px; }
    label { display: block; margin-top: 10px; font-weight: bold; }
    input { width: 100%; padding: 6px; font-size: 16px; box-sizing: border-box; }
    button { margin-top: 16px; padding: 10px; width: 100%; font-size: 16px; }
    #telemetry { background: #f0f0f0; padding: 12px; border-radius: 6px; font-family: monospace; }
    .gainCols { display: flex; gap: 20px; }
    .gainCols > div { flex: 1; }
    #scopeRow { display: flex; gap: 10px; margin-top: 8px; }
    #scopeRow canvas { width: 50%; height: 600px; background: #111; border-radius: 6px; }
    #scopeControls { display: flex; gap: 10px; align-items: center; margin-top: 8px; }
    #scopeControls button { width: auto; padding: 8px 16px; margin-top: 0; }
  </style>
</head>
<body>
  <h1>TVC Gimbal Tuner</h1>
  <div id="telemetry">Loading telemetry...</div>

  <h2>Gains</h2>
  <div class="gainCols">
    <div>
      <h3>Pitch</h3>
      <label>Kp</label><input type="number" step="0.01" id="kp_pitch">
      <label>Ki</label><input type="number" step="0.001" id="ki_pitch">
      <label>Kd</label><input type="number" step="0.0001" id="kd_pitch">
    </div>
    <div>
      <h3>Yaw</h3>
      <label>Kp</label><input type="number" step="0.01" id="kp_yaw">
      <label>Ki</label><input type="number" step="0.001" id="ki_yaw">
      <label>Kd</label><input type="number" step="0.0001" id="kd_yaw">
    </div>
  </div>
  <button onclick="updateGains()">Apply</button>

  <h2>Oscilloscope (both axes, ~3s window, full loop-rate resolution)</h2>
  <div id="scopeRow">
    <canvas id="scopePitch" width="360" height="600"></canvas>
    <canvas id="scopeYaw" width="360" height="600"></canvas>
  </div>
  <div id="scopeControls">
    <button onclick="scopePaused = !scopePaused">Pause / Resume</button>
    <span id="scopeStatus">running</span>
  </div>
  <div id="scopeControls">
    <button onclick="startRecording()">Start Recording</button>
    <button onclick="stopRecording()">Stop Recording &amp; Save</button>
    <span id="captureStatus">checking...</span>
  </div>
  <a href="/download" download="capture.json">
    <button type="button">Download Captured JSON</button>
  </a>

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

    var scopePaused = false;
    const pitchChannels = [
      { key: 'paz',    label: 'az raw (g)',        color: '#e74c3c', min: -1.5, max: 1.5 },
      { key: 'paccel', label: 'accelPitch (deg)',  color: '#3498db', min: -60,  max: 60 },
      { key: 'pgyro',  label: 'gyro rate (deg/s)', color: '#e67e22', min: -200, max: 200 },
      { key: 'pangle', label: 'pitch angle (deg)', color: '#9b59b6', min: -30,  max: 30 },
      { key: 'pservo', label: 'servo output (deg)', color: '#2ecc71', min: 40,  max: 140 },
    ];
    const yawChannels = [
      { key: 'yay',    label: 'ay raw (g)',        color: '#e74c3c', min: -1.5, max: 1.5 },
      { key: 'yaccel', label: 'accelYaw (deg)',    color: '#3498db', min: -60,  max: 60 },
      { key: 'ygyro',  label: 'gyro rate (deg/s)', color: '#e67e22', min: -200, max: 200 },
      { key: 'yangle', label: 'yaw angle (deg)',   color: '#9b59b6', min: -30,  max: 30 },
      { key: 'yservo', label: 'servo output (deg)', color: '#2ecc71', min: 40,  max: 140 },
    ];

    function drawScope(canvasId, channels, data) {
      const canvas = document.getElementById(canvasId);
      const ctx = canvas.getContext('2d');
      const W = canvas.width, H = canvas.height;
      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, W, H);
      const rowH = H / channels.length;

      channels.forEach((ch, i) => {
        const arr = data[ch.key];
        const yTop = i * rowH;

        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        ctx.strokeRect(0, yTop, W, rowH);

        if (!arr || arr.length < 2) return;

        const minV = ch.min;
        const maxV = ch.max;
        const fullRange = maxV - minV;

        if (minV < 0 && maxV > 0) {
          const zeroY = yTop + rowH - ((0 - minV) / fullRange) * rowH;
          ctx.strokeStyle = '#444';
          ctx.beginPath();
          ctx.moveTo(0, zeroY); ctx.lineTo(W, zeroY);
          ctx.stroke();
        }

        ctx.strokeStyle = ch.color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        let clipped = false;
        arr.forEach((v, idx) => {
          const x = (idx / (arr.length - 1)) * W;
          const vClamped = Math.max(minV, Math.min(maxV, v));
          if (v < minV || v > maxV) clipped = true;
          const y = yTop + rowH - ((vClamped - minV) / fullRange) * rowH;
          if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();

        const last = arr[arr.length - 1];
        ctx.fillStyle = clipped ? '#ff5555' : '#eee';
        ctx.font = '10px monospace';
        ctx.fillText(
          ch.label + '  scale:[' + minV + ',' + maxV + ']' + (clipped ? '  CLIPPING' : '') + '  last:' + last.toFixed(2),
          4, yTop + 12
        );
      });
    }

    async function pollScope() {
      if (!scopePaused) {
        try {
          const r = await fetch('/trace');
          const d = await r.json();
          drawScope('scopePitch', pitchChannels, d);
          drawScope('scopeYaw', yawChannels, d);
          document.getElementById('scopeStatus').textContent = 'running';
        } catch (e) {}
      } else {
        document.getElementById('scopeStatus').textContent = 'paused';
      }
      setTimeout(pollScope, 150);
    }

    async function startRecording() {
      await fetch('/start_recording');
      pollCaptureStatus();
    }
    async function stopRecording() {
      await fetch('/stop_recording');
      pollCaptureStatus();
    }
    async function pollCaptureStatus() {
      try {
        const r = await fetch('/capture_status');
        const d = await r.json();
        const el = document.getElementById('captureStatus');
        el.textContent = d.status + ' (' + d.samples + ' samples, ~' + (d.samples * 0.024).toFixed(1) + 's)';
        el.style.color = d.status === 'recording' ? '#e74c3c' : '#2ecc71';
      } catch (e) {}
      setTimeout(pollCaptureStatus, 500);
    }

    loadGains();
    pollTelemetry();
    pollScope();
    pollCaptureStatus();
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Serial.println("=== TVC Gimbal - minimal loop + dashboard ===");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted.");
  }

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
  pitchAngle = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;
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
                  ",\"pitchServo\":" + String(telemetry_pitchServo) +
                  ",\"yawServo\":" + String(telemetry_yawServo) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/capture.json")) {
      request->send(LittleFS, "/capture.json", "application/json", true);
    } else {
      request->send(404, "text/plain", "No capture yet - trigger one first, then check /capture_status.");
    }
  });

  server.on("/start_recording", HTTP_GET, [](AsyncWebServerRequest *request) {
    traceIndex = 0;
    traceFull = false;
    recording = true;
    Serial.println("Recording started.");
    request->send(200, "text/plain", "OK");
  });

  server.on("/stop_recording", HTTP_GET, [](AsyncWebServerRequest *request) {
    recording = false;
    writeCaptureToFile();
    Serial.println("Recording stopped, capture saved.");
    request->send(200, "text/plain", "OK");
  });

  server.on("/capture_status", HTTP_GET, [](AsyncWebServerRequest *request) {
    int count = traceFull ? TRACE_LEN : traceIndex;
    String status = recording ? "recording" : "stopped";
    request->send(200, "application/json",
      "{\"status\":\"" + status + "\",\"samples\":" + String(count) + "}");
  });

  server.on("/trace", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Live scope only needs a recent window, not the whole recording
    // buffer - cap it here so live polling stays cheap and safe
    // regardless of how large TRACE_LEN is for full-session capture.
    #define LIVE_SCOPE_WINDOW 150
    int fullCount = traceFull ? TRACE_LEN : traceIndex;
    int count = min(fullCount, LIVE_SCOPE_WINDOW);
    int start = (traceIndex - count + TRACE_LEN) % TRACE_LEN;
    String json = "{";
    json += "\"paz\":"    + traceArrayToJson(trace_paz, TRACE_LEN, start, count) + ",";
    json += "\"paccel\":" + traceArrayToJson(trace_paccel, TRACE_LEN, start, count) + ",";
    json += "\"pgyro\":"  + traceArrayToJson(trace_pgyro, TRACE_LEN, start, count) + ",";
    json += "\"pangle\":" + traceArrayToJson(trace_pangle, TRACE_LEN, start, count) + ",";
    json += "\"pservo\":" + traceArrayToJson(trace_pservo, TRACE_LEN, start, count) + ",";
    json += "\"yay\":"    + traceArrayToJson(trace_yay, TRACE_LEN, start, count) + ",";
    json += "\"yaccel\":" + traceArrayToJson(trace_yaccel, TRACE_LEN, start, count) + ",";
    json += "\"ygyro\":"  + traceArrayToJson(trace_ygyro, TRACE_LEN, start, count) + ",";
    json += "\"yangle\":" + traceArrayToJson(trace_yangle, TRACE_LEN, start, count) + ",";
    json += "\"yservo\":" + traceArrayToJson(trace_yservo, TRACE_LEN, start, count);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();

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

  // Confirmed-working blend: pitch 0.65/0.35, yaw 0.9/0.1.
  pitchAngle = 0.65 * (pitchAngle + gyroPitchRate * dt) + 0.35 * accelPitch;
  yawAngle   = 0.9  * (yawAngle   + gyroYawRate   * dt) + 0.1  * accelYaw;

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

  telemetry_pitch = pitchAngle;
  telemetry_yaw = yawAngle;
  telemetry_pitchServo = pitchServoAngle;
  telemetry_yawServo = yawServoAngle;

  // Simple explicit recording: write every sample while recording is
  // true, no threshold detection. Start/Stop are controlled directly
  // from the dashboard (or can be closed entirely during the test -
  // recording is a pure RAM write, no WiFi needed while it runs). The
  // one-time file write happens in /stop_recording, after recording
  // has already ended, so it never disturbs the capture itself.
  if (recording) {
    trace_millis[traceIndex] = now;
    trace_paz[traceIndex] = az;
    trace_paccel[traceIndex] = accelPitch;
    trace_pgyro[traceIndex] = gyroPitchRate;
    trace_pangle[traceIndex] = pitchAngle;
    trace_pservo[traceIndex] = pitchServoAngle;
    trace_yay[traceIndex] = ay;
    trace_yaccel[traceIndex] = accelYaw;
    trace_ygyro[traceIndex] = gyroYawRate;
    trace_yangle[traceIndex] = yawAngle;
    trace_yservo[traceIndex] = yawServoAngle;

    traceIndex = (traceIndex + 1) % TRACE_LEN;
    if (traceIndex == 0) traceFull = true;
    if (traceIndex == 0 && traceFull) {
      // Buffer wrapped and filled again - stop automatically so old
      // data isn't silently overwritten without the user noticing.
      recording = false;
      writeCaptureToFile();
      Serial.println("Buffer full - recording auto-stopped and saved.");
    }
  }

  Serial.print(millis());
  Serial.print("  Pitch: "); Serial.print(pitchAngle);
  Serial.print("  servo: "); Serial.print(pitchServoAngle);
  Serial.print(" || Yaw: "); Serial.print(yawAngle);
  Serial.print("  servo: "); Serial.println(yawServoAngle);

  delay(20);
}