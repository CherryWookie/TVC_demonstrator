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
#define GIMBAL_MAX_SAFE 20.0

#define FAN_PIN 17
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8
#define FAN_SPEED 200

int angleToPulse(float angle) {
  return (int)((angle) * (SERVO_MAX - SERVO_MIN) / 180.0 + SERVO_MIN);
}

MPU6050 mpu(Wire);


#define ACCEL_X_AVG -0.9616
#define ACCEL_Y_AVG -0.0555
#define ACCEL_Z_AVG -0.0584
#define GYRO_X_AVG 0.5768
#define GYRO_Y_AVG 1.3094
#define GYRO_Z_AVG -0.6862

// --- Accelerometer offsets (units: g) - FINAL, real mounted position ---
// Note: -X is the up axis here, so X_OFFSET = ax_avg - (-1.0), not ax_avg - 1.0
#define X_OFFSET (ACCEL_X_AVG + 1.0)
#define Y_OFFSET (ACCEL_Y_AVG)
#define Z_OFFSET (ACCEL_Z_AVG)

// --- Gyroscope zero-rate offsets (units: deg/s) ---
#define GYRO_X_OFFSET (GYRO_X_AVG)
#define GYRO_Y_OFFSET (GYRO_Y_AVG)
#define GYRO_Z_OFFSET (GYRO_Z_AVG)



float pitchAngle = 0;
float yawAngle = 0;
unsigned long lastTime = 0;

// TEST CONFIG: pitch and yaw made structurally identical for direct
// comparison. Gain MAGNITUDES matched exactly (2.2, 16, 0.0063).
// Pitch kept POSITIVE (its confirmed-correct sign via repeated push
// tests); yaw kept NEGATIVE (its own confirmed-correct sign). This is
// the one intentional difference - everything else, including the
// filter blend logic below, is now identical between the two axes.
float Kp_pitch = 2.2;
float Ki_pitch = 16;
float Kd_pitch = 0.0063;

float Kp_yaw = -2.2;
float Ki_yaw = -16;
float Kd_yaw = 0.0063;

float pitchIntegral = 0, pitchLastError = 0;
float yawIntegral = 0, yawLastError = 0;
bool pitchSaturated = false, yawSaturated = false;

#define INTEGRAL_LIMIT 80.0

// Soft deadband: below this error magnitude, contribute nothing (immune
// to sensor noise, same as before). Above it, the error ramps up
// CONTINUOUSLY from zero rather than jumping straight to its full raw
// value - this removes the discontinuity a hard on/off gate creates
// right at the threshold, which is what was causing the jitter loop.
#define PITCH_DEADBAND 0.6
#define YAW_DEADBAND 0.6

float applyDeadband(float error, float deadband) {
  if (fabs(error) <= deadband) return 0.0;
  return error - deadband * (error > 0 ? 1.0 : -1.0);
}

float telemetry_pitch = 0, telemetry_yaw = 0;
float telemetry_pitchCorrection = 0, telemetry_yawCorrection = 0;
float telemetry_pitchServo = 90, telemetry_yawServo = 90;

// --- Oscilloscope trace buffer ---
// Captures at full loop rate (~50Hz) so noise characteristics are
// actually visible, rather than limited by how fast the browser polls.
// The webpage fetches the whole recent window each time, not just the
// latest sample.
#define TRACE_LEN 150   // ~3 seconds of history at 50Hz
float trace_az[TRACE_LEN];
float trace_azCorr[TRACE_LEN];
float trace_tang[TRACE_LEN];
float trace_alpha[TRACE_LEN];
float trace_gyroRate[TRACE_LEN];
float trace_pitchAngle[TRACE_LEN];
float trace_dt[TRACE_LEN];
float trace_ax[TRACE_LEN];
float trace_ay[TRACE_LEN];
int traceIndex = 0;
bool traceFull = false;

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
    #scope { width: 100%; height: 720px; background: #111; border-radius: 6px; display: block; margin-top: 8px; }
    #scopeControls { display: flex; gap: 10px; align-items: center; margin-top: 8px; }
    #scopeControls button { width: auto; padding: 8px 16px; margin-top: 0; }
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

  <h2>Oscilloscope (pitch diagnostics, ~3s window, full loop-rate resolution)</h2>
  <canvas id="scope" width="740" height="720"></canvas>
  <div id="scopeControls">
    <button onclick="scopePaused = !scopePaused">Pause / Resume</button>
    <span id="scopeStatus">running</span>
  </div>

  <h2>Record &amp; Export</h2>
  <div id="scopeControls">
    <button id="recordBtn" onclick="toggleRecording()">Start Recording</button>
    <span id="recordStatus">not recording, 0 samples</span>
  </div>
  <div id="scopeControls">
    <button onclick="prepareExport()">Prepare Export</button>
    <button onclick="downloadExport()">Download JSON File</button>
    <button onclick="clearRecording()">Clear</button>
  </div>
  <label>Exported JSON (click box, Ctrl+A, Ctrl+C to copy and paste into chat)</label>
  <textarea id="exportBox" readonly rows="10" style="width:100%; font-family: monospace; font-size: 11px;" onclick="this.select()"></textarea>

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
    var isRecording = false;
    var recordedLog = [];
    const scopeChannels = [
      { key: 'az',         label: 'az (raw, g)',          color: '#e74c3c' },
      { key: 'ax',         label: 'ax (raw, g)',          color: '#c0392b' },
      { key: 'ay',         label: 'ay (raw, g)',          color: '#d35400' },
      { key: 'azCorr',      label: 'az (corrected, g)',    color: '#3498db' },
      { key: 'tang',        label: 'tangential term (g)',  color: '#2ecc71' },
      { key: 'alpha',       label: 'alpha (rad/s^2)',      color: '#f39c12' },
      { key: 'gyroRate',     label: 'gyro rate (deg/s)',   color: '#e67e22' },
      { key: 'pitchAngle',  label: 'pitch angle (deg)',    color: '#9b59b6' },
      { key: 'dt',           label: 'dt (s) - watch for spikes', color: '#ff0000' },
    ];

    function toggleRecording() {
      isRecording = !isRecording;
      document.getElementById('recordBtn').textContent = isRecording ? 'Stop Recording' : 'Start Recording';
      updateRecordStatus();
    }
    function updateRecordStatus() {
      document.getElementById('recordStatus').textContent =
        (isRecording ? 'recording' : 'not recording') + ', ' + recordedLog.length + ' samples';
    }
    function clearRecording() {
      recordedLog = [];
      updateRecordStatus();
      document.getElementById('exportBox').value = '';
    }
    function prepareExport() {
      document.getElementById('exportBox').value = JSON.stringify(recordedLog, null, 2);
    }
    function downloadExport() {
      const blob = new Blob([JSON.stringify(recordedLog, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'tvc_trace_' + Date.now() + '.json';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }

    function drawScope(data) {
      const canvas = document.getElementById('scope');
      const ctx = canvas.getContext('2d');
      const W = canvas.width, H = canvas.height;
      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, W, H);
      const rowH = H / scopeChannels.length;

      scopeChannels.forEach((ch, i) => {
        const arr = data[ch.key];
        const yTop = i * rowH;

        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        ctx.strokeRect(0, yTop, W, rowH);

        if (!arr || arr.length < 2) return;

        let minV = Math.min(...arr);
        let maxV = Math.max(...arr);
        if (minV === maxV) { minV -= 1; maxV += 1; }
        const range = maxV - minV;
        const pad = range * 0.1;
        minV -= pad; maxV += pad;
        const fullRange = maxV - minV;

        // zero line, if in range
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
        arr.forEach((v, idx) => {
          const x = (idx / (arr.length - 1)) * W;
          const y = yTop + rowH - ((v - minV) / fullRange) * rowH;
          if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();

        const last = arr[arr.length - 1];
        ctx.fillStyle = '#eee';
        ctx.font = '11px monospace';
        ctx.fillText(
          ch.label + '   min:' + minV.toFixed(3) + '  max:' + maxV.toFixed(3) + '  last:' + last.toFixed(3),
          6, yTop + 13
        );
      });
    }

    async function pollScope() {
      if (!scopePaused) {
        try {
          const r = await fetch('/trace');
          const d = await r.json();
          drawScope(d);
          document.getElementById('scopeStatus').textContent = 'running';

          if (isRecording) {
            const n = d.pitchAngle ? d.pitchAngle.length : 0;
            if (n > 0) {
              recordedLog.push({
                t: Date.now(),
                az: d.az[n - 1],
                ax: d.ax[n - 1],
                ay: d.ay[n - 1],
                azCorr: d.azCorr[n - 1],
                tang: d.tang[n - 1],
                alpha: d.alpha[n - 1],
                gyroRate: d.gyroRate[n - 1],
                pitchAngle: d.pitchAngle[n - 1],
                dt: d.dt[n - 1]
              });
              updateRecordStatus();
            }
          }
        } catch (e) {}
      } else {
        document.getElementById('scopeStatus').textContent = 'paused';
      }
      setTimeout(pollScope, 150);
    }

    loadGains();
    pollTelemetry();
    pollScope();
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Serial.println("=== TVC Gimbal PID + WiFi dashboard (build: MAIN-4, pitch/yaw structurally matched) ===");

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
  // FINAL mapping: -X = up, Y = pitch axis, Z = yaw axis
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
                  ",\"pitchCorrection\":" + String(telemetry_pitchCorrection) +
                  ",\"yawCorrection\":" + String(telemetry_yawCorrection) +
                  ",\"pitchServo\":" + String(telemetry_pitchServo) +
                  ",\"yawServo\":" + String(telemetry_yawServo) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/trace", HTTP_GET, [](AsyncWebServerRequest *request) {
    int start = traceFull ? traceIndex : 0;
    int count = traceFull ? TRACE_LEN : traceIndex;
    String json = "{";
    json += "\"az\":" + traceArrayToJson(trace_az, TRACE_LEN, start, count) + ",";
    json += "\"azCorr\":" + traceArrayToJson(trace_azCorr, TRACE_LEN, start, count) + ",";
    json += "\"tang\":" + traceArrayToJson(trace_tang, TRACE_LEN, start, count) + ",";
    json += "\"alpha\":" + traceArrayToJson(trace_alpha, TRACE_LEN, start, count) + ",";
    json += "\"gyroRate\":" + traceArrayToJson(trace_gyroRate, TRACE_LEN, start, count) + ",";
    json += "\"pitchAngle\":" + traceArrayToJson(trace_pitchAngle, TRACE_LEN, start, count) + ",";
    json += "\"dt\":" + traceArrayToJson(trace_dt, TRACE_LEN, start, count) + ",";
    json += "\"ax\":" + traceArrayToJson(trace_ax, TRACE_LEN, start, count) + ",";
    json += "\"ay\":" + traceArrayToJson(trace_ay, TRACE_LEN, start, count);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();

  Serial.println("Setup complete. Stabilization active.");
  delay(1000);
  lastTime = millis();
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

void loop() {
  mpu.fetchData();

  float ax = mpu.getAccX() - X_OFFSET;
  float ay = mpu.getAccY() - Y_OFFSET;
  float az = mpu.getAccZ() - Z_OFFSET;

  // FINAL mapping: -X = up, Y = pitch axis, Z = yaw axis
  float accelPitch = atan2(az, sqrt(ax * ax + ay * ay)) * 180.0 / PI;
  float accelYaw   = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroPitchRate = mpu.getGyroY() - GYRO_Y_OFFSET;
  float gyroYawRate   = mpu.getGyroZ() - GYRO_Z_OFFSET;

  // --- Motion gate --- units are g, compare against 1.0
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);

  // Centripetal acceleration correction: as the assembly rotates, the
  // IMU (mounted off the actual pivot point) experiences real
  // centripetal acceleration = l * omega^2, always directed from the
  // IMU toward the pivot. This is indistinguishable from gravity to
  // the accelerometer, and is the root cause of the tilt-speed-
  // proportional backtrack. Rather than heuristically discounting
  // accel trust, compute the predicted error directly and subtract it
  // before the angle is ever calculated.
  //
  // Real measured distances: IMU sits 0.1674m from the pitch pivot
  // (substantial lever arm), but essentially AT the yaw pivot (chip is
  // mounted on the tube's side, centered directly over the pivot
  // point for yaw). Squared in the centripetal formula below, this is
  // the actual physical reason pitch shows tilt-speed-proportional
  // backtrack and yaw doesn't - not a code asymmetry.
  #define L_PIVOT_TO_IMU_PITCH 0.1674  // meters, inner tube pivot -> IMU
  #define L_PIVOT_TO_IMU_YAW   0.0     // meters, chip mounted at the yaw pivot itself
  #define G_MS2 9.81

  float omegaPitchRad = gyroPitchRate * PI / 180.0;
  float omegaYawRad   = gyroYawRate   * PI / 180.0;

  // Tangential acceleration term: centripetal (above) depends on
  // angular VELOCITY squared, and turned out too small at realistic
  // hand-tilt speeds to explain the observed error. Tangential
  // acceleration depends on angular ACCELERATION instead (l * alpha),
  // and peaks exactly at the START of a motion - matching the actual
  // reported timing of the backtrack far better than centripetal did.
  //
  // Raw differentiation of gyro rate massively amplifies sample-to-
  // sample noise (dividing by a small dt blows small wobbles up into
  // a large, noisy alpha estimate). Low-pass the rate signal itself
  // before differencing - this is what was causing jitter DURING
  // correction specifically as Kp increased: the controller reacting
  // to noise injected by this term, not a real disturbance.
  static float smoothedGyroPitchRate = 0, smoothedGyroYawRate = 0;
  static float lastSmoothedGyroPitchRate = 0, lastSmoothedGyroYawRate = 0;
  #define RATE_SMOOTHING 0.3

  smoothedGyroPitchRate = RATE_SMOOTHING * smoothedGyroPitchRate + (1.0 - RATE_SMOOTHING) * gyroPitchRate;
  smoothedGyroYawRate   = RATE_SMOOTHING * smoothedGyroYawRate   + (1.0 - RATE_SMOOTHING) * gyroYawRate;

  float alphaPitchRad = ((smoothedGyroPitchRate - lastSmoothedGyroPitchRate) * PI / 180.0) / dt;
  float alphaYawRad   = ((smoothedGyroYawRate   - lastSmoothedGyroYawRate)   * PI / 180.0) / dt;
  lastSmoothedGyroPitchRate = smoothedGyroPitchRate;
  lastSmoothedGyroYawRate   = smoothedGyroYawRate;

  float tangentialPitch_g = (L_PIVOT_TO_IMU_PITCH * alphaPitchRad) / G_MS2;
  float tangentialYaw_g   = (L_PIVOT_TO_IMU_YAW   * alphaYawRad)   / G_MS2;

  // Magnitude only - direction assumed along Z (pitch) / Y (yaw),
  // matching the axis already used in the accelPitch/accelYaw
  // formulas below. Sign is a placeholder - verify empirically: if
  // the backtrack gets WORSE after flashing this, flip the sign here.
  float centripetalPitch_g = (L_PIVOT_TO_IMU_PITCH * omegaPitchRad * omegaPitchRad) / G_MS2;
  float centripetalYaw_g   = (L_PIVOT_TO_IMU_YAW   * omegaYawRad   * omegaYawRad)   / G_MS2;

  // Axis assignment, using the established mounting convention
  // (-X = up, tube stands along X, pivot-to-IMU offset runs along X):
  //   - Centripetal (points along the radius, IMU toward pivot) -> X
  //   - Tangential (perpendicular to rotation axis Y and radius X,
  //     i.e. Y x X = -Z) -> Z
  float axCorrected = ax - centripetalPitch_g;
  float azCorrected = az - tangentialPitch_g;
  float ayCorrected = ay - centripetalYaw_g - tangentialYaw_g;

  float accelPitchCorrected = atan2(azCorrected, sqrt(axCorrected * axCorrected + ay * ay)) * 180.0 / PI;
  float accelYawCorrected   = atan2(ayCorrected, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // Dynamic accel blend weight, gated by ALPHA (angular acceleration)
  // rather than rate. Rate starts near zero and ramps up, so gating on
  // rate alone doesn't suppress accel trust until AFTER the flick has
  // already happened. Alpha peaks specifically at the ONSET of motion -
  // exactly where the wrong-direction flick occurs - so gating on it
  // directly targets the actual problem window instead of reacting a
  // beat late.
  #define ACCEL_WEIGHT_REST 0.12   // at rest / steady motion: strong drift correction
  #define ACCEL_WEIGHT_ONSET 0.02  // during a motion-onset transient: mostly gyro
  #define ALPHA_GATE_LOW  1.0      // rad/s^2, full rest weight below this
  #define ALPHA_GATE_HIGH 6.0      // rad/s^2, full onset weight at/above this

  float alphaMag = fabs(alphaPitchRad);
  float pitchAlphaScale = 1.0 - (alphaMag - ALPHA_GATE_LOW) / (ALPHA_GATE_HIGH - ALPHA_GATE_LOW);
  pitchAlphaScale = constrain(pitchAlphaScale, 0.0, 1.0);
  float pitchAccelWeight = ACCEL_WEIGHT_ONSET + (ACCEL_WEIGHT_REST - ACCEL_WEIGHT_ONSET) * pitchAlphaScale;

  pitchAngle = (1.0 - pitchAccelWeight) * (pitchAngle + gyroPitchRate * dt) + pitchAccelWeight * accelPitchCorrected;
  yawAngle   = 0.9 * (yawAngle   + gyroYawRate   * dt) + 0.1 * accelYawCorrected;

  float pitchCorrection = computePID(0, pitchAngle, pitchIntegral,
                                      pitchLastError, dt,
                                      Kp_pitch, Ki_pitch, Kd_pitch, pitchSaturated,
                                      PITCH_DEADBAND);
  float yawCorrection = computePID(0, yawAngle, yawIntegral,
                                    yawLastError, dt,
                                    Kp_yaw, Ki_yaw, Kd_yaw, yawSaturated,
                                    YAW_DEADBAND);

  // Determine saturation BEFORE clamping, so next loop's anti-windup
  // check reflects whether we actually hit the limit this time.
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
  telemetry_pitchCorrection = pitchCorrection;
  telemetry_yawCorrection = yawCorrection;
  telemetry_pitchServo = pitchServoAngle;
  telemetry_yawServo = yawServoAngle;

  trace_az[traceIndex] = az;
  trace_azCorr[traceIndex] = azCorrected;
  trace_tang[traceIndex] = tangentialPitch_g;
  trace_alpha[traceIndex] = alphaPitchRad;
  trace_gyroRate[traceIndex] = gyroPitchRate;
  trace_pitchAngle[traceIndex] = pitchAngle;
  trace_dt[traceIndex] = dt;
  trace_ax[traceIndex] = ax;
  trace_ay[traceIndex] = ay;
  traceIndex = (traceIndex + 1) % TRACE_LEN;
  if (traceIndex == 0) traceFull = true;

  Serial.print("Pitch: "); Serial.print(pitchAngle);
  Serial.print("  az: "); Serial.print(az, 3);
  Serial.print("  azCorr: "); Serial.print(azCorrected, 3);
  Serial.print("  tangPitch_g: "); Serial.print(tangentialPitch_g, 4);
  Serial.print("  alphaPitch: "); Serial.print(alphaPitchRad, 3);
  Serial.print("  servo: "); Serial.print(pitchServoAngle);
  Serial.print("  pitchIntegral: "); Serial.print(pitchIntegral, 4);
  Serial.print(" || Yaw: "); Serial.print(yawAngle);
  Serial.print("  servo: "); Serial.print(yawServoAngle);
  Serial.print("  yawIntegral: "); Serial.println(yawIntegral, 4);

  delay(20);
}