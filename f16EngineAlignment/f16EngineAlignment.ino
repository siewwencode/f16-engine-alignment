#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS & SERIAL CONFIGURATION
// ============================================================================
// GPIO 32 and 33 avoid ESP32-WROVER PSRAM line conflicts (Pins 16/17)
#define RXD2 32 // Connect to WT61C TX
#define TXD2 33 // Connect to WT61C RX

HardwareSerial SensorSerial(2);

// ============================================================================
// 1b. WIFI / WEB UI CONFIGURATION
// ============================================================================
const char *WIFI_SSID = "F16-Alignment";      
const char *WIFI_PASSWORD = "12345678"; 

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>F16 Engine Alignment - Movement Monitor</title>
<style>
  body { font-family: Arial, sans-serif; background:#111; color:#eee; text-align:center; margin:0; padding:2rem; }
  h1 { font-size:1.3rem; font-weight:normal; color:#aaa; }
  #status { width:220px; height:220px; border-radius:50%; margin:2rem auto; display:flex;
            align-items:center; justify-content:center; font-size:1.6rem; font-weight:bold;
            transition: background-color 0.2s ease; box-shadow:0 0 25px rgba(0,0,0,0.6); }
  .moving { background:#c0392b; }
  .stationary { background:#27ae60; }
  table { margin:1.5rem auto; border-collapse:collapse; }
  td { padding:0.3rem 0.8rem; text-align:left; }
  td.label { color:#888; }
  #ip { color:#666; font-size:0.8rem; margin-top:2rem; }
  #resetBtn { background:#2c3e50; color:#eee; border:none; padding:0.7rem 1.6rem;
              border-radius:6px; font-size:1rem; cursor:pointer; }
  #resetBtn:active { background:#1a242f; }
</style>
</head>
<body>
  <h1>Engine Alignment Movement Monitor</h1>
  <div id="status" class="stationary">STATIONARY</div>
  <table>
    <tr><td class="label">Offset (with gravity)</td><td id="posWithGravity">0.00 cm</td></tr>
    <tr><td class="label">Total Travel (with gravity)</td><td id="distWithGravity">0.00 cm</td></tr>
    <tr><td class="label">Offset (without gravity)</td><td id="posWithoutGravity">0.00 cm</td></tr>
    <tr><td class="label">Total Travel (without gravity)</td><td id="distWithoutGravity">0.00 cm</td></tr>
    <tr><td class="label">X Acceleration (with gravity)</td><td id="accelWithGravity">0.00 m/s²</td></tr>
    <tr><td class="label">X Acceleration (without gravity)</td><td id="accelWithoutGravity">0.00 m/s²</td></tr>
  </table>
  <button id="resetBtn" onclick="resetOrigin()">Reset Origin</button>
  <div id="ip"></div>
<script>
async function resetOrigin() {
  try {
    await fetch('/reset', { method: 'POST' });
  } catch (e) {
    // Ignore transient network errors
  }
}
async function poll() {
  try {
    const res = await fetch('/status');
    const data = await res.json();
    const el = document.getElementById('status');
    if (data.moving) {
      el.textContent = 'MOVING';
      el.className = 'moving';
    } else {
      el.textContent = 'STATIONARY';
      el.className = 'stationary';
    }
    document.getElementById('posWithGravity').textContent = data.position_with_gravity_cm.toFixed(2) + ' cm';
    document.getElementById('distWithGravity').textContent = data.distance_with_gravity_cm.toFixed(2) + ' cm';
    document.getElementById('posWithoutGravity').textContent = data.position_without_gravity_cm.toFixed(2) + ' cm';
    document.getElementById('distWithoutGravity').textContent = data.distance_without_gravity_cm.toFixed(2) + ' cm';
    document.getElementById('accelWithGravity').textContent = data.acceleration_with_gravity_mps2.toFixed(3) + ' m/s²';
    document.getElementById('accelWithoutGravity').textContent = data.acceleration_without_gravity_mps2.toFixed(3) + ' m/s²';
  } catch (e) {
    // Ignore transient network errors between polls
  }
}
setInterval(poll, 100);
poll();
</script>
</body>
</html>
)HTML";

// ============================================================================
// 2. SENSOR & KINEMATICS VARIABLES
// ============================================================================
float ax_raw = 0.0;             // Raw X acceleration from sensor in 'g'
float ay_raw = 0.0;             // Raw Y acceleration from sensor in 'g'
float az_raw = 0.0;             // Raw Z acceleration from sensor in 'g'
float ax_baseline_mps2 = 0.0;   // Static corrected X acceleration at origin
float gx_dps = 0.0;              // Angular velocity around X
float gy_dps = 0.0;              // Angular velocity around Y
float gz_dps = 0.0;              // Angular velocity around Z
float roll_deg = 0.0;
float pitch_deg = 0.0;
float yaw_deg = 0.0;
bool hasAcceleration = false;
bool hasAngles = false;
bool isCalibrated = false;

float linear_ax = 0.0;          // Gravity-compensated X acceleration (m/s^2)
float linear_ax_with_gravity = 0.0; // X acceleration with gravity retained (m/s^2)
float velocity_x = 0.0;         // Linear velocity (m/s)
float position_x = 0.0;         // Signed displacement from virtual origin (m)
float distance_x = 0.0;         // Total distance traveled along world X (m)
float velocity_x_with_gravity = 0.0;
float position_x_with_gravity = 0.0;
float distance_x_with_gravity = 0.0;
bool stationary = true;

// High-speed timing variables
unsigned long lastTime_us = 0;
unsigned long lastPrint = 0;

// ============================================================================
// 3. DRIFT SUPPRESSION THRESHOLDS
// ============================================================================
// Deadband threshold: ignores small noise fluctuations around the origin
const float ACCEL_THRESHOLD = 0.100; // Integrate each X stream only above this value (m/s^2)
const float DAMPING_FACTOR = 0.95; // Dampens velocity to reduce integration runaway
const float GRAVITY = 9.80665;     // m/s^2

// ============================================================================
// 4. PARSE WT61C ACCELERATION PACKET (Type 0x51)
// ============================================================================
void readWT61CPackets() {
  while (SensorSerial.available() >= 11) {
    // Check packet header
    if (SensorSerial.peek() != 0x55) {
      SensorSerial.read(); // Discard unaligned byte
      continue;
    }

    uint8_t buf[11];
    SensorSerial.readBytes(buf, 11);

    // Validate checksum (sum of bytes 0 to 9)
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += buf[i];
    }
    if (sum != buf[10]) continue;

    // Process Type 0x51: Acceleration Packet
    if (buf[1] == 0x51) {
      int16_t raw_ax = (int16_t)((buf[3] << 8) | buf[2]);

      ax_raw = (float)raw_ax / 32768.0 * 16.0;
      int16_t raw_ay = (int16_t)((buf[5] << 8) | buf[4]);
      ay_raw = (float)raw_ay / 32768.0 * 16.0;
      int16_t raw_az = (int16_t)((buf[7] << 8) | buf[6]);
      az_raw = (float)raw_az / 32768.0 * 16.0;
      hasAcceleration = true;
    }

    // Process Type 0x52: Angular velocity around X, Y, and Z
    if (buf[1] == 0x52) {
      int16_t raw_gx = (int16_t)((buf[3] << 8) | buf[2]);
      int16_t raw_gy = (int16_t)((buf[5] << 8) | buf[4]);
      int16_t raw_gz = (int16_t)((buf[7] << 8) | buf[6]);
      gx_dps = (float)raw_gx / 32768.0 * 2000.0;
      gy_dps = (float)raw_gy / 32768.0 * 2000.0;
      gz_dps = (float)raw_gz / 32768.0 * 2000.0;
    }

    // Process Type 0x53: Roll, pitch, and yaw angles
    if (buf[1] == 0x53) {
      int16_t raw_roll = (int16_t)((buf[3] << 8) | buf[2]);
      int16_t raw_pitch = (int16_t)((buf[5] << 8) | buf[4]);
      int16_t raw_yaw = (int16_t)((buf[7] << 8) | buf[6]);
      roll_deg = (float)raw_roll / 32768.0 * 180.0;
      pitch_deg = (float)raw_pitch / 32768.0 * 180.0;
      yaw_deg = (float)raw_yaw / 32768.0 * 180.0;
      hasAngles = true;
    }
  }
}

// ============================================================================
// 5. ORIGIN TARE / RESET FUNCTION
// ============================================================================
/**
 * Sets the current resting point as the virtual origin (0, 0, 0),
 * and resets velocity and X displacement.
 */
void resetOrigin() {
  // Capture the current gravity-corrected reading as the zero baseline.
  float pitch_rad = pitch_deg * PI / 180.0;
  float gravity_ax = -GRAVITY * sinf(pitch_rad);
  ax_baseline_mps2 = ax_raw * GRAVITY - gravity_ax;

  velocity_x = 0.0;
  position_x = 0.0;
  distance_x = 0.0;
  velocity_x_with_gravity = 0.0;
  position_x_with_gravity = 0.0;
  distance_x_with_gravity = 0.0;
  isCalibrated = true;

  Serial.println("\n=======================================================");
  Serial.printf(">>> [ORIGIN SET] Position: (0, 0, 0) | X baseline: %+5.2f m/s² <<<\n", ax_baseline_mps2);
  Serial.println("=======================================================\n");
}

// ============================================================================
// 6. WEB UI REQUEST HANDLERS
// ============================================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String json = "{";
  json += "\"moving\":" + String(stationary ? "false" : "true") + ",";
  json += "\"position_with_gravity_cm\":" + String(position_x_with_gravity * 100.0, 2) + ",";
  json += "\"distance_with_gravity_cm\":" + String(distance_x_with_gravity * 100.0, 2) + ",";
  json += "\"position_without_gravity_cm\":" + String(position_x * 100.0, 2) + ",";
  json += "\"distance_without_gravity_cm\":" + String(distance_x * 100.0, 2) + ",";
  json += "\"acceleration_with_gravity_mps2\":" + String(linear_ax_with_gravity, 3) + ",";
  json += "\"acceleration_without_gravity_mps2\":" + String(linear_ax, 3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleReset() {
  resetOrigin();
  server.send(200, "text/plain", "OK");
}

// ============================================================================
// 7. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  SensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  lastTime_us = micros();

  Serial.println("\n--- WT61C Origin Distance Tracker ---");
  Serial.println("Output: X acceleration, displacement, and distance with and without gravity");
  Serial.println("Type '0' or 'r' in Serial Monitor to set the current point as origin (0,0,0).\n");

  // ESP32 broadcasts its own hotspot; connect your phone to this network directly.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Access Point \"%s\" started. Open http://%s in a browser to view movement status.\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/reset", HTTP_POST, handleReset);
  server.begin();
}

// ============================================================================
// 8. MAIN LOOP
// ============================================================================
void loop() {
  // Service pending web UI requests
  server.handleClient();

  // Check for reset command from user
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '0' || c == 'r' || c == 'R') {
      resetOrigin();
    }
  }

  // Read latest sensor data
  readWT61CPackets();

  // Compute loop delta time (dt) in seconds
  unsigned long now_us = micros();
  float dt = (float)(now_us - lastTime_us) / 1000000.0;
  lastTime_us = now_us;

  // Convert raw 'g' to m/s²
  float current_ax_mps2 = ax_raw * GRAVITY;

  // Auto-calibrate on first valid incoming packet if not yet initialized
  if (!isCalibrated && hasAcceleration && hasAngles) {
    resetOrigin();
  }

  // --------------------------------------------------------------------------
  // Kinematics Calculations relative to Origin
  // --------------------------------------------------------------------------
  if (isCalibrated) {
    // Remove the gravity component using roll and pitch, then subtract tare bias.
    float pitch_rad = pitch_deg * PI / 180.0;
    float gravity_ax = -GRAVITY * sinf(pitch_rad);
    linear_ax_with_gravity = current_ax_mps2 - ax_baseline_mps2;
    linear_ax = (current_ax_mps2 - gravity_ax) - ax_baseline_mps2;

    // Integrate each X-axis stream independently only above the threshold.
    bool withoutGravityActive = abs(linear_ax) > ACCEL_THRESHOLD;
    bool withGravityActive = abs(linear_ax_with_gravity) > ACCEL_THRESHOLD;

    if (withoutGravityActive) {
      velocity_x += linear_ax * dt;
      position_x += velocity_x * dt;
      distance_x += abs(velocity_x * dt);
    } else {
      velocity_x = 0.0;
    }

    if (withGravityActive) {
      velocity_x_with_gravity += linear_ax_with_gravity * dt;
      position_x_with_gravity += velocity_x_with_gravity * dt;
      distance_x_with_gravity += abs(velocity_x_with_gravity * dt);
    } else {
      velocity_x_with_gravity = 0.0;
    }

    stationary = !withoutGravityActive && !withGravityActive;
  }

  // --------------------------------------------------------------------------
  // Telemetry Output (Every 100 ms)
  // --------------------------------------------------------------------------
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
            Serial.printf("X no gravity: %+.3f m/s^2 | Displacement: %+.2f cm | Distance: %.2f cm | X with gravity: %+.3f m/s^2 | Displacement: %+.2f cm | Distance: %.2f cm\n",
              linear_ax, position_x * 100.0, distance_x * 100.0,
              linear_ax_with_gravity, position_x_with_gravity * 100.0,
              distance_x_with_gravity * 100.0);
  }
}