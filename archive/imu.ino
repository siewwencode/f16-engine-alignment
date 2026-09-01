#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi Access Point Credentials
const char* ap_ssid = "ESP32_Distance_Tracker";
const char* ap_pass = "12345678"; // Min 8 characters

// WT16C-TTL Pin Connections on ESP32-WROVER
#define RXD2 32 // Connect to WT16C TX
#define TXD2 33 // Connect to WT16C RX

HardwareSerial SensorSerial(2);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Linear X-Axis Tracking Variables
int rawDistance_mm = 0;
int lastValidRaw_mm = -1;
int zeroReference_mm = 0;          // Tare point for X = 0 mm
int posX_mm = 0;                   // Current X-axis position (+/- displacement)
float totalTraveled_m = 0.0;       // Total distance moved (odometer)
unsigned long lastBroadcast = 0;

// Filter parameter (moving average to eliminate acoustic jitter)
float filteredDistance_mm = 0.0;
const float filterAlpha = 0.3; // Lower = smoother, Higher = more responsive

// Mobile Dashboard UI
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>X-Axis Linear Tracker</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0b0f19; color: #f8fafc; margin: 0; padding: 16px; display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 90vh; }
    .card { background: #1e293b; border-radius: 20px; padding: 24px; width: 90%; max-width: 360px; box-shadow: 0 12px 30px rgba(0,0,0,0.5); text-align: center; border: 1px solid #334155; }
    .title { font-size: 0.8rem; text-transform: uppercase; letter-spacing: 1.5px; color: #94a3b8; margin-bottom: 4px; }
    .metric { font-size: 3rem; font-weight: 800; color: #38bdf8; margin: 4px 0 16px 0; font-variant-numeric: tabular-nums; }
    .unit { font-size: 1.1rem; color: #64748b; font-weight: 500; margin-left: 4px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px; }
    .sub-card { background: #0f172a; padding: 12px; border-radius: 12px; border: 1px solid #1e293b; }
    .sub-metric { font-size: 1.3rem; font-weight: 700; color: #e2e8f0; margin-top: 4px; font-variant-numeric: tabular-nums; }
    .btn { background: #ef4444; border: none; color: white; padding: 14px; font-size: 1rem; font-weight: 700; border-radius: 12px; margin-top: 18px; cursor: pointer; width: 100%; transition: transform 0.1s, background 0.2s; }
    .btn:active { background: #dc2626; transform: scale(0.97); }
    .status { font-size: 0.75rem; margin-top: 14px; color: #22c55e; font-weight: 500; }
  </style>
</head>
<body>
  <div class="card">
    <div class="title">X-Axis Position (Displacement)</div>
    <div class="metric"><span id="posX">0</span><span class="unit">mm</span></div>
    
    <div class="grid">
      <div class="sub-card">
        <div class="title" style="font-size: 0.7rem;">Total Traveled</div>
        <div class="sub-metric"><span id="traveled">0.00</span> <span style="font-size: 0.8rem; color: #94a3b8;">m</span></div>
      </div>
      <div class="sub-card">
        <div class="title" style="font-size: 0.7rem;">Raw Distance</div>
        <div class="sub-metric"><span id="raw">0</span> <span style="font-size: 0.8rem; color: #94a3b8;">mm</span></div>
      </div>
    </div>

    <button class="btn" onclick="resetOrigin()">Zero Origin (X = 0)</button>
    <div id="status" class="status">Connecting...</div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = () => { 
        document.getElementById('status').innerText = 'Live Feed Connected (10 Hz)'; 
        document.getElementById('status').style.color = '#22c55e'; 
      };
      websocket.onclose = () => { 
        document.getElementById('status').innerText = 'Reconnecting...'; 
        document.getElementById('status').style.color = '#eab308'; 
        setTimeout(initWebSocket, 2000); 
      };
      websocket.onmessage = (event) => {
        let data = JSON.parse(event.data);
        document.getElementById('posX').innerText = (data.x > 0 ? "+" : "") + data.x;
        document.getElementById('traveled').innerText = data.traveled.toFixed(3);
        document.getElementById('raw').innerText = data.raw;
      };
    }

    function resetOrigin() {
      websocket.send("RESET");
    }

    window.onload = initWebSocket;
  </script>
</body>
</html>
)rawliteral";

// Continuous Frame Parser for WT16C
int readWT16CDistance() {
  while (SensorSerial.available() >= 4) {
    if (SensorSerial.peek() != 0xFF) {
      SensorSerial.read(); // Drop byte until header is found
      continue;
    }

    if (SensorSerial.available() < 4) break;

    uint8_t header   = SensorSerial.read(); // 0xFF
    uint8_t data_h   = SensorSerial.read();
    uint8_t data_l   = SensorSerial.read();
    uint8_t checksum = SensorSerial.read();

    uint8_t calculated_sum = (header + data_h + data_l) & 0xFF;
    if (calculated_sum == checksum) {
      int d = (data_h << 8) | data_l;
      if (d >= 20 && d <= 6000) { // Discard out-of-range sensor readings
        return d;
      }
    }
  }
  return -1;
}

// WebSocket Command Handler
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      if (strcmp((char*)data, "RESET") == 0) {
        zeroReference_mm = (int)filteredDistance_mm;
        totalTraveled_m = 0.0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // WT16C UART configuration (Baud: 115200)
  SensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Start Wi-Fi Access Point
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.println("\n[X-Axis Linear Tracker Initialized]");
  Serial.print("Connect to SSID: "); Serial.println(ap_ssid);
  Serial.print("Open URL: http://"); Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
}

void loop() {
  ws.cleanupClients();

  int dist = readWT16CDistance();
  if (dist > 0) {
    rawDistance_mm = dist;

    // Apply low-pass EMA filter to dampen acoustic reflection noise
    if (filteredDistance_mm == 0.0) {
      filteredDistance_mm = rawDistance_mm;
      zeroReference_mm = rawDistance_mm;
    } else {
      filteredDistance_mm = (filterAlpha * rawDistance_mm) + ((1.0 - filterAlpha) * filteredDistance_mm);
    }

    int currentFiltered_mm = (int)(filteredDistance_mm + 0.5);

    // Calculate displacement relative to Origin (X = 0)
    posX_mm = currentFiltered_mm - zeroReference_mm;

    // Accumulate total distance traveled (odometer)
    if (lastValidRaw_mm > 0) {
      int delta = abs(currentFiltered_mm - lastValidRaw_mm);
      // Deadband: ignore fluctuations under 2 mm
      if (delta >= 2) {
        totalTraveled_m += (delta / 1000.0);
        lastValidRaw_mm = currentFiltered_mm;
      }
    } else {
      lastValidRaw_mm = currentFiltered_mm;
    }
  }

  // Push updates over WebSocket at 10 Hz (100 ms)
  if (millis() - lastBroadcast > 100) {
    lastBroadcast = millis();
    char buffer[96];
    snprintf(buffer, sizeof(buffer), 
             "{\"x\":%d,\"traveled\":%.3f,\"raw\":%d}", 
             posX_mm, totalTraveled_m, rawDistance_mm);
    ws.textAll(buffer);
  }
}