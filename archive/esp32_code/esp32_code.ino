#include <WiFi.h>
#include <WebServer.h>

// --- Access Point Credentials ---
const char* ap_ssid = "WT61C_Sensor";
const char* ap_password = "password123"; // Minimum 8 characters

WebServer server(80);
HardwareSerial SensorSerial(2); // Use ESP32 Hardware Serial 2

// Raw data variables
float rawRoll = 0, rawPitch = 0, rawYaw = 0;
// Offset variables for calibration (Tare)
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;

// WT61C Parsing Buffer
uint8_t buffer[11];
int bufPos = 0;

// --- HTML & JavaScript UI ---
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WT61C Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f9; padding: 20px; margin: 0; }
    /* Hide pages by default, only show the 'active' one */
    .page { display: none; }
    .active { display: block; }
    
    .card { background: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); max-width: 400px; margin: 40px auto; position: relative; }
    .data-row { font-size: 24px; margin: 15px 0; font-weight: bold; color: #333; }
    .label { color: #888; font-size: 18px; font-weight: normal; }
    
    button { background-color: #007BFF; color: white; border: none; padding: 15px 30px; font-size: 18px; border-radius: 5px; cursor: pointer; margin-top: 20px; transition: 0.2s;}
    button:hover { background-color: #0056b3; }
    button:disabled { background-color: #cccccc; cursor: not-allowed; }
    
    /* Small back button at the top left of the card */
    .back-btn { position: absolute; top: 15px; left: 15px; padding: 8px 15px; font-size: 14px; background-color: #6c757d; margin-top: 0; }
    .back-btn:hover { background-color: #5a6268; }
    
    .status-text { margin-top: 20px; font-size: 18px; font-weight: bold; color: #d9534f; height: 25px;}
  </style>
</head>
<body>

  <!-- 1. Landing / Calibration Page -->
  <div id="landing-page" class="page active card">
    <h2>Sensor Setup</h2>
    <p style="color: #666; margin-bottom: 25px;">Place the sensor on a flat surface and ensure it is absolutely still.</p>
    <button id="cal-btn" onclick="startCalibration()">Calibrate Sensor</button>
    <div id="cal-status" class="status-text"></div>
  </div>

  <!-- 2. Live Data Page -->
  <div id="data-page" class="page card">
    <button class="back-btn" onclick="resetToLanding()">&#8592; Back</button>
    <h2 style="margin-top: 40px;">Live Data</h2>
    <div class="data-row"><span class="label">Roll (X): </span><span id="roll">0.00°</span></div>
    <div class="data-row"><span class="label">Pitch (Y): </span><span id="pitch">0.00°</span></div>
    <div class="data-row"><span class="label">Yaw (Z): </span><span id="yaw">0.00°</span></div>
  </div>

  <script>
    let dataInterval; // Holds our loop timer

    function startCalibration() {
      let btn = document.getElementById('cal-btn');
      let status = document.getElementById('cal-status');
      let timeLeft = 10;
      
      // Disable button so user can't click it multiple times
      btn.disabled = true;
      status.innerText = "Calibrating... " + timeLeft + "s";
      
      // Start a 1-second countdown loop
      let countdown = setInterval(function() {
        timeLeft--;
        if (timeLeft > 0) {
          status.innerText = "Calibrating... " + timeLeft + "s";
        } else {
          // Timer finished
          clearInterval(countdown);
          status.innerText = "Setting zero point...";
          
          // Tell ESP32 to grab the current flat values
          fetch('/calibrate')
            .then(response => {
              // Reset UI for next time
              btn.disabled = false;
              status.innerText = "";
              // Switch to the data page
              showDataPage();
            })
            .catch(err => {
              status.innerText = "Error connecting to sensor.";
              btn.disabled = false;
            });
        }
      }, 1000);
    }

    function showDataPage() {
      // Hide landing, show data
      document.getElementById('landing-page').classList.remove('active');
      document.getElementById('data-page').classList.add('active');
      
      // Fetch live data immediately, then set it to loop every 500ms (0.5 seconds)
      fetchData();
      dataInterval = setInterval(fetchData, 500);
    }

    function resetToLanding() {
      // Stop the 0.5s data fetching loop to save battery/bandwidth
      clearInterval(dataInterval);
      
      // Hide data, show landing
      document.getElementById('data-page').classList.remove('active');
      document.getElementById('landing-page').classList.add('active');
    }

    function fetchData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('roll').innerText = data.roll.toFixed(2) + '°';
          document.getElementById('pitch').innerText = data.pitch.toFixed(2) + '°';
          document.getElementById('yaw').innerText = data.yaw.toFixed(2) + '°';
        });
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200); 
  // WROVER PINS: RX = 32, TX = 33
  SensorSerial.begin(115200, SERIAL_8N1, 32, 33);

  // --- Start Access Point ---
  Serial.println("\nStarting Access Point...");
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP Started! Network Name: ");
  Serial.println(ap_ssid);
  Serial.print("Connect to this IP in your browser: ");
  Serial.println(IP); 

  // --- Web Server Routes ---
  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/calibrate", []() {
    // Tare the current angles to 0
    offsetRoll = rawRoll;
    offsetPitch = rawPitch;
    offsetYaw = rawYaw;
    server.send(200, "text/plain", "Calibrated");
  });

  server.on("/data", []() {
    // Subtract the offsets so the flat position reads as 0
    float displayRoll = rawRoll - offsetRoll;
    float displayPitch = rawPitch - offsetPitch;
    float displayYaw = rawYaw - offsetYaw;
    
    String json = "{\"roll\":" + String(displayRoll) + 
                  ",\"pitch\":" + String(displayPitch) + 
                  ",\"yaw\":" + String(displayYaw) + "}";
                  
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();

  // Read data from WT61C
  while (SensorSerial.available()) {
    uint8_t c = SensorSerial.read();
    buffer[bufPos] = c;
    
    if (bufPos == 0 && c != 0x55) continue; 
    
    bufPos++;
    
    if (bufPos == 11) {
      uint8_t sum = 0;
      for(int i = 0; i < 10; i++) sum += buffer[i];
      
      if (sum == buffer[10]) {
        if (buffer[1] == 0x53) {
          short r = (buffer[3] << 8) | buffer[2];
          short p = (buffer[5] << 8) | buffer[4];
          short y = (buffer[7] << 8) | buffer[6];
          
          rawRoll = ((float)r / 32768.0) * 180.0;
          rawPitch = ((float)p / 32768.0) * 180.0;
          rawYaw = ((float)y / 32768.0) * 180.0;
        }
      }
      bufPos = 0;
    }
  }
}