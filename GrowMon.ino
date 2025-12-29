/*******************************************************
 * ESP32-S2 Mini + SCD30 (UART Modbus) + Web UI + OTA
 * - 6 digital outputs
 * - 4 digital inputs (pull-up, switches)
 * - SCD30 over UART/Modbus (Sensirion, Robertndrei library)
 * - NTP time sync (RTC-like)
 * - OTA updates
 * - Web UI with:
 *     - Live SCD30 data (5s)
 *     - Graph (1 point every 60s)
 *     - Input/output control
 *     - Graph history persisted via localStorage
 *******************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <time.h>

// SCD30 Modbus library (https://github.com/Robertndrei/SCD30-Modbus)
#include <scd30_modbus.h>

// ---------------------- USER CONFIG ----------------------

// WiFi credentials
const char* ssid = "mealworm";
const char* password = "6361A998AC4D88F79EAA";

// OTA configuration
const char* otaHostname = "ESP32-S2-Sensor";
const char* otaPassword = "bu11erfly";  // CHANGE THIS

// Time / NTP config
const char* ntpServer       = "pool.ntp.org";
const long  gmtOffset_sec   = 2 * 3600;   // e.g. GMT+2
const int   daylightOffset_sec = 0;       // adjust for DST if needed

// Digital Outputs (6)
const uint8_t outputPins[6] = {1, 2, 3, 4, 5, 6};

// Digital Inputs (4) with pull-ups (switches)
const uint8_t inputPins[4]  = {7, 8, 9, 10};

// SCD30 UART Pins (Modbus)
// 33 & 35 are now free for SPI as requested.
#define SCD30_RX_PIN 11  // ESP32-S2 RX connected to SCD30 TX
#define SCD30_TX_PIN 12  // ESP32-S2 TX connected to SCD30 RX

// Modbus parameters
#define SCD30_BAUDRATE 19200

// Reading intervals
const unsigned long SENSOR_READ_INTERVAL   = 5000;   // 5s: live data update
const unsigned long GRAPH_UPDATE_INTERVAL  = 60000;  // 60s: graph point
const size_t        MOVING_AVG_SAMPLES     = 12;     // 12 * 5s = 60s window

// ---------------------- GLOBALS ----------------------

WebServer server(80);

// SCD30 Modbus instance
HardwareSerial SCD30Serial(1);  // Use UART1
SCD30_Modbus airSensor;

// Latest sensor readings (live)
float co2_ppm      = NAN;
float temperatureC = NAN;
float humidityRH   = NAN;

// Circular buffer for moving average (still used internally)
float co2Buffer[MOVING_AVG_SAMPLES];
float tempBuffer[MOVING_AVG_SAMPLES];
float humBuffer[MOVING_AVG_SAMPLES];
size_t bufferIndex = 0;
bool bufferFilled  = false;

// Timing
unsigned long lastSensorRead   = 0;
unsigned long lastGraphUpdate  = 0;

// Time-string for display
String lastTimeString = "N/A";

// ---------------------- FORWARD DECLARATIONS ----------------------
void handleRoot();
void handleToggle();
void handleStatus();
String generateHTML();
String getDateTimeString();
void setupWifi();
void setupTime();
void setupOTA();
void readSCD30();
void updateMovingAverageBuffers();
void addGraphPointIfDue();

// ---------------------- SETUP ----------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize Outputs
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }

  // Initialize Inputs with internal pull-ups
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }

  // Initialize UART for SCD30 (set pins)
  SCD30Serial.begin(SCD30_BAUDRATE, SERIAL_8N1, SCD30_RX_PIN, SCD30_TX_PIN);

  // Initialize the SCD30 Modbus sensor
  airSensor.begin(&SCD30Serial, SCD30_BAUDRATE);

  // Optional configuration (you can uncomment if needed):
  // if (!airSensor.setMeasurementInterval(2)) {
  //   Serial.println("ERROR! Failed to set measurement interval");
  // }

  // Start continuous measurement (pressure offset 0 = disabled)
  if (!airSensor.startContinuousMeasurement(0)) {
    Serial.println("Failed to start SCD30 continuous measurement.");
  } else {
    Serial.println("SCD30 Modbus sensor initialized and measuring.");
  }

  // Connect WiFi
  setupWifi();

  // Setup time via NTP
  setupTime();

  // Setup OTA
  setupOTA();

  // Setup WebServer routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/toggle", HTTP_GET, handleToggle);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("HTTP server started.");

  // Init buffers
  for (size_t i = 0; i < MOVING_AVG_SAMPLES; i++) {
    co2Buffer[i]  = NAN;
    tempBuffer[i] = NAN;
    humBuffer[i]  = NAN;
  }

  lastSensorRead  = millis();
  lastGraphUpdate = millis();
}

// ---------------------- LOOP ----------------------

void loop() {
  // OTA
  ArduinoOTA.handle();

  // Web
  server.handleClient();

  unsigned long now = millis();

  // Periodic SCD30 read (every SENSOR_READ_INTERVAL)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    readSCD30();
    updateMovingAverageBuffers();
  }

  // Periodic graph point (every GRAPH_UPDATE_INTERVAL)
  if (now - lastGraphUpdate >= GRAPH_UPDATE_INTERVAL) {
    lastGraphUpdate = now;
    addGraphPointIfDue();
  }
}

// ---------------------- WIFI & TIME ----------------------

void setupWifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) { // ~20s
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed. Continuing without WiFi.");
  }
}

void setupTime() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Getting time from NTP...");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeString[64];
      strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.print("Time synchronized: ");
      Serial.println(timeString);
      lastTimeString = String(timeString);
    } else {
      Serial.println("Failed to obtain time from NTP.");
    }
  } else {
    Serial.println("WiFi not connected; skipping NTP time sync.");
  }
}

String getDateTimeString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeString[64];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
    lastTimeString = String(timeString);
  }
  return lastTimeString;
}

// ---------------------- OTA ----------------------

void setupOTA() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("OTA disabled (WiFi not connected).");
    return;
  }

  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
          type = "sketch";
        } else {  // U_SPIFFS
          type = "filesystem";
        }
        Serial.println("OTA Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nOTA End");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

  ArduinoOTA.begin();
  Serial.println("OTA Initialized. Use hostname '" + String(otaHostname) + "' for updates.");
}

// ---------------------- SCD30 READING (UART MODBUS) ----------------------

void readSCD30() {
  // Check if new data is ready (matches README example)
  if (!airSensor.dataReady()) {
    Serial.println("SCD30: no new data yet.");
    return;
  }

  // Read the measurement into internal fields (CO2, temperature, relative_humidity)
  if (!airSensor.read()) {
    Serial.println("SCD30: Error reading sensor data.");
    return;
  }

  // Use public fields exposed by the library
  co2_ppm      = airSensor.CO2;
  temperatureC = airSensor.temperature;
  humidityRH   = airSensor.relative_humidity;

  Serial.print("SCD30 (Modbus) Read -> CO2: ");
  Serial.print(co2_ppm);
  Serial.print(" ppm, Temp: ");
  Serial.print(temperatureC);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidityRH);
  Serial.println(" %RH");
}

void updateMovingAverageBuffers() {
  co2Buffer[bufferIndex]  = co2_ppm;
  tempBuffer[bufferIndex] = temperatureC;
  humBuffer[bufferIndex]  = humidityRH;

  bufferIndex++;
  if (bufferIndex >= MOVING_AVG_SAMPLES) {
    bufferIndex  = 0;
    bufferFilled = true;
  }
}

void addGraphPointIfDue() {
  // Compute moving average over valid samples
  size_t samples = bufferFilled ? MOVING_AVG_SAMPLES : bufferIndex;
  if (samples == 0) return;

  float sumCO2 = 0, sumT = 0, sumH = 0;
  size_t validCount = 0;

  for (size_t i = 0; i < samples; i++) {
    if (!isnan(co2Buffer[i]) && !isnan(tempBuffer[i]) && !isnan(humBuffer[i])) {
      sumCO2 += co2Buffer[i];
      sumT   += tempBuffer[i];
      sumH   += humBuffer[i];
      validCount++;
    }
  }

  if (validCount == 0) {
    Serial.println("No valid samples for moving average.");
    return;
  }

  float avgCO2 = sumCO2 / validCount;
  float avgT   = sumT   / validCount;
  float avgH   = sumH   / validCount;

  Serial.print("1-min Moving Avg -> CO2: ");
  Serial.print(avgCO2);
  Serial.print(" ppm, Temp: ");
  Serial.print(avgT);
  Serial.print(" °C, Humidity: ");
  Serial.print(avgH);
  Serial.println(" %RH");

  // Currently the averages are only logged; the frontend still uses live values
  // for graph points every ~60s.
}

// ---------------------- HTTP HANDLERS ----------------------

void handleRoot() {
  server.send(200, "text/html", generateHTML());
}

void handleToggle() {
  if (!server.hasArg("out")) {
    server.send(400, "text/plain", "Missing 'out' parameter");
    return;
  }

  int outIndex = server.arg("out").toInt();
  if (outIndex < 0 || outIndex >= 6) {
    server.send(400, "text/plain", "Invalid output index");
    return;
  }

  int pin = outputPins[outIndex];
  int state = digitalRead(pin);
  digitalWrite(pin, !state);

  // Redirect back to root (page reload)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  // Build JSON: live sensor data + IO states + timestamp
  String json = "{";

  json += "\"co2_ppm\":"       + String(co2_ppm, 2)      + ",";
  json += "\"temperature_c\":" + String(temperatureC, 2) + ",";
  json += "\"humidity_rh\":"   + String(humidityRH, 2)   + ",";
  json += "\"datetime\":\""    + getDateTimeString()     + "\",";

  // Inputs
  json += "\"inputs\":[";
  for (int i = 0; i < 4; i++) {
    int val = digitalRead(inputPins[i]);
    json += String(val == LOW ? 1 : 0);  // 1 = pressed (LOW), 0 = released (HIGH)
    if (i < 3) json += ",";
  }
  json += "],";

  // Outputs
  json += "\"outputs\":[";
  for (int i = 0; i < 6; i++) {
    int val = digitalRead(outputPins[i]);
    json += String(val == HIGH ? 1 : 0);
    if (i < 5) json += ",";
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

// ---------------------- HTML PAGE ----------------------
// ---------------------- HTML PAGE ----------------------

String generateHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-S2 SCD30 Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <!-- Chart.js CDN -->
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #121212;
      color: #ffffff;
      text-align: center;
      margin: 0;
      padding: 0;
    }
    .container {
      max-width: 1000px;
      margin: 0 auto;
      padding: 20px;
    }
    h1 {
      color: #00bcd4;
    }
    .card {
      background-color: #1e1e1e;
      border-radius: 10px;
      padding: 15px;
      margin: 10px auto;
      box-shadow: 0 2px 4px rgba(0,0,0,0.5);
    }
    .flex-row {
      display: flex;
      flex-wrap: wrap;
      justify-content: space-around;
    }
    .flex-item {
      flex: 1 1 200px;
      margin: 10px;
    }
    .label {
      font-weight: bold;
      color: #ff9800;
    }
    .value {
      font-size: 1.4em;
      margin-top: 5px;
    }
    .btn {
      display: inline-block;
      padding: 10px 20px;
      margin: 5px;
      border: none;
      border-radius: 20px;
      background-color: #03a9f4;
      color: #ffffff;
      font-size: 1em;
      cursor: pointer;
      transition: background-color 0.2s, transform 0.1s;
      text-decoration: none;
    }
    .btn:hover {
      background-color: #0288d1;
      transform: translateY(-1px);
    }
    .btn-on {
      background-color: #4caf50;
    }
    .btn-off {
      background-color: #f44336;
    }
    .indicator-on {
      color: #4caf50;
      font-weight: bold;
    }
    .indicator-off {
      color: #f44336;
      font-weight: bold;
    }
    .footer {
      margin-top: 20px;
      font-size: 0.9em;
      color: #aaaaaa;
    }
    #co2ChartContainer {
      margin-top: 20px;
    }
    canvas {
      background-color: #1e1e1e;
      border-radius: 8px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32-S2 SCD30 Monitor</h1>
    <div class="card">
      <div class="value" id="datetime">Time: --</div>
    </div>

    <div class="flex-row">
      <div class="card flex-item">
        <div class="label">CO₂ (ppm)</div>
        <div class="value" id="co2">--</div>
      </div>
      <div class="card flex-item">
        <div class="label">Temperature (°C)</div>
        <div class="value" id="temp">--</div>
      </div>
      <div class="card flex-item">
        <div class="label">Humidity (%RH)</div>
        <div class="value" id="hum">--</div>
      </div>
    </div>

    <div class="card">
      <h2>Digital Inputs</h2>
      <div id="inputs">
        <!-- Filled by JS -->
      </div>
    </div>

    <div class="card">
      <h2>Digital Outputs</h2>
      <div id="outputs">
        <!-- Filled by JS -->
      </div>
    </div>

    <div class="card">
      <h2>CO₂ / Temp / Humidity Trend (1-min Averages)</h2>
      <div id="co2ChartContainer">
        <canvas id="co2Chart"></canvas>
      </div>
    </div>

    <div class="card">
      <h3>OTA Status</h3>
      <p>OTA updates are <span class="indicator-on">ENABLED</span>.</p>
      <p>Hostname: <span class="label">)rawliteral" + String(otaHostname) + R"rawliteral(</span></p>
    </div>

    <div class="footer">
      <p>Live data updates every 5 seconds. Graph point every 60 seconds.</p>
      <p>Graph history is stored locally in your browser and survives page reloads.</p>
    </div>
  </div>

  <script>
    // ------------ Globals ------------
    let co2El = document.getElementById('co2');
    let tempEl = document.getElementById('temp');
    let humEl = document.getElementById('hum');
    let datetimeEl = document.getElementById('datetime');
    let inputsEl = document.getElementById('inputs');
    let outputsEl = document.getElementById('outputs');

    // Graph data arrays (for Chart.js)
    let labels = [];
    let co2Data = [];
    let tempData = [];
    let humData = [];
    const maxPoints = 120; // up to 120 minutes if 1-min points

    let lastGraphTimestamp = 0;

    // ------------ Local Storage Helpers ------------

    function loadGraphFromStorage() {
      try {
        const stored = localStorage.getItem('scd30_graph_history');
        if (!stored) return;
        const obj = JSON.parse(stored);
        if (obj && Array.isArray(obj.labels) && Array.isArray(obj.co2Data) &&
            Array.isArray(obj.tempData) && Array.isArray(obj.humData)) {
          labels   = obj.labels;
          co2Data  = obj.co2Data;
          tempData = obj.tempData;
          humData  = obj.humData;
        }
      } catch(e) {
        console.error('Failed to load graph history from storage:', e);
      }
    }

    function saveGraphToStorage() {
      try {
        const obj = {
          labels: labels,
          co2Data: co2Data,
          tempData: tempData,
          humData: humData
        };
        localStorage.setItem('scd30_graph_history', JSON.stringify(obj));
      } catch(e) {
        console.error('Failed to save graph history to storage:', e);
      }
    }

    // ------------ Chart.js Init ------------

    loadGraphFromStorage();

    const ctx = document.getElementById('co2Chart').getContext('2d');
    const co2Chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [
          {
            label: 'CO₂ (ppm)',
            data: co2Data,
            borderColor: 'rgba(0, 188, 212, 1)',
            backgroundColor: 'rgba(0, 188, 212, 0.2)',
            yAxisID: 'y1',
            tension: 0.2,
            borderWidth: 2,
            pointRadius: 2
          },
          {
            label: 'Temp (°C)',
            data: tempData,
            borderColor: 'rgba(255, 152, 0, 1)',
            backgroundColor: 'rgba(255, 152, 0, 0.2)',
            yAxisID: 'y2',
            tension: 0.2,
            borderWidth: 2,
            pointRadius: 2
          },
          {
            label: 'Humidity (%RH)',
            data: humData,
            borderColor: 'rgba(76, 175, 80, 1)',
            backgroundColor: 'rgba(76, 175, 80, 0.2)',
            yAxisID: 'y2',
            tension: 0.2,
            borderWidth: 2,
            pointRadius: 2
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          y1: {
            type: 'linear',
            position: 'left',
            title: {
              display: true,
              text: 'CO₂ (ppm)',
              color: '#00bcd4'
            },
            ticks: {
              color: '#00bcd4'
            },
            grid: {
              color: 'rgba(255, 255, 255, 0.05)'
            }
          },
          y2: {
            type: 'linear',
            position: 'right',
            title: {
              display: true,
              text: 'Temp (°C) / Humidity (%RH)',
              color: '#ff9800'
            },
            ticks: {
              color: '#ff9800'
            },
            grid: {
              drawOnChartArea: false
            }
          },
          x: {
            ticks: {
              color: '#ffffff'
            },
            grid: {
              color: 'rgba(255, 255, 255, 0.05)'
            }
          }
        },
        plugins: {
          legend: {
            labels: {
              color: '#ffffff'
            }
          }
        }
      }
    });

    // ------------ UI Builders ------------

    function buildIOElements(inputs, outputs) {
      // Inputs
      let inHtml = '';
      for (let i = 0; i < inputs.length; i++) {
        const pressed = inputs[i] === 1;
        inHtml += `
          <div>
            <span class="label">Input ${i}</span>:
            <span class="${pressed ? 'indicator-on' : 'indicator-off'}">
              ${pressed ? 'ON (Pressed)' : 'OFF (Released)'}
            </span>
          </div>
        `;
      }
      inputsEl.innerHTML = inHtml;

      // Outputs
      let outHtml = '';
      for (let i = 0; i < outputs.length; i++) {
        const on = outputs[i] === 1;
        outHtml += `
          <div>
            <span class="label">Output ${i}</span>:
            <span class="${on ? 'indicator-on' : 'indicator-off'}">
              ${on ? 'ON' : 'OFF'}
            </span>
            <a class="btn ${on ? 'btn-off' : 'btn-on'}" href="/toggle?out=${i}">
              Turn ${on ? 'OFF' : 'ON'}
            </a>
          </div>
        `;
      }
      outputsEl.innerHTML = outHtml;
    }

    // ------------ Data Update ------------

    function maybeAddGraphPoint(datetime, co2, temp, hum) {
      const now = Date.now();
      // Add a new point at most once every 60s
      if (now - lastGraphTimestamp < 60000) return;
      lastGraphTimestamp = now;

      const label = datetime || new Date().toLocaleTimeString();

      labels.push(label);
      co2Data.push(co2);
      tempData.push(temp);
      humData.push(hum);

      if (labels.length > maxPoints) {
        labels.shift();
        co2Data.shift();
        tempData.shift();
        humData.shift();
      }

      co2Chart.update();
      saveGraphToStorage();
    }

    function updateStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          if (data.co2_ppm !== undefined) {
            co2El.textContent = data.co2_ppm.toFixed(2);
          }
          if (data.temperature_c !== undefined) {
            tempEl.textContent = data.temperature_c.toFixed(2);
          }
          if (data.humidity_rh !== undefined) {
            humEl.textContent = data.humidity_rh.toFixed(2);
          }
          if (data.datetime) {
            datetimeEl.textContent = 'Time: ' + data.datetime;
          }
          if (Array.isArray(data.inputs) && Array.isArray(data.outputs)) {
            buildIOElements(data.inputs, data.outputs);
          }

          // Graph point every ~60s using the live values
          if (!isNaN(data.co2_ppm) && !isNaN(data.temperature_c) && !isNaN(data.humidity_rh)) {
            maybeAddGraphPoint(data.datetime, data.co2_ppm, data.temperature_c, data.humidity_rh);
          }
        })
        .catch(err => {
          console.error('Error fetching /status:', err);
        });
    }

    // Initial UI builds with empty arrays for safety
    buildIOElements([], []);

    // Initial fetch
    updateStatus();

    // Poll every 5 seconds to match live sensor interval
    setInterval(updateStatus, 5000);
  </script>
</body>
</html>
)rawliteral";

  return html;
}
