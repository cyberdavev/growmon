/****
 * ESP32-S2 Mini + SCD30 (UART Modbus) + Web UI + OTA
 * - 4 digital outputs:
 *     0: Light
 *     1: Nutrient
 *     2: Ext Fan
 *     3: Mist
 * - 2 digital inputs with pull-ups:
 *     7: Level 1
 *     8: Level 2
 * - Daily schedules:
 *     - Light: 1 ON/OFF time pair (repeats daily)
 *     - Nutrient: 8 ON/OFF time slots (repeat daily)
 * - Humidity control:
 *     - If humidity < min -> Mist ON until humidity >= max
 *     - If humidity > max -> Ext Fan ON until humidity < max
 * - All schedule + humidity settings stored in NVS and persist over power cycles
 * - SCD30 over UART/Modbus (Robertndrei library)
 * - NTP time sync (RTC-like), OTA updates
 * - Web UI with live data, graph, IO control, and schedule/config UI
 ****/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <Preferences.h>

// SCD30 Modbus library (https://github.com/Robertndrei/SCD30-Modbus)
#include <scd30_modbus.h>

// ---- USER CONFIG ----

// WiFi credentials (CHANGE THESE)
const char* ssid     = "mealworm";
const char* password = "6361A998AC4D88F79EAA";

// OTA configuration
const char* otaHostname = "ESP32-S2-Sensor";
const char* otaPassword = "bu11erfly";  // CHANGE THIS

// Time / NTP config
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 2 * 3600;   // e.g. GMT+2
const int   daylightOffset_sec = 0;          // adjust for DST if needed

// Digital Outputs (4) with labels
const uint8_t NUM_OUTPUTS = 4;
const uint8_t outputPins[NUM_OUTPUTS]     = {1, 2, 3, 4};
const char*   outputLabels[NUM_OUTPUTS]   = {"Light", "Nutrient", "Ext Fan", "Mist"};
// Indices for clarity
const uint8_t OUTPUT_LIGHT   = 0;
const uint8_t OUTPUT_NUTRIENT= 1;
const uint8_t OUTPUT_EXT_FAN = 2;
const uint8_t OUTPUT_MIST    = 3;

// Digital Inputs (2) with pull-ups (switches) and labels
const uint8_t NUM_INPUTS = 2;
const uint8_t inputPins[NUM_INPUTS]   = {7, 8};
const char*   inputLabels[NUM_INPUTS] = {"Level 1", "Level 2"};

// SCD30 UART Pins (Modbus)
// 33 & 35 are free for SPI.
#define SCD30_RX_PIN 11  // ESP32-S2 RX connected to SCD30 TX
#define SCD30_TX_PIN 12  // ESP32-S2 TX connected to SCD30 RX

// Modbus parameters
#define SCD30_BAUDRATE 19200

// Reading intervals
const unsigned long SENSOR_READ_INTERVAL   = 5000;   // 5s: live data update
const unsigned long GRAPH_UPDATE_INTERVAL  = 60000;  // 60s: graph point
const size_t        MOVING_AVG_SAMPLES     = 12;     // 12 * 5s = 60s window

// Schedule constants
const uint8_t NUTRIENT_SLOTS = 8;
const uint16_t DISABLED_TIME = 0xFFFF;  // sentinel for disabled slot

// Default configuration (if nothing stored yet)
const uint16_t DEFAULT_LIGHT_ON_MIN   = 8 * 60;   // 08:00
const uint16_t DEFAULT_LIGHT_OFF_MIN  = 20 * 60;  // 20:00
const float    DEFAULT_HUM_MIN        = 50.0;     // 50%
const float    DEFAULT_HUM_MAX        = 70.0;     // 70%

// ---- GLOBALS ----

WebServer server(80);
HardwareSerial SCD30Serial(1);  // Use UART1
SCD30_Modbus airSensor;
Preferences prefs;

// Latest sensor readings (live)
float co2_ppm      = NAN;
float temperatureC = NAN;
float humidityRH   = NAN;

// Circular buffer for moving average (still used internally)
float  co2Buffer[MOVING_AVG_SAMPLES];
float  tempBuffer[MOVING_AVG_SAMPLES];
float  humBuffer[MOVING_AVG_SAMPLES];
size_t bufferIndex  = 0;
bool   bufferFilled = false;

// Timing
unsigned long lastSensorRead  = 0;
unsigned long lastGraphUpdate = 0;

// Time-string for display
String lastTimeString = "N/A";

// Schedules and thresholds (in minutes from midnight)
uint16_t lightOnMinutesCfg  = DEFAULT_LIGHT_ON_MIN;
uint16_t lightOffMinutesCfg = DEFAULT_LIGHT_OFF_MIN;

uint16_t nutrientOnMinutes[NUTRIENT_SLOTS];
uint16_t nutrientOffMinutes[NUTRIENT_SLOTS];

float humidityMinThreshold = DEFAULT_HUM_MIN;
float humidityMaxThreshold = DEFAULT_HUM_MAX;

// Humidity control state (for hysteresis)
bool mistActive = false;
bool fanActive  = false;

// ---- FORWARD DECLARATIONS ----
void handleRoot();
void handleToggle();
void handleStatus();
void handleSaveConfig();

String generateHTML();
String getDateTimeString();

void setupWifi();
void setupTime();
void setupOTA();

void readSCD30();
void updateMovingAverageBuffers();
void addGraphPointIfDue();

void loadConfig();
void saveConfigFromRequest();
void updateAutomation();

int  getMinutesFromMidnight();
bool isTimeInInterval(uint16_t nowMin, uint16_t startMin, uint16_t endMin);

// ---- SETUP ----

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize Outputs
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }

  // Initialize Inputs with internal pull-ups
  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }

  // Initialize UART for SCD30 (set pins)
  SCD30Serial.begin(SCD30_BAUDRATE, SERIAL_8N1, SCD30_RX_PIN, SCD30_TX_PIN);

  // Initialize the SCD30 Modbus sensor
  airSensor.begin(&SCD30Serial, SCD30_BAUDRATE);

  // Start continuous measurement (pressure offset 0 = disabled)
  if (!airSensor.startContinuousMeasurement(0)) {
    Serial.println("Failed to start SCD30 continuous measurement.");
  } else {
    Serial.println("SCD30 Modbus sensor initialized and measuring.");
  }

  // Load schedule + humidity config from NVS
  loadConfig();

  // Connect WiFi
  setupWifi();

  // Setup time via NTP
  setupTime();

  // Setup OTA
  setupOTA();

  // Setup WebServer routes
  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/toggle",    HTTP_GET,  handleToggle);
  server.on("/status",    HTTP_GET,  handleStatus);
  server.on("/saveConfig",HTTP_POST, handleSaveConfig);
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

// ---- LOOP ----

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

  // Automation: schedules + humidity control
  updateAutomation();
}

// ---- WIFI & TIME ----

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

// ---- OTA ----

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

// ---- SCD30 READING (UART MODBUS) ----

void readSCD30() {
  // Check if new data is ready
  if (!airSensor.dataReady()) {
    Serial.println("SCD30: no new data yet.");
    return;
  }

  // Read the measurement into internal fields
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

  // Averages are logged; frontend still uses live values for 1-min points
}

// ---- CONFIG LOAD/SAVE ----

void loadConfig() {
  prefs.begin("growmon", true);  // read-only
  lightOnMinutesCfg  = prefs.getUShort("light_on",  DEFAULT_LIGHT_ON_MIN);
  lightOffMinutesCfg = prefs.getUShort("light_off", DEFAULT_LIGHT_OFF_MIN);

  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char keyOn[12];
    char keyOff[12];
    snprintf(keyOn,  sizeof(keyOn),  "nut_on%u",  i);
    snprintf(keyOff, sizeof(keyOff), "nut_off%u", i);
    nutrientOnMinutes[i]  = prefs.getUShort(keyOn,  DISABLED_TIME);
    nutrientOffMinutes[i] = prefs.getUShort(keyOff, DISABLED_TIME);
  }

  humidityMinThreshold = prefs.getFloat("hum_min", DEFAULT_HUM_MIN);
  humidityMaxThreshold = prefs.getFloat("hum_max", DEFAULT_HUM_MAX);
  prefs.end();

  // Basic sanity checks
  if (humidityMinThreshold < 0 || humidityMinThreshold > 100) humidityMinThreshold = DEFAULT_HUM_MIN;
  if (humidityMaxThreshold < 0 || humidityMaxThreshold > 100) humidityMaxThreshold = DEFAULT_HUM_MAX;
}

void saveConfigFromRequest() {
  // Parse light schedule
  int loH = server.hasArg("light_on_h")  ? server.arg("light_on_h").toInt()  : 8;
  int loM = server.hasArg("light_on_m")  ? server.arg("light_on_m").toInt()  : 0;
  int lfH = server.hasArg("light_off_h") ? server.arg("light_off_h").toInt() : 20;
  int lfM = server.hasArg("light_off_m") ? server.arg("light_off_m").toInt() : 0;

  loH = constrain(loH, 0, 23);
  lfH = constrain(lfH, 0, 23);
  loM = constrain(loM, 0, 59);
  lfM = constrain(lfM, 0, 59);

  lightOnMinutesCfg  = loH * 60 + loM;
  lightOffMinutesCfg = lfH * 60 + lfM;

  // Nutrient slots
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char hOnName[16], mOnName[16], hOffName[16], mOffName[16];
    snprintf(hOnName,  sizeof(hOnName),  "nut_on_h%u",  i);
    snprintf(mOnName,  sizeof(mOnName),  "nut_on_m%u",  i);
    snprintf(hOffName, sizeof(hOffName), "nut_off_h%u", i);
    snprintf(mOffName, sizeof(mOffName), "nut_off_m%u", i);

    int hOn  = server.hasArg(hOnName)  ? server.arg(hOnName).toInt()  : 0;
    int mOn  = server.hasArg(mOnName)  ? server.arg(mOnName).toInt()  : 0;
    int hOff = server.hasArg(hOffName) ? server.arg(hOffName).toInt() : 0;
    int mOff = server.hasArg(mOffName) ? server.arg(mOffName).toInt() : 0;

    hOn  = constrain(hOn,  0, 23);
    mOn  = constrain(mOn,  0, 59);
    hOff = constrain(hOff, 0, 23);
    mOff = constrain(mOff, 0, 59);

    uint16_t onMin  = hOn  * 60 + mOn;
    uint16_t offMin = hOff * 60 + mOff;

    // If on == off, treat as disabled
    if (onMin == offMin) {
      nutrientOnMinutes[i]  = DISABLED_TIME;
      nutrientOffMinutes[i] = DISABLED_TIME;
    } else {
      nutrientOnMinutes[i]  = onMin;
      nutrientOffMinutes[i] = offMin;
    }
  }

  // Humidity thresholds
  if (server.hasArg("hum_min")) {
    humidityMinThreshold = server.arg("hum_min").toFloat();
  }
  if (server.hasArg("hum_max")) {
    humidityMaxThreshold = server.arg("hum_max").toFloat();
  }

  // Clamp to [0,100]
  humidityMinThreshold = constrain(humidityMinThreshold, 0.0f, 100.0f);
  humidityMaxThreshold = constrain(humidityMaxThreshold, 0.0f, 100.0f);

  // Save to NVS
  prefs.begin("growmon", false);
  prefs.putUShort("light_on",  lightOnMinutesCfg);
  prefs.putUShort("light_off", lightOffMinutesCfg);

  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char keyOn[12];
    char keyOff[12];
    snprintf(keyOn,  sizeof(keyOn),  "nut_on%u",  i);
    snprintf(keyOff, sizeof(keyOff), "nut_off%u", i);
    prefs.putUShort(keyOn,  nutrientOnMinutes[i]);
    prefs.putUShort(keyOff, nutrientOffMinutes[i]);
  }

  prefs.putFloat("hum_min", humidityMinThreshold);
  prefs.putFloat("hum_max", humidityMaxThreshold);
  prefs.end();
}

// ---- AUTOMATION ----

int getMinutesFromMidnight() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  return timeinfo.tm_hour * 60 + timeinfo.tm_min;
}

// Handles intervals that may cross midnight
bool isTimeInInterval(uint16_t nowMin, uint16_t startMin, uint16_t endMin) {
  if (startMin == endMin) return false;     // disabled / zero-length
  if (startMin < endMin) {
    return (nowMin >= startMin && nowMin < endMin);
  } else {
    // crosses midnight, e.g. 20:00->06:00
    return (nowMin >= startMin || nowMin < endMin);
  }
}

void updateAutomation() {
  int nowMin = getMinutesFromMidnight();
  if (nowMin < 0) return;  // no valid time yet

  // --- LIGHT SCHEDULE (Output 0) ---
  bool lightOn = isTimeInInterval((uint16_t)nowMin, lightOnMinutesCfg, lightOffMinutesCfg);
  digitalWrite(outputPins[OUTPUT_LIGHT], lightOn ? HIGH : LOW);

  // --- NUTRIENT SCHEDULE (Output 1) ---
  bool nutrientOn = false;
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    uint16_t onMin  = nutrientOnMinutes[i];
    uint16_t offMin = nutrientOffMinutes[i];
    if (onMin == DISABLED_TIME || offMin == DISABLED_TIME) continue;
    if (isTimeInInterval((uint16_t)nowMin, onMin, offMin)) {
      nutrientOn = true;
      break;
    }
  }
  digitalWrite(outputPins[OUTPUT_NUTRIENT], nutrientOn ? HIGH : LOW);

  // --- HUMIDITY CONTROL FOR MIST (Output 3) AND EXT FAN (Output 2) ---
  // Hysteresis behavior:
  //  - Mist:
  //      If humidity < min  AND mist is OFF -> turn ON
  //      If humidity >= max AND mist is ON  -> turn OFF
  //  - Ext Fan:
  //      If humidity > max  AND fan is OFF  -> turn ON
  //      If humidity < max  AND fan is ON   -> turn OFF
  if (!isnan(humidityRH) && humidityMinThreshold < humidityMaxThreshold) {
    // Mist control
    if (!mistActive && humidityRH < humidityMinThreshold) {
      mistActive = true;
    } else if (mistActive && humidityRH >= humidityMaxThreshold) {
      mistActive = false;
    }

    // Fan control
    if (!fanActive && humidityRH > humidityMaxThreshold) {
      fanActive = true;
    } else if (fanActive && humidityRH < humidityMaxThreshold) {
      fanActive = false;
    }
  } else {
    // Invalid thresholds or no reading -> turn both off
    mistActive = false;
    fanActive  = false;
  }

  digitalWrite(outputPins[OUTPUT_MIST],    mistActive ? HIGH : LOW);
  digitalWrite(outputPins[OUTPUT_EXT_FAN], fanActive  ? HIGH : LOW);
}

// ---- HTTP HANDLERS ----

void handleRoot() {
  server.send(200, "text/html", generateHTML());
}

void handleToggle() {
  if (!server.hasArg("out")) {
    server.send(400, "text/plain", "Missing 'out' parameter");
    return;
  }

  int outIndex = server.arg("out").toInt();
  if (outIndex < 0 || outIndex >= NUM_OUTPUTS) {
    server.send(400, "text/plain", "Invalid output index");
    return;
  }

  int pin = outputPins[outIndex];
  int state = digitalRead(pin);
  digitalWrite(pin, !state);

  // NOTE: Automation logic in updateAutomation() will override
  // these manual toggles as soon as it runs. Use mainly for testing.

  // Redirect back to root (page reload)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  // Build JSON: live sensor data + IO states + timestamp + config (for UI)
  String json = "{";

  json += "\"co2_ppm\":"       + String(co2_ppm, 2)      + ",";
  json += "\"temperature_c\":" + String(temperatureC, 2) + ",";
  json += "\"humidity_rh\":"   + String(humidityRH, 2)   + ",";
  json += "\"datetime\":\""    + getDateTimeString()     + "\",";

  // Inputs
  json += "\"inputs\":[";
  for (int i = 0; i < NUM_INPUTS; i++) {
    int val = digitalRead(inputPins[i]);
    json += String(val == LOW ? 1 : 0);  // 1 = pressed (LOW), 0 = released (HIGH)
    if (i < NUM_INPUTS - 1) json += ",";
  }
  json += "],";

  // Outputs
  json += "\"outputs\":[";
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    int val = digitalRead(outputPins[i]);
    json += String(val == HIGH ? 1 : 0);
    if (i < NUM_OUTPUTS - 1) json += ",";
  }
  json += "],";

  // Config for UI
  json += "\"light_on_min\":"  + String(lightOnMinutesCfg)  + ",";
  json += "\"light_off_min\":" + String(lightOffMinutesCfg) + ",";
  json += "\"nut_on\":[";
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    json += String(nutrientOnMinutes[i]);
    if (i < NUTRIENT_SLOTS - 1) json += ",";
  }
  json += "],\"nut_off\":[";
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    json += String(nutrientOffMinutes[i]);
    if (i < NUTRIENT_SLOTS - 1) json += ",";
  }
  json += "],";

  json += "\"hum_min\":" + String(humidityMinThreshold, 1) + ",";
  json += "\"hum_max\":" + String(humidityMaxThreshold, 1);

  json += "}";

  server.send(200, "application/json", json);
}

void handleSaveConfig() {
  saveConfigFromRequest();
  server.send(200, "text/plain", "OK");
}

// ---- HTML PAGE ----

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
      color: #ffff;
      text-align: center;
      margin: 0;
      padding: 0;
    }
    .container {
      max-width: 1200px;
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
      color: #ffff;
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
      color: #aaaa;
    }
    #co2ChartContainer {
      margin-top: 20px;
      height: 500px;       /* taller graph area */
    }
    #co2Chart {
      width: 100%;
      height: 100%;        /* fill the container */
    }
    canvas {
      background-color: #1e1e1e;
      border-radius: 8px;
    }
    .schedule-row {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      align-items: center;
      margin: 5px 0;
      gap: 8px;
      font-size: 0.9em;
    }
    .schedule-row label {
      min-width: 60px;
      text-align: right;
    }
    .schedule-row select,
    .schedule-row input[type="number"] {
      padding: 3px 5px;
      border-radius: 4px;
      border: 1px solid #444;
      background-color: #222;
      color: #fff;
    }
    #saveStatus {
      margin-top: 10px;
      font-size: 0.9em;
    }
    .nut-grid-header {
      display: flex;
      justify-content: center;
      gap: 16px;
      font-weight: bold;
      margin-bottom: 4px;
    }
    .nut-grid-row {
      display: flex;
      justify-content: center;
      gap: 16px;
      align-items: center;
      margin: 3px 0;
    }
    .nut-slot-label {
      width: 60px;
      text-align: right;
      color: #ff9800;
      font-weight: bold;
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
      <h2>Schedules & Humidity Control</h2>
      <form id="scheduleForm" onsubmit="saveConfig(event)">
        <h3>Light Schedule (Daily)</h3>
        <div class="schedule-row">
          <label>Time ON:</label>
          <select id="light_on_h" name="light_on_h"></select>
          :
          <select id="light_on_m" name="light_on_m"></select>
        </div>
        <div class="schedule-row">
          <label>Time OFF:</label>
          <select id="light_off_h" name="light_off_h"></select>
          :
          <select id="light_off_m" name="light_off_m"></select>
        </div>

        <h3>Nutrient Pump Schedule (Daily)</h3>
        <div class="nut-grid-header">
          <span style="width:60px;"></span>
          <span>ON</span>
          <span>OFF</span>
        </div>
        <div id="nutrientSlots">
          <!-- 8 slots: each row has ON/ OFF hour/min selects -->
          <!-- Slots 0..7 -->
          <!-- We build them statically so they are always visible -->
          <!-- Slot 1 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 1</span>
            <span>
              <select id="nut_on_h0" name="nut_on_h0"></select> :
              <select id="nut_on_m0" name="nut_on_m0"></select>
            </span>
            <span>
              <select id="nut_off_h0" name="nut_off_h0"></select> :
              <select id="nut_off_m0" name="nut_off_m0"></select>
            </span>
          </div>
          <!-- Slot 2 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 2</span>
            <span>
              <select id="nut_on_h1" name="nut_on_h1"></select> :
              <select id="nut_on_m1" name="nut_on_m1"></select>
            </span>
            <span>
              <select id="nut_off_h1" name="nut_off_h1"></select> :
              <select id="nut_off_m1" name="nut_off_m1"></select>
            </span>
          </div>
          <!-- Slot 3 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 3</span>
            <span>
              <select id="nut_on_h2" name="nut_on_h2"></select> :
              <select id="nut_on_m2" name="nut_on_m2"></select>
            </span>
            <span>
              <select id="nut_off_h2" name="nut_off_h2"></select> :
              <select id="nut_off_m2" name="nut_off_m2"></select>
            </span>
          </div>
          <!-- Slot 4 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 4</span>
            <span>
              <select id="nut_on_h3" name="nut_on_h3"></select> :
              <select id="nut_on_m3" name="nut_on_m3"></select>
            </span>
            <span>
              <select id="nut_off_h3" name="nut_off_h3"></select> :
              <select id="nut_off_m3" name="nut_off_m3"></select>
            </span>
          </div>
          <!-- Slot 5 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 5</span>
            <span>
              <select id="nut_on_h4" name="nut_on_h4"></select> :
              <select id="nut_on_m4" name="nut_on_m4"></select>
            </span>
            <span>
              <select id="nut_off_h4" name="nut_off_h4"></select> :
              <select id="nut_off_m4" name="nut_off_m4"></select>
            </span>
          </div>
          <!-- Slot 6 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 6</span>
            <span>
              <select id="nut_on_h5" name="nut_on_h5"></select> :
              <select id="nut_on_m5" name="nut_on_m5"></select>
            </span>
            <span>
              <select id="nut_off_h5" name="nut_off_h5"></select> :
              <select id="nut_off_m5" name="nut_off_m5"></select>
            </span>
          </div>
          <!-- Slot 7 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 7</span>
            <span>
              <select id="nut_on_h6" name="nut_on_h6"></select> :
              <select id="nut_on_m6" name="nut_on_m6"></select>
            </span>
            <span>
              <select id="nut_off_h6" name="nut_off_h6"></select> :
              <select id="nut_off_m6" name="nut_off_m6"></select>
            </span>
          </div>
          <!-- Slot 8 -->
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot 8</span>
            <span>
              <select id="nut_on_h7" name="nut_on_h7"></select> :
              <select id="nut_on_m7" name="nut_on_m7"></select>
            </span>
            <span>
              <select id="nut_off_h7" name="nut_off_h7"></select> :
              <select id="nut_off_m7" name="nut_off_m7"></select>
            </span>
          </div>
        </div>

        <h3>Humidity Control</h3>
        <div class="schedule-row">
          <label>Min (%):</label>
          <input type="number" id="humMin" name="hum_min" step="0.1" min="0" max="100">
          <label>Max (%):</label>
          <input type="number" id="humMax" name="hum_max" step="0.1" min="0" max="100">
        </div>

        <button class="btn" type="submit">Save Schedule & Humidity</button>
      </form>
      <div id="saveStatus"></div>
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
      <p>Light, Nutrient, Ext Fan and Mist are controlled automatically by the schedules and humidity settings.</p>
    </div>
  </div>

  <script>
    const NUTRIENT_SLOTS = 8;
    const maxPoints = 120;

    let co2El = document.getElementById('co2');
    let tempEl = document.getElementById('temp');
    let humEl = document.getElementById('hum');
    let datetimeEl = document.getElementById('datetime');
    let inputsEl = document.getElementById('inputs');
    let outputsEl = document.getElementById('outputs');
    let saveStatusEl = document.getElementById('saveStatus');
    let humMinInput = document.getElementById('humMin');
    let humMaxInput = document.getElementById('humMax');

    // Graph data
    let labels = [];
    let co2Data = [];
    let tempData = [];
    let humData = [];
    let lastGraphTimestamp = 0;

    // Load graph history
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

    loadGraphFromStorage();

    // Chart.js
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
              color: '#ffff'
            },
            grid: {
              color: 'rgba(255, 255, 255, 0.05)'
            }
          }
        },
        plugins: {
          legend: {
            labels: {
              color: '#ffff'
            }
          }
        }
      }
    });

    // IO UI
    function buildIOElements(inputs, outputs) {
      let inHtml = '';
      const inputLabelsArr = [)rawliteral" +
        "\"" + String(inputLabels[0]) + "\", " +
        "\"" + String(inputLabels[1]) + "\"" +
      R"rawliteral(];
      for (let i = 0; i < inputs.length; i++) {
        const pressed = inputs[i] === 1;
        const label = inputLabelsArr[i] || ('Input ' + i);
        inHtml += `
          <div>
            <span class="label">${label}</span>:
            <span class="${pressed ? 'indicator-on' : 'indicator-off'}">
              ${pressed ? 'ON (Pressed)' : 'OFF (Released)'}
            </span>
          </div>
        `;
      }
      inputsEl.innerHTML = inHtml;

      let outHtml = '';
      const outputLabelsArr = [)rawliteral" +
        "\"" + String(outputLabels[0]) + "\", " +
        "\"" + String(outputLabels[1]) + "\", " +
        "\"" + String(outputLabels[2]) + "\", " +
        "\"" + String(outputLabels[3]) + "\"" +
      R"rawliteral(];
      for (let i = 0; i < outputs.length; i++) {
        const on = outputs[i] === 1;
        const label = outputLabelsArr[i] || ('Output ' + i);
        outHtml += `
          <div>
            <span class="label">${label}</span>:
            <span class="${on ? 'indicator-on' : 'indicator-off'}">
              ${on ? 'ON' : 'OFF'}
            </span>
            <a class="btn ${on ? 'btn-off' : 'btn-on'}" href="/toggle?out=${i}">
              ${label} ${on ? 'OFF' : 'ON'}
            </a>
          </div>
        `;
      }
      outputsEl.innerHTML = outHtml;
    }

    // Time select helpers
    function fillTimeSelects(idH, idM) {
      const hSel = document.getElementById(idH);
      const mSel = document.getElementById(idM);
      if (!hSel || !mSel) return;

      hSel.innerHTML = '';
      mSel.innerHTML = '';

      for (let h = 0; h < 24; h++) {
        const opt = document.createElement('option');
        opt.value = h;
        opt.textContent = (h < 10 ? '0' : '') + h;
        hSel.appendChild(opt);
      }
      for (let m = 0; m < 60; m += 5) {
        const opt = document.createElement('option');
        opt.value = m;
        opt.textContent = (m < 10 ? '0' : '') + m;
        mSel.appendChild(opt);
      }
    }

    function setTimeSelectFromMinutes(idH, idM, minutes) {
      const hSel = document.getElementById(idH);
      const mSel = document.getElementById(idM);
      if (!hSel || !mSel) return;
      if (minutes < 0 || minutes >= 24*60) {
        hSel.value = 0;
        mSel.value = 0;
        return;
      }
      const h = Math.floor(minutes / 60);
      const m = minutes % 60;
      const step = 5;
      const mStep = Math.round(m / step) * step;
      hSel.value = h;
      mSel.value = mStep % 60;
    }

    // Set up all select options at page load
    function initStaticTimeSelects() {
      // Light
      fillTimeSelects('light_on_h',  'light_on_m');
      fillTimeSelects('light_off_h', 'light_off_m');
      // Nutrient 8 slots
      for (let i = 0; i < NUTRIENT_SLOTS; i++) {
        fillTimeSelects('nut_on_h'  + i, 'nut_on_m'  + i);
        fillTimeSelects('nut_off_h' + i, 'nut_off_m' + i);
      }
    }

    function initScheduleUIFromStatus(data) {
      // Light
      const lightOnMin  = data.light_on_min  || 0;
      const lightOffMin = data.light_off_min || 0;
      setTimeSelectFromMinutes('light_on_h',  'light_on_m',  lightOnMin);
      setTimeSelectFromMinutes('light_off_h', 'light_off_m', lightOffMin);

      // Nutrient
      const nutOn  = Array.isArray(data.nut_on)  ? data.nut_on  : [];
      const nutOff = Array.isArray(data.nut_off) ? data.nut_off : [];
      for (let i = 0; i < NUTRIENT_SLOTS; i++) {
        const onMin  = nutOn[i]  !== undefined ? nutOn[i]  : 0xFFFF;
        const offMin = nutOff[i] !== undefined ? nutOff[i] : 0xFFFF;
        if (onMin !== 0xFFFF && offMin !== 0xFFFF) {
          setTimeSelectFromMinutes('nut_on_h'  + i, 'nut_on_m'  + i, onMin);
          setTimeSelectFromMinutes('nut_off_h' + i, 'nut_off_m' + i, offMin);
        } else {
          // disabled: show 00:00
          setTimeSelectFromMinutes('nut_on_h'  + i, 'nut_on_m'  + i, 0);
          setTimeSelectFromMinutes('nut_off_h' + i, 'nut_off_m' + i, 0);
        }
      }

      // Humidity thresholds
      if (typeof data.hum_min === 'number') humMinInput.value = data.hum_min.toFixed(1);
      if (typeof data.hum_max === 'number') humMaxInput.value = data.hum_max.toFixed(1);
    }

    function saveConfig(event) {
      event.preventDefault();
      saveStatusEl.textContent = 'Saving...';

      const form = document.getElementById('scheduleForm');
      const formData = new FormData(form);
      const params = new URLSearchParams(formData).toString();

      fetch('/saveConfig', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: params
      })
      .then(res => res.text())
      .then(text => {
        saveStatusEl.textContent = 'Saved successfully.';
        setTimeout(() => { saveStatusEl.textContent = ''; }, 3000);
      })
      .catch(err => {
        console.error('Error saving config:', err);
        saveStatusEl.textContent = 'Error saving config.';
      });
    }

    function maybeAddGraphPoint(datetime, co2, temp, hum) {
      const now = Date.now();
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

          if (!updateStatus.initializedSchedule) {
            initScheduleUIFromStatus(data);
            updateStatus.initializedSchedule = true;
          }

          if (!isNaN(data.co2_ppm) && !isNaN(data.temperature_c) && !isNaN(data.humidity_rh)) {
            maybeAddGraphPoint(data.datetime, data.co2_ppm, data.temperature_c, data.humidity_rh);
          }
        })
        .catch(err => {
          console.error('Error fetching /status:', err);
        });
    }
    updateStatus.initializedSchedule = false;

    // Initial setup
    initStaticTimeSelects();
    buildIOElements([], []);
    updateStatus();
    setInterval(updateStatus, 5000);
  </script>
</body>
</html>
)rawliteral";

  return html;
}