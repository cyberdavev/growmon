/****
 * ESP32-S2 Mini + SCD30 (UART Modbus) + Web UI + Web OTA Updater
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
 *     - Nutrient: 12 ON time slots (repeat daily), each with an enabled flag
 *         Nutrient output turns OFF once BOTH inputs are OFF,
 *         and only turns ON again at the next enabled ON slot.
 * - Humidity control:
 *     - If humidity < min -> Mist ON until humidity >= max
 *     - If humidity > max -> Ext Fan ON until humidity < max
 * - Manual Override:
 *     - Override switch OFF -> outputs follow automation
 *     - Override switch ON  -> outputs follow web sliders
 * - All schedule + humidity + override settings stored in NVS and persist over power cycles
 * - SCD30 over UART/Modbus (Robertndrei library)
 * - NTP time sync (RTC-like)
 * - Web UI with live data, graph, IO control with sliders + indicator lamps, schedule/config UI
 * - Web OTA updater (no login prompt), based on Update.h approach:
 *   https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
 ****/

#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Preferences.h>
#include <Update.h>      // Web OTA

// SCD30 Modbus library (https://github.com/Robertndrei/SCD30-Modbus)
#include <scd30_modbus.h>

// ---- USER CONFIG ----

// WiFi credentials (CHANGE THESE)
const char* ssid     = "mealworm";
const char* password = "6361A998AC4D88F79EAA";

// Time / NTP config
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 2 * 3600;   // e.g. GMT+2
const int   daylightOffset_sec = 0;          // adjust for DST if needed

// Digital Outputs (4) with labels
const uint8_t NUM_OUTPUTS = 4;
const uint8_t outputPins[NUM_OUTPUTS]   = {1, 2, 3, 4};
const char*   outputLabels[NUM_OUTPUTS] = {"Light", "Nutrient", "Ext Fan", "Mist"};
// Indices for clarity
const uint8_t OUTPUT_LIGHT     = 0;
const uint8_t OUTPUT_NUTRIENT  = 1;
const uint8_t OUTPUT_EXT_FAN   = 2;
const uint8_t OUTPUT_MIST      = 3;

// Digital Inputs (2) with pull-ups (switches) and labels
const uint8_t NUM_INPUTS = 2;
const uint8_t inputPins[NUM_INPUTS]   = {7, 8};
const char*   inputLabels[NUM_INPUTS] = {"Level 1", "Level 2"};

// Inputs used for nutrient "done" condition
const uint8_t NUTRIENT_INPUT1_INDEX = 0; // Level 1 -> inputPins[0]
const uint8_t NUTRIENT_INPUT2_INDEX = 1; // Level 2 -> inputPins[1]

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
const uint8_t  NUTRIENT_SLOTS = 12;
const uint16_t DISABLED_TIME  = 0xFFFF;  // sentinel for disabled slot

// Default configuration (if nothing stored yet)
const uint16_t DEFAULT_LIGHT_ON_MIN   = 8 * 60;   // 08:00
const uint16_t DEFAULT_LIGHT_OFF_MIN  = 20 * 60;  // 20:00
const float    DEFAULT_HUM_MIN        = 50.0;     // 50%
const float    DEFAULT_HUM_MAX        = 70.0;     // 70%

// ---- GLOBALS ----

WebServer      server(80);
HardwareSerial SCD30Serial(1);  // Use UART1
SCD30_Modbus   airSensor;
Preferences    prefs;

// Latest sensor readings (live)
float co2_ppm      = 400;
float temperatureC = 0;
float humidityRH   = 0;

// Circular buffer for moving average
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

// Nutrient ON times and enabled flags
uint16_t nutrientOnMinutes[NUTRIENT_SLOTS];
bool     nutrientSlotEnabled[NUTRIENT_SLOTS];

// Humidity thresholds
float humidityMinThreshold = DEFAULT_HUM_MIN;
float humidityMaxThreshold = DEFAULT_HUM_MAX;

// Humidity control state (for hysteresis)
bool mistActive = false;
bool fanActive  = false;

// Nutrient control state
bool nutrientOutputOn   = false;
int  nutrientLastMinute = -1;
// Nutrient pump safety timeout
const unsigned long NUTRIENT_MAX_ON_TIME_MS = 60000; // 60 seconds
unsigned long nutrientOnStartMs = 0;

// Manual override: if true -> outputs follow manualOverrideOutputs[]
bool manualOverrideEnabled = false;
bool manualOverrideOutputs[NUM_OUTPUTS] = {false, false, false, false};

// ---- FORWARD DECLARATIONS ----
void handleRoot();
void handleToggle();
void handleStatus();
void handleSaveConfig();
void handleSetOverride();

// Web OTA updater endpoint
void setupWebOTA();

String generateHTML();
String getDateTimeString();

void setupWifi();
void setupTime();

void readSCD30();
void updateMovingAverageBuffers();
void addGraphPointIfDue();

void loadConfig();
void saveConfigFromRequest();
void saveOverrideToNVS();
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

  // Load schedule + humidity + override config from NVS
  loadConfig();

  // Connect WiFi
  setupWifi();

  // Setup time via NTP
  setupTime();

  // Setup WebServer routes
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/toggle",      HTTP_GET,  handleToggle);
  server.on("/status",      HTTP_GET,  handleStatus);
  server.on("/saveConfig",  HTTP_POST, handleSaveConfig);
  server.on("/setOverride", HTTP_POST, handleSetOverride);

  // Web OTA updater endpoint (/update)
  setupWebOTA();

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

  // Automation: schedules + humidity + nutrient logic
  updateAutomation();

  delay(1);
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

// ---- WEB OTA (no login) ----

void setupWebOTA() {
  // Endpoint to receive firmware .bin upload
  // Based on the approach shown here:
  // https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
  server.on("/update", HTTP_POST,
    []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      Serial.println((Update.hasError()) ? "OTA Update: FAIL" : "OTA Update: OK");
      delay(300);
      ESP.restart();
    },
    []() {
      HTTPUpload& upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("OTA Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { // true = set the size to the current progress
          Serial.printf("OTA Update Success: %u bytes. Rebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        Serial.println("OTA Update Aborted");
      }
      delay(1);
    }
  );
}

// ---- SCD30 READING (UART MODBUS) ----

void readSCD30() {
  if (!airSensor.dataReady()) {
    Serial.println("SCD30: no new data yet.");
    return;
  }

  if (!airSensor.read()) {
    Serial.println("SCD30: Error reading sensor data.");
    return;
  }

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
    snprintf(keyOn, sizeof(keyOn), "nut_on%u", i);
    nutrientOnMinutes[i] = prefs.getUShort(keyOn, DISABLED_TIME);

    char keyEn[12];
    snprintf(keyEn, sizeof(keyEn), "nut_en%u", i);
    bool defEn = (nutrientOnMinutes[i] != DISABLED_TIME);
    nutrientSlotEnabled[i] = prefs.getBool(keyEn, defEn);
  }

  humidityMinThreshold = prefs.getFloat("hum_min", DEFAULT_HUM_MIN);
  humidityMaxThreshold = prefs.getFloat("hum_max", DEFAULT_HUM_MAX);

  manualOverrideEnabled = prefs.getBool("override_en", false);
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    char key[16];
    snprintf(key, sizeof(key), "override_o%u", i);
    manualOverrideOutputs[i] = prefs.getBool(key, false);
  }

  prefs.end();

  if (humidityMinThreshold < 0 || humidityMinThreshold > 100) humidityMinThreshold = DEFAULT_HUM_MIN;
  if (humidityMaxThreshold < 0 || humidityMaxThreshold > 100) humidityMaxThreshold = DEFAULT_HUM_MAX;
}

void saveConfigFromRequest() {
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

  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char hOnName[16], mOnName[16], enName[16];
    snprintf(hOnName, sizeof(hOnName), "nut_on_h%u", i);
    snprintf(mOnName, sizeof(mOnName), "nut_on_m%u", i);
    snprintf(enName, sizeof(enName), "nut_en%u",  i);

    int hOn = server.hasArg(hOnName) ? server.arg(hOnName).toInt() : 0;
    int mOn = server.hasArg(mOnName) ? server.arg(mOnName).toInt() : 0;

    hOn = constrain(hOn, 0, 23);
    mOn = constrain(mOn, 0, 59);

    uint16_t onMin = hOn * 60 + mOn;
    bool enabled = server.hasArg(enName);  // checkbox present when checked

    if (!enabled) {
      nutrientOnMinutes[i]   = DISABLED_TIME;
      nutrientSlotEnabled[i] = false;
    } else {
      nutrientOnMinutes[i]   = onMin;
      nutrientSlotEnabled[i] = true;
    }
  }

  if (server.hasArg("hum_min")) humidityMinThreshold = server.arg("hum_min").toFloat();
  if (server.hasArg("hum_max")) humidityMaxThreshold = server.arg("hum_max").toFloat();

  humidityMinThreshold = constrain(humidityMinThreshold, 0.0f, 100.0f);
  humidityMaxThreshold = constrain(humidityMaxThreshold, 0.0f, 100.0f);

  prefs.begin("growmon", false);
  prefs.putUShort("light_on",  lightOnMinutesCfg);
  prefs.putUShort("light_off", lightOffMinutesCfg);

  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char keyOn[12];
    snprintf(keyOn, sizeof(keyOn), "nut_on%u", i);
    prefs.putUShort(keyOn, nutrientOnMinutes[i]);
  }
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    char keyEn[12];
    snprintf(keyEn, sizeof(keyEn), "nut_en%u", i);
    prefs.putBool(keyEn, nutrientSlotEnabled[i]);
  }

  prefs.putFloat("hum_min", humidityMinThreshold);
  prefs.putFloat("hum_max", humidityMaxThreshold);
  prefs.end();
}

void saveOverrideToNVS() {
  prefs.begin("growmon", false);
  prefs.putBool("override_en", manualOverrideEnabled);
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    char key[16];
    snprintf(key, sizeof(key), "override_o%u", i);
    prefs.putBool(key, manualOverrideOutputs[i]);
  }
  prefs.end();
}

// ---- AUTOMATION ----

int getMinutesFromMidnight() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  return timeinfo.tm_hour * 60 + timeinfo.tm_min;
}

bool isTimeInInterval(uint16_t nowMin, uint16_t startMin, uint16_t endMin) {
  if (startMin == endMin) return false;
  if (startMin < endMin) {
    return (nowMin >= startMin && nowMin < endMin);
  } else {
    return (nowMin >= startMin || nowMin < endMin);
  }
}

void updateAutomation() {
  int nowMin = getMinutesFromMidnight();
  if (nowMin < 0) return;

  bool desiredLight    = false;
  bool desiredNutrient = false;
  bool desiredExtFan   = false;
  bool desiredMist     = false;

  desiredLight = isTimeInInterval((uint16_t)nowMin, lightOnMinutesCfg, lightOffMinutesCfg);

  if (nowMin != nutrientLastMinute) {
    nutrientLastMinute = nowMin;
    for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
      uint16_t onMin = nutrientOnMinutes[i];
      if (!nutrientSlotEnabled[i]) continue;
      if (onMin == DISABLED_TIME)  continue;
      if (onMin == (uint16_t)nowMin) {
        nutrientOutputOn = true;
        nutrientOnStartMs = millis();
        break;
      }
    }
  }

  // Safety timeout: force nutrient pump OFF after max on-time
  if (nutrientOutputOn && (millis() - nutrientOnStartMs >= NUTRIENT_MAX_ON_TIME_MS)) {
    nutrientOutputOn = false;
  }
  // Inverted logic: HIGH = active (level reached)
  bool level1On = (digitalRead(inputPins[NUTRIENT_INPUT1_INDEX]) == HIGH);
  bool level2On = (digitalRead(inputPins[NUTRIENT_INPUT2_INDEX]) == HIGH);

  if (nutrientOutputOn && !level1On && !level2On) {
    nutrientOutputOn = false;
  }
  desiredNutrient = nutrientOutputOn;

  if (!isnan(humidityRH) && humidityMinThreshold < humidityMaxThreshold) {
    if (!mistActive && humidityRH < humidityMinThreshold) mistActive = true;
    else if (mistActive && humidityRH >= humidityMaxThreshold) mistActive = false;

    if (!fanActive && humidityRH > humidityMaxThreshold) fanActive = true;
    else if (fanActive && humidityRH < humidityMaxThreshold) fanActive = false;
  } else {
    mistActive = false;
    fanActive  = false;
  }

  desiredMist   = mistActive;
  desiredExtFan = fanActive;

  if (!manualOverrideEnabled) {
    digitalWrite(outputPins[OUTPUT_LIGHT],    desiredLight    ? HIGH : LOW);
    digitalWrite(outputPins[OUTPUT_NUTRIENT], desiredNutrient ? HIGH : LOW);
    digitalWrite(outputPins[OUTPUT_EXT_FAN],  desiredExtFan   ? HIGH : LOW);
    digitalWrite(outputPins[OUTPUT_MIST],     desiredMist     ? HIGH : LOW);
  } else {
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
      digitalWrite(outputPins[i], manualOverrideOutputs[i] ? HIGH : LOW);
    }
  }
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

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  String json = "{";

  json += "\"co2_ppm\":"       + String(co2_ppm, 2)      + ",";
  json += "\"temperature_c\":" + String(temperatureC, 2) + ",";
  json += "\"humidity_rh\":"   + String(humidityRH, 2)   + ",";
  json += "\"datetime\":\""    + getDateTimeString()     + "\",";

  json += "\"inputs\":[";
  for (int i = 0; i < NUM_INPUTS; i++) {
    int val = digitalRead(inputPins[i]);
    json += String(val == LOW ? 1 : 0);
    if (i < NUM_INPUTS - 1) json += ",";
  }
  json += "],";

  json += "\"outputs\":[";
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    int val = digitalRead(outputPins[i]);
    json += String(val == HIGH ? 1 : 0);
    if (i < NUM_OUTPUTS - 1) json += ",";
  }
  json += "],";

  json += "\"override\":" + String(manualOverrideEnabled ? "true" : "false") + ",";
  json += "\"override_outputs\":[";
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    json += String(manualOverrideOutputs[i] ? 1 : 0);
    if (i < NUM_OUTPUTS - 1) json += ",";
  }
  json += "],";

  json += "\"light_on_min\":"  + String(lightOnMinutesCfg)  + ",";
  json += "\"light_off_min\":" + String(lightOffMinutesCfg) + ",";
  json += "\"nut_on\":[";
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    json += String(nutrientOnMinutes[i]);
    if (i < NUTRIENT_SLOTS - 1) json += ",";
  }
  json += "],";
  json += "\"nut_en\":[";
  for (uint8_t i = 0; i < NUTRIENT_SLOTS; i++) {
    json += (nutrientSlotEnabled[i] ? "1" : "0");
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

void handleSetOverride() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");

    bool newOverride = manualOverrideEnabled;
    bool newOutputs[NUM_OUTPUTS];
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) newOutputs[i] = manualOverrideOutputs[i];

    int oPos = body.indexOf("\"override\"");
    if (oPos >= 0) {
      int colon = body.indexOf(":", oPos);
      if (colon >= 0) {
        String val = body.substring(colon + 1);
        val.toLowerCase();
        if (val.indexOf("true") >= 0)  newOverride = true;
        if (val.indexOf("false") >= 0) newOverride = false;
      }
    }

    int arrPos = body.indexOf("\"outputs\"");
    if (arrPos >= 0) {
      int lb = body.indexOf("[", arrPos);
      int rb = body.indexOf("]", lb);
      if (lb >= 0 && rb > lb) {
        String arr = body.substring(lb + 1, rb);
        uint8_t idx = 0;
        while (idx < NUM_OUTPUTS && arr.length() > 0) {
          int comma = arr.indexOf(",");
          String token;
          if (comma >= 0) {
            token = arr.substring(0, comma);
            arr = arr.substring(comma + 1);
          } else {
            token = arr;
            arr = "";
          }
          token.trim();
          int v = token.toInt();
          newOutputs[idx] = (v != 0);
          idx++;
        }
      }
    }

    manualOverrideEnabled = newOverride;
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) manualOverrideOutputs[i] = newOutputs[i];
    saveOverrideToNVS();

    server.send(200, "application/json", "{\"status\":\"ok\"}");
    return;
  }

  server.send(400, "application/json", "{\"status\":\"error\",\"msg\":\"no body\"}");
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
    body { font-family: Arial, sans-serif; background-color: #121212; color: #ffff; text-align: center; margin: 0; padding: 0; }
    .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
    h1 { color: #00bcd4; }
    .card { background-color: #1e1e1e; border-radius: 10px; padding: 15px; margin: 10px auto; box-shadow: 0 2px 4px rgba(0,0,0,0.5); }
    .flex-row { display: flex; flex-wrap: wrap; justify-content: space-around; }
    .flex-item { flex: 1 1 200px; margin: 10px; }
    .label { font-weight: bold; color: #ff9800; }
    .value { font-size: 1.4em; margin-top: 5px; }

    .btn { display: inline-block; padding: 10px 20px; margin: 5px; border: none; border-radius: 20px; background-color: #03a9f4; color: #ffff; font-size: 1em; cursor: pointer; transition: background-color 0.2s, transform 0.1s; text-decoration: none; }
    .btn:hover { background-color: #0288d1; transform: translateY(-1px); }
    .indicator-on { color: #4caf50; font-weight: bold; }
    .indicator-off { color: #f44336; font-weight: bold; }

    #co2ChartContainer { margin-top: 20px; height: 500px; }
    canvas { background-color: #1e1e1e; border-radius: 8px; }

    .schedule-row { display: flex; flex-wrap: wrap; justify-content: center; align-items: center; margin: 5px 0; gap: 8px; font-size: 0.9em; }
    .schedule-row label { min-width: 60px; text-align: right; }
    .schedule-row select, .schedule-row input[type="number"] { padding: 3px 5px; border-radius: 4px; border: 1px solid #444; background-color: #222; color: #fff; }

    .nut-grid-header { display: flex; justify-content: center; gap: 16px; font-weight: bold; margin-bottom: 4px; }
    .nut-grid-row { display: flex; justify-content: center; gap: 16px; align-items: center; margin: 3px 0; }
    .nut-slot-label { width: 60px; text-align: right; color: #ff9800; font-weight: bold; }

    .switch { position: relative; display: inline-block; width: 50px; height: 24px; margin-left: 10px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider-round { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .2s; border-radius: 24px; }
    .slider-round:before { position: absolute; content: ""; height: 18px; width: 18px; left: 3px; bottom: 3px; background-color: white; transition: .2s; border-radius: 50%; }
    input:checked + .slider-round { background-color: #4caf50; }
    input:checked + .slider-round:before { transform: translateX(26px); }

    .lamp { display: inline-block; width: 14px; height: 14px; margin-left: 10px; border-radius: 50%; background-color: #f44336; box-shadow: 0 0 6px rgba(244, 67, 54, 0.8); }
    .lamp-on { background-color: #4caf50; box-shadow: 0 0 8px rgba(76, 175, 80, 0.9); }

    .override-row { margin-bottom: 10px; }

    /* Web OTA */
    .ota-box { max-width: 520px; margin: 0 auto; }
    #otaFileLabel { display:block; padding:10px; border:1px solid #444; border-radius:6px; background:#222; cursor:pointer; }
    #otaProgressWrap { width:100%; height:10px; background:#333; border-radius:10px; overflow:hidden; margin-top:10px; }
    #otaProgressBar { width:0%; height:10px; background:#03a9f4; }
    #otaMsg { margin-top: 10px; font-size: 0.95em; color: #aaaa; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Grow Mon V0.02</h1>

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
      <div id="inputs"></div>
    </div>

    <div class="card">
      <h2>Digital Outputs</h2>
      <div class="override-row">
        <span class="label">Override</span>:
        <label class="switch">
          <input type="checkbox" id="overrideSwitch">
          <span class="slider-round"></span>
        </label>
        <span id="overrideStatus" class="indicator-off">OFF (Automatic)</span>
      </div>
      <div id="outputs"></div>
    </div>

    <div class="card">
      <h2>Schedules & Humidity Control</h2>
      <form id="scheduleForm" onsubmit="saveConfig(event)">
        <h3>Light Schedule (Daily)</h3>
        <div class="schedule-row">
          <label>Time ON:</label>
          <select id="light_on_h" name="light_on_h"></select> : <select id="light_on_m" name="light_on_m"></select>
        </div>
        <div class="schedule-row">
          <label>Time OFF:</label>
          <select id="light_off_h" name="light_off_h"></select> : <select id="light_off_m" name="light_off_m"></select>
        </div>

        <h3>Nutrient Pump Schedule (Daily)</h3>
        <div class="nut-grid-header">
          <span style="width:60px;"></span>
          <span>Enabled</span>
          <span>ON Time</span>
        </div>
        <div id="nutrientSlots"></div>

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

    <!-- WEB OTA SECTION (bottom of page) -->
    <div class="card">
      <h2>Web OTA Update</h2>
      <div class="ota-box">
        <form id="otaForm" method="POST" action="/update" enctype="multipart/form-data">
          <input type="file" id="otaFile" name="update" style="display:none" accept=".bin">
          <label id="otaFileLabel" for="otaFile">Choose firmware file (.bin)...</label>
          <button class="btn" type="submit" id="otaUploadBtn">Upload & Update</button>
          <div id="otaProgressWrap"><div id="otaProgressBar"></div></div>
          <div id="otaMsg">After upload completes, the device will reboot automatically.</div>
        </form>
      </div>
    </div>

    <div class="card">
      <h3>OTA Notes</h3>
      <p class="footer" style="margin:0;">
        Export the binary in Arduino IDE: <span class="label">Sketch → Export compiled Binary</span>, then upload the generated <span class="label">.bin</span>.
      </p>
    </div>

  </div>

  <script>
    const NUTRIENT_SLOTS = 12;
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
    let overrideSwitch = document.getElementById('overrideSwitch');
    let overrideStatus = document.getElementById('overrideStatus');

    // Web OTA UI
    const otaForm = document.getElementById('otaForm');
    const otaFile = document.getElementById('otaFile');
    const otaFileLabel = document.getElementById('otaFileLabel');
    const otaProgressBar = document.getElementById('otaProgressBar');
    const otaMsg = document.getElementById('otaMsg');
    const otaUploadBtn = document.getElementById('otaUploadBtn');

    let labels = [];
    let co2Data = [];
    let tempData = [];
    let humData = [];
    let lastGraphTimestamp = 0;

    function buildNutrientSlotsUI() {
      const container = document.getElementById('nutrientSlots');
      let html = '';
      for (let i = 0; i < NUTRIENT_SLOTS; i++) {
        const slotNum = i + 1;
        html += `
          <div class="nut-grid-row">
            <span class="nut-slot-label">Slot ${slotNum}</span>
            <span>
              <input type="checkbox" id="nut_en${i}" name="nut_en${i}">
            </span>
            <span>
              <select id="nut_on_h${i}" name="nut_on_h${i}"></select> :
              <select id="nut_on_m${i}" name="nut_on_m${i}"></select>
            </span>
          </div>
        `;
      }
      container.innerHTML = html;
    }

    function loadGraphFromStorage() {
      try {
        const stored = localStorage.getItem('scd30_graph_history');
        if (!stored) return;
        const obj = JSON.parse(stored);
        if (obj && Array.isArray(obj.labels)) {
          labels = obj.labels; co2Data = obj.co2Data; tempData = obj.tempData; humData = obj.humData;
        }
      } catch(e) { console.error(e); }
    }

    function saveGraphToStorage() {
      try {
        const obj = { labels, co2Data, tempData, humData };
        localStorage.setItem('scd30_graph_history', JSON.stringify(obj));
      } catch(e) { console.error(e); }
    }

    loadGraphFromStorage();

    const ctx = document.getElementById('co2Chart').getContext('2d');
    const co2Chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [
          { label: 'CO₂ (ppm)', data: co2Data, borderColor: '#00bcd4', yAxisID: 'y1', tension: 0.2, borderWidth: 2, pointRadius: 2 },
          { label: 'Temp (°C)', data: tempData, borderColor: '#ff9800', yAxisID: 'y2', tension: 0.2, borderWidth: 2, pointRadius: 2 },
          { label: 'Humidity (%RH)', data: humData, borderColor: '#4caf50', yAxisID: 'y2', tension: 0.2, borderWidth: 2, pointRadius: 2 }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false,
        scales: {
          y1: { type: 'linear', position: 'left', title: { display: true, text: 'CO₂' }, ticks: { color: '#00bcd4' } },
          y2: { type: 'linear', position: 'right', title: { display: true, text: 'T/H' }, ticks: { color: '#ff9800' } }
        },
        plugins: { legend: { labels: { color: '#fff' } } }
      }
    });

    function buildIOElements(inputs, outputs) {
      const inputLabelsArr = ["Level 1", "Level 2"];
      const outputLabelsArr = ["Light", "Nutrient", "Ext Fan", "Mist"];

      let inHtml = '';
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
      for (let i = 0; i < outputs.length; i++) {
        const on = outputs[i] === 1;
        const label = outputLabelsArr[i] || ('Output ' + i);
        outHtml += `
          <div>
            <span class="label">${label}</span>:
            <label class="switch">
              <input type="checkbox" class="outSwitch" data-index="${i}" ${on ? 'checked' : ''}>
              <span class="slider-round"></span>
            </label>
            <span class="lamp ${on ? 'lamp-on' : ''}" id="lamp${i}"></span>
          </div>
        `;
      }
      outputsEl.innerHTML = outHtml;

      const sw = document.querySelectorAll('.outSwitch');
      sw.forEach(el => el.addEventListener('change', onOutputSwitchChanged));
    }

    function applyOverrideUI(overrideOn, overrideOutputs) {
      overrideSwitch.checked = overrideOn;
      if (overrideOn) {
        overrideStatus.textContent = 'ON (Manual)';
        overrideStatus.classList.remove('indicator-off');
        overrideStatus.classList.add('indicator-on');
      } else {
        overrideStatus.textContent = 'OFF (Automatic)';
        overrideStatus.classList.remove('indicator-on');
        overrideStatus.classList.add('indicator-off');
      }

      const sw = document.querySelectorAll('.outSwitch');
      sw.forEach((el) => {
        const idx = parseInt(el.getAttribute('data-index'));
        if (!isNaN(idx) && overrideOutputs && overrideOutputs.length > idx) {
          el.checked = overrideOutputs[idx] === 1;
        }
      });
    }

    function sendOverrideState() {
      const sw = document.querySelectorAll('.outSwitch');
      let outputs = [0,0,0,0];
      sw.forEach((el) => {
        const idx = parseInt(el.getAttribute('data-index'));
        if (!isNaN(idx) && idx < outputs.length) outputs[idx] = el.checked ? 1 : 0;
      });

      const body = JSON.stringify({ override: overrideSwitch.checked, outputs: outputs });

      fetch('/setOverride', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: body
      }).catch(err => console.error('setOverride error', err));
    }

    function onOutputSwitchChanged() {
      if (!overrideSwitch.checked) overrideSwitch.checked = true;
      sendOverrideState();
    }

    function fillTimeSelects(idH, idM) {
      const hSel = document.getElementById(idH), mSel = document.getElementById(idM);
      for (let h=0; h<24; h++) hSel.options.add(new Option(h<10?'0'+h:h, h));
      for (let m=0; m<60; m+=5) mSel.options.add(new Option(m<10?'0'+m:m, m));
    }

    function setTimeSelectFromMinutes(idH, idM, min) {
      const hSel = document.getElementById(idH), mSel = document.getElementById(idM);
      if (min === 65535) min = 0;
      hSel.value = Math.floor(min/60);
      mSel.value = Math.round((min%60)/5)*5 % 60;
    }

    function initStaticTimeSelects() {
      fillTimeSelects('light_on_h', 'light_on_m');
      fillTimeSelects('light_off_h', 'light_off_m');
      for (let i=0; i<NUTRIENT_SLOTS; i++) fillTimeSelects('nut_on_h'+i, 'nut_on_m'+i);
    }

    function saveConfig(e) {
      e.preventDefault();
      saveStatusEl.textContent = 'Saving...';
      fetch('/saveConfig', { method: 'POST', body: new URLSearchParams(new FormData(e.target)) })
        .then(() => { saveStatusEl.textContent = 'Saved.'; setTimeout(()=>saveStatusEl.textContent='', 2000); });
    }

    function maybeAddGraphPoint(dt, co2, temp, hum) {
      const now = Date.now();
      if (now - lastGraphTimestamp < 60000) return;
      lastGraphTimestamp = now;
      labels.push(dt); co2Data.push(co2); tempData.push(temp); humData.push(hum);
      if (labels.length > maxPoints) { labels.shift(); co2Data.shift(); tempData.shift(); humData.shift(); }
      co2Chart.update(); saveGraphToStorage();
    }

    function updateStatus() {
      fetch('/status').then(r => r.json()).then(data => {
        co2El.textContent = data.co2_ppm.toFixed(2);
        tempEl.textContent = data.temperature_c.toFixed(2);
        humEl.textContent = data.humidity_rh.toFixed(2);
        datetimeEl.textContent = 'Time: ' + data.datetime;

        if (Array.isArray(data.inputs) && Array.isArray(data.outputs)) {
          buildIOElements(data.inputs, data.outputs);
        }

        const ov = !!data.override;
        const ovOut = Array.isArray(data.override_outputs) ? data.override_outputs : [];
        applyOverrideUI(ov, ovOut);

        if (!updateStatus.init) {
          initScheduleUIFromStatus(data);
          updateStatus.init = true;
        }
        maybeAddGraphPoint(data.datetime, data.co2_ppm, data.temperature_c, data.humidity_rh);
      });
    }

    function initScheduleUIFromStatus(data) {
      setTimeSelectFromMinutes('light_on_h', 'light_on_m', data.light_on_min);
      setTimeSelectFromMinutes('light_off_h', 'light_off_m', data.light_off_min);

      if (Array.isArray(data.nut_on)) {
        data.nut_on.forEach((min, i) => {
          if (i < NUTRIENT_SLOTS) setTimeSelectFromMinutes('nut_on_h'+i, 'nut_on_m'+i, min);
        });
      }

      if (Array.isArray(data.nut_en)) {
        data.nut_en.forEach((en, i) => {
          if (i < NUTRIENT_SLOTS) {
            const cb = document.getElementById('nut_en'+i);
            if (cb) cb.checked = (en === 1 || en === "1");
          }
        });
      }

      humMinInput.value = data.hum_min;
      humMaxInput.value = data.hum_max;
    }

    // Web OTA interactions
    otaFile.addEventListener('change', () => {
      if (otaFile.files && otaFile.files.length > 0) {
        otaFileLabel.textContent = otaFile.files[0].name;
        otaMsg.textContent = 'Ready to upload: ' + otaFile.files[0].name;
      } else {
        otaFileLabel.textContent = 'Choose firmware file (.bin)...';
      }
    });

    otaForm.addEventListener('submit', (e) => {
      e.preventDefault();
      if (!otaFile.files || otaFile.files.length === 0) {
        otaMsg.textContent = 'Please select a .bin file first.';
        return;
      }

      otaUploadBtn.disabled = true;
      otaProgressBar.style.width = '0%';
      otaMsg.textContent = 'Uploading...';

      const formData = new FormData();
      formData.append('update', otaFile.files[0]);

      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);

      xhr.upload.onprogress = (evt) => {
        if (evt.lengthComputable) {
          const pct = Math.round((evt.loaded / evt.total) * 100);
          otaProgressBar.style.width = pct + '%';
          otaMsg.textContent = 'Upload progress: ' + pct + '%';
        }
      };

      xhr.onload = () => {
        otaUploadBtn.disabled = false;
        if (xhr.status === 200) {
          otaMsg.textContent = 'Upload finished. Device is rebooting...';
        } else {
          otaMsg.textContent = 'Upload failed. HTTP ' + xhr.status;
        }
      };

      xhr.onerror = () => {
        otaUploadBtn.disabled = false;
        otaMsg.textContent = 'Upload error (network).';
      };

      xhr.send(formData);
    });

    buildNutrientSlotsUI();
    initStaticTimeSelects();
    buildIOElements([], []);
    updateStatus();
    setInterval(updateStatus, 1000);

    overrideSwitch.addEventListener('change', () => sendOverrideState());
  </script>
</body>
</html>
)rawliteral";

  return html;
}
