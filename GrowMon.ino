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

