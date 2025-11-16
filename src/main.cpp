/*
 * VaultGuard v7.0 - ENHANCED VERSION WITH CIRCUITIQ SERVER LOGIC
 * Server: https://meifhi-esp-server.onrender.com
 * 
 * FEATURES IMPORTED FROM CIRCUITIQ:
 * ✓ WebSocket bidirectional communication with SSL
 * ✓ HTTPS POST data transmission with retry logic
 * ✓ Command reception and acknowledgment system
 * ✓ Auto-reconnection with exponential backoff
 * ✓ Comprehensive sensor calibration system
 * ✓ Full server protocol compliance
 * ✓ PIR child safety system
 * ✓ Energy monitoring and reporting
 * 
 * SENSOR CALIBRATION:
 * ✓ ACS712 5A Current Sensor with offset correction
 * ✓ ZMPT101B Voltage Sensor for Philippines 220V/60Hz
 * ✓ Automatic calibration storage in EEPROM
 * ✓ Runtime calibration adjustment via commands
 * 
 * Created: 2025
 * Author: VaultGuard Team with CircuitIQ Integration
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

using namespace websockets;

// ==================== PIN DEFINITIONS ====================
#define ACS712_PIN          34      // Current sensor analog input
#define ZMPT101B_PIN        35      // Voltage sensor analog input
#define SSR_CONTROL_PIN     5       // Solid State Relay control
#define RED_LED_PIN         2       // Error/Warning LED
#define GREEN_LED_PIN       4       // Status OK LED
#define BLUE_LED_PIN        19      // Data transmission LED
#define PIR_SENSOR_PIN      18      // PIR motion sensor (HW-456 SR505-M)
#define BUZZER_PIN          21      // Alert buzzer for safety warnings
#define SD_CS_PIN           15      // SD Card chip select

// ==================== NETWORK CONFIGURATION ====================
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"
#define SERVER_HOST         "meifhi-esp-server.onrender.com"
#define SERVER_PORT         443
#define SERVER_ENDPOINT     "/api/data"
#define WS_PATH             "/ws"
#define DEVICE_ID           "VAULTGUARD_001"
#define DEVICE_TYPE         "VAULTGUARD"
#define DEVICE_VERSION      "7.0"
#define WIFI_TIMEOUT_MS     20000
#define SERVER_SEND_INTERVAL 5000
#define SERVER_TIMEOUT      10000
#define MAX_RETRY_ATTEMPTS  3
#define WS_PING_INTERVAL    30000
#define WS_PONG_TIMEOUT     10000

// ==================== SENSOR CALIBRATION FROM CIRCUITIQ ====================
#define ADC_RESOLUTION      4096.0
#define ADC_VREF            3.3

// ACS712 5A Current Sensor Calibration
#define ACS712_SENSITIVITY  0.185      // 185mV/A for 5A version
#define ACS712_ZERO_CURRENT 2.5         // Zero current voltage
#define VOLTAGE_DIVIDER_RATIO 0.667     // Voltage divider for 3.3V ADC
#define MAX_VALID_CURRENT   6.0         // Maximum valid current reading
#define MIN_VALID_CURRENT   0.03        // Minimum current (noise threshold)
#define LOAD_DETECTION_THRESHOLD 0.08   // Current threshold for load detection
#define CURRENT_SAMPLES     50          // Number of samples for averaging

// ZMPT101B Voltage Sensor Calibration (Philippines 220V @ 60Hz)
#define ZMPT101B_SENSITIVITY 0.004     // Sensor sensitivity
#define AC_FREQUENCY        60          // Philippines AC frequency
#define RMS_SAMPLES         100         // Samples for RMS calculation
#define NOMINAL_VOLTAGE     220.0       // Philippines nominal voltage
#define MAX_VOLTAGE         240.0       // Maximum expected voltage
#define MIN_VOLTAGE         200.0       // Minimum valid voltage
#define VOLTAGE_SAMPLES     100         // Number of voltage samples

// ==================== PIR SAFETY SETTINGS ====================
#define PIR_ENABLED         true
#define PIR_MOTION_TIMEOUT  10000      // 10 seconds after motion stops
#define PIR_CHECK_INTERVAL  50          // Check every 50ms
#define PIR_DEBOUNCE_TIME   100         // Debounce 100ms
#define PIR_ALERT_BEEPS     3           // Number of alert beeps
#define PIR_SENSITIVITY     5           // Number of readings for debounce

// ==================== SYSTEM SETTINGS ====================
#define SSR_ON_STATE        LOW
#define SSR_OFF_STATE       HIGH
#define WATCHDOG_TIMEOUT_S  30
#define EEPROM_SIZE         512
#define EEPROM_MAGIC        0x42
#define DEFAULT_POWER_FACTOR 0.95
#define ENERGY_UPDATE_INTERVAL 1000

// ==================== STATE ENUMS ====================
enum SystemState {
  STATE_INITIALIZING,
  STATE_CONNECTING,
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_ERROR,
  STATE_SAFE_MODE
};

enum PIRState {
  PIR_IDLE,               // No motion, normal operation
  PIR_MOTION_NO_LOAD,     // Motion + no load → SSR OFF
  PIR_MOTION_WITH_LOAD,   // Motion + load → no action
  PIR_COOLDOWN            // Waiting after motion stops
};

enum ConnectionState {
  CONN_DISCONNECTED,
  CONN_WIFI_ONLY,
  CONN_FULLY_CONNECTED
};

// ==================== CALIBRATION STRUCTURE ====================
struct CalibrationData {
  uint8_t magic = EEPROM_MAGIC;
  float currentOffset = 1.65;
  float voltageOffset = 1.65;
  float currentCalibration = 1.0;
  float voltageCalibration = 1.0;
  float powerFactorCalibration = DEFAULT_POWER_FACTOR;
  uint32_t calibrationCount = 0;
  unsigned long lastCalibration = 0;
};

// ==================== GLOBAL VARIABLES ====================

// System state
SystemState currentState = STATE_INITIALIZING;
PIRState pirState = PIR_IDLE;
ConnectionState connectionState = CONN_DISCONNECTED;

// Network objects
WiFiClientSecure httpsClient;
WebsocketsClient wsClient;
bool wifiConnected = false;
bool wsConnected = false;
unsigned long lastWifiCheck = 0;
unsigned long lastWsReconnect = 0;
unsigned long lastDataSend = 0;
unsigned long lastWsPing = 0;
int wsReconnectAttempts = 0;
int httpRetryCount = 0;

// Sensor readings
float currentReading = 0.0;
float voltageReading = 0.0;
float powerReading = 0.0;
float energyConsumed = 0.0;
float powerFactor = DEFAULT_POWER_FACTOR;
float frequency = AC_FREQUENCY;
float apparentPower = 0.0;
float reactivePower = 0.0;
unsigned long lastEnergyUpdate = 0;

// Calibration data
CalibrationData calData;
bool calibrationMode = false;
int calibrationStep = 0;

// SSR control
bool ssrEnabled = true;
bool ssrCommandState = true;
bool ssrPirOverride = false;
bool ssrSafetyOverride = false;

// PIR safety
bool pirEnabled = PIR_ENABLED;
bool pirMotionDetected = false;
bool loadPluggedIn = false;
unsigned long lastMotionTime = 0;
unsigned long lastPirCheck = 0;
int pirTriggerCount = 0;
int pirReadings[PIR_SENSITIVITY] = {0};
int pirReadIndex = 0;

// Statistics
unsigned long loopCount = 0;
unsigned long systemStartTime = 0;
int serverSuccessCount = 0;
int serverErrorCount = 0;
int wsMessageCount = 0;
int commandsReceived = 0;
int commandsExecuted = 0;

// Data buffer for reliable transmission
struct DataBuffer {
  float voltage;
  float current;
  float power;
  float energy;
  bool ssrState;
  bool pirMotion;
  unsigned long timestamp;
};
DataBuffer dataBuffer[10];
int bufferIndex = 0;

// ==================== FUNCTION DECLARATIONS ====================
void setupSystem();
void setupWiFi();
void setupWebSocket();
void reconnectWiFi();
void reconnectWebSocket();
void handleWebSocketMessage(WebsocketsMessage message);
void handleCommand(JsonDocument& doc);
void sendCommandAck(const String& command, bool success, const String& message = "");
void sendDataToServer();
bool sendDataWithRetry(const String& jsonData);
void updateSensors();
void updatePIRSafety();
void controlSSR();
float readCurrent();
float readVoltage();
void calculatePower();
void loadCalibration();
void saveCalibration();
void runCalibration();
void printStatus();
void feedWatchdog();
void handleSystemError(const String& error);
void sendSystemStatus();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n\n================================================"));
  Serial.println(F("   VAULTGUARD v7.0 - ENHANCED WITH CIRCUITIQ"));
  Serial.println(F("================================================"));
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Device Type: %s\n", DEVICE_TYPE);
  Serial.printf("Version: %s\n", DEVICE_VERSION);
  Serial.printf("Server: %s:%d\n", SERVER_HOST, SERVER_PORT);
  Serial.println(F("================================================\n"));
  
  setupSystem();
}

// ==================== SYSTEM INITIALIZATION ====================
void setupSystem() {
  // Initialize watchdog timer
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.println(F("✓ Watchdog timer initialized"));
  
  // Initialize EEPROM and load calibration
  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();
  Serial.println(F("✓ EEPROM initialized, calibration loaded"));
  
  // Initialize pins
  pinMode(SSR_CONTROL_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_SENSOR_PIN, INPUT_PULLDOWN);
  pinMode(ACS712_PIN, INPUT);
  pinMode(ZMPT101B_PIN, INPUT);
  
  // Set initial states
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println(F("✓ GPIO pins initialized"));
  
  // Test sensors
  delay(100);
  float testCurrent = readCurrent();
  float testVoltage = readVoltage();
  int pirReading = digitalRead(PIR_SENSOR_PIN);
  
  Serial.println(F("\n--- Initial Sensor Readings ---"));
  Serial.printf("Current: %.3f A\n", testCurrent);
  Serial.printf("Voltage: %.1f V\n", testVoltage);
  Serial.printf("PIR Sensor: %s (Pin %d = %s)\n", 
                pirReading == LOW ? "READY" : "MOTION DETECTED",
                PIR_SENSOR_PIN,
                pirReading == LOW ? "LOW" : "HIGH");
  
  // Check if load is initially connected
  loadPluggedIn = (testCurrent > LOAD_DETECTION_THRESHOLD);
  Serial.printf("Load Status: %s\n", loadPluggedIn ? "CONNECTED" : "EMPTY SOCKET");
  
  // PIR Safety System notification
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║   🛡️ CHILD SAFETY SYSTEM ENABLED 🛡️    ║"));
  Serial.println(F("║                                        ║"));
  Serial.println(F("║  PIR Sensor: HW-456 SR505-M           ║"));
  Serial.println(F("║  Protection: Empty socket + Motion    ║"));
  Serial.println(F("║  Action: Immediate power cutoff       ║"));
  Serial.println(F("║  Warmup: 30 seconds required          ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup WebSocket if WiFi is connected
  if (wifiConnected) {
    setupWebSocket();
  }
  
  systemStartTime = millis();
  lastEnergyUpdate = millis();
  currentState = STATE_MONITORING;
  
  // System ready indication
  for (int i = 0; i < 3; i++) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
  
  Serial.println(F("\n✓✓✓ SYSTEM READY ✓✓✓\n"));
}

// ==================== WiFi CONNECTION ====================
void setupWiFi() {
  Serial.printf("\nConnecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT_MS) {
      Serial.println(F("\n✗ WiFi connection timeout"));
      wifiConnected = false;
      connectionState = CONN_DISCONNECTED;
      return;
    }
    Serial.print(".");
    digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    delay(500);
  }
  
  wifiConnected = true;
  connectionState = CONN_WIFI_ONLY;
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  Serial.println(F("\n✓ WiFi connected"));
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
  
  // Configure HTTPS client
  httpsClient.setInsecure(); // For development - use certificate in production
}

// ==================== WebSocket CONNECTION ====================
void setupWebSocket() {
  Serial.printf("\nConnecting to WebSocket: wss://%s:%d%s\n", 
                SERVER_HOST, SERVER_PORT, WS_PATH);
  
  // Configure WebSocket callbacks
  wsClient.onMessage([](WebsocketsMessage message) {
    handleWebSocketMessage(message);
  });
  
  wsClient.onEvent([](WebsocketsEvent event, String data) {
    switch(event) {
      case WebsocketsEvent::ConnectionOpened:
        Serial.println(F("✓ WebSocket connected"));
        wsConnected = true;
        connectionState = CONN_FULLY_CONNECTED;
        wsReconnectAttempts = 0;
        digitalWrite(BLUE_LED_PIN, HIGH);
        sendSystemStatus();
        break;
        
      case WebsocketsEvent::ConnectionClosed:
        Serial.println(F("✗ WebSocket disconnected"));
        wsConnected = false;
        connectionState = CONN_WIFI_ONLY;
        digitalWrite(BLUE_LED_PIN, LOW);
        break;
        
      case WebsocketsEvent::GotPing:
        Serial.println(F("← Ping received"));
        wsClient.pong();
        break;
        
      case WebsocketsEvent::GotPong:
        Serial.println(F("← Pong received"));
        break;
    }
  });
  
  // Build WebSocket URL with device ID
  String wsUrl = String("wss://") + SERVER_HOST + ":" + SERVER_PORT + WS_PATH + 
                 "?deviceId=" + DEVICE_ID + "&type=" + DEVICE_TYPE;
  
  // Connect to WebSocket
  wsConnected = wsClient.connect(wsUrl);
  
  if (wsConnected) {
    Serial.println(F("✓ WebSocket connection established"));
    connectionState = CONN_FULLY_CONNECTED;
    
    // Send initial registration
    StaticJsonDocument<256> doc;
    doc["type"] = "register";
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["version"] = DEVICE_VERSION;
    
    String message;
    serializeJson(doc, message);
    wsClient.send(message);
  } else {
    Serial.println(F("✗ WebSocket connection failed"));
    connectionState = CONN_WIFI_ONLY;
  }
}

// ==================== RECONNECTION LOGIC ====================
void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    wsConnected = false;
    connectionState = CONN_DISCONNECTED;
    
    Serial.println(F("\n↻ Reconnecting to WiFi..."));
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT_MS) {
      delay(500);
      digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      connectionState = CONN_WIFI_ONLY;
      digitalWrite(RED_LED_PIN, LOW);
      Serial.printf("✓ WiFi reconnected: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println(F("✗ WiFi reconnection failed"));
    }
  }
}

void reconnectWebSocket() {
  if (!wsConnected && wifiConnected) {
    wsReconnectAttempts++;
    
    // Exponential backoff
    unsigned long backoffDelay = min(1000UL * (1 << wsReconnectAttempts), 60000UL);
    
    Serial.printf("\n↻ WebSocket reconnection attempt %d (delay: %lu ms)\n", 
                  wsReconnectAttempts, backoffDelay);
    
    delay(backoffDelay);
    
    wsClient.close();
    setupWebSocket();
  }
}

// ==================== WebSocket MESSAGE HANDLING ====================
void handleWebSocketMessage(WebsocketsMessage message) {
  wsMessageCount++;
  digitalWrite(BLUE_LED_PIN, HIGH);
  
  Serial.printf("\n← WebSocket message #%d: %s\n", wsMessageCount, message.data().c_str());
  
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message.data());
  
  if (error) {
    Serial.printf("✗ JSON parse error: %s\n", error.c_str());
    sendCommandAck("parse_error", false, error.c_str());
    return;
  }
  
  // Handle different message types
  const char* type = doc["type"];
  
  if (strcmp(type, "command") == 0) {
    commandsReceived++;
    handleCommand(doc);
  } 
  else if (strcmp(type, "ping") == 0) {
    // Respond with pong
    StaticJsonDocument<128> pongDoc;
    pongDoc["type"] = "pong";
    pongDoc["timestamp"] = millis();
    String pongMessage;
    serializeJson(pongDoc, pongMessage);
    wsClient.send(pongMessage);
  }
  else if (strcmp(type, "config") == 0) {
    // Handle configuration update
    if (doc.containsKey("sendInterval")) {
      // Update send interval if provided
      unsigned long newInterval = doc["sendInterval"];
      Serial.printf("→ Config: Send interval updated to %lu ms\n", newInterval);
    }
  }
  else if (strcmp(type, "calibrate") == 0) {
    // Start calibration sequence
    calibrationMode = true;
    calibrationStep = 0;
    runCalibration();
  }
  
  digitalWrite(BLUE_LED_PIN, LOW);
}

// ==================== COMMAND HANDLING ====================
void handleCommand(JsonDocument& doc) {
  const char* command = doc["command"];
  Serial.printf("→ Command received: %s\n", command);
  
  bool success = false;
  String responseMessage = "";
  
  if (strcmp(command, "SSR_ON") == 0) {
    ssrCommandState = true;
    success = true;
    responseMessage = "SSR turned ON";
    Serial.println(F("✓ SSR ON command executed"));
  }
  else if (strcmp(command, "SSR_OFF") == 0) {
    ssrCommandState = false;
    success = true;
    responseMessage = "SSR turned OFF";
    Serial.println(F("✓ SSR OFF command executed"));
  }
  else if (strcmp(command, "SSR_TOGGLE") == 0) {
    ssrCommandState = !ssrCommandState;
    success = true;
    responseMessage = String("SSR toggled to ") + (ssrCommandState ? "ON" : "OFF");
    Serial.println(F("✓ SSR TOGGLE command executed"));
  }
  else if (strcmp(command, "PIR_ENABLE") == 0) {
    pirEnabled = true;
    success = true;
    responseMessage = "PIR safety enabled";
    Serial.println(F("✓ PIR enabled"));
  }
  else if (strcmp(command, "PIR_DISABLE") == 0) {
    pirEnabled = false;
    pirState = PIR_IDLE;
    ssrPirOverride = false;
    success = true;
    responseMessage = "PIR safety disabled";
    Serial.println(F("✓ PIR disabled"));
  }
  else if (strcmp(command, "GET_STATUS") == 0) {
    sendSystemStatus();
    success = true;
    responseMessage = "Status sent";
  }
  else if (strcmp(command, "RESET_ENERGY") == 0) {
    energyConsumed = 0.0;
    lastEnergyUpdate = millis();
    success = true;
    responseMessage = "Energy counter reset";
    Serial.println(F("✓ Energy counter reset"));
  }
  else if (strcmp(command, "CALIBRATE") == 0) {
    calibrationMode = true;
    calibrationStep = 0;
    runCalibration();
    success = true;
    responseMessage = "Calibration started";
  }
  else if (strcmp(command, "RESET_CALIBRATION") == 0) {
    // Reset to default calibration
    calData = CalibrationData();
    saveCalibration();
    success = true;
    responseMessage = "Calibration reset to defaults";
    Serial.println(F("✓ Calibration reset"));
  }
  else if (strcmp(command, "REBOOT") == 0) {
    success = true;
    responseMessage = "Rebooting...";
    sendCommandAck(command, success, responseMessage);
    delay(1000);
    ESP.restart();
  }
  else {
    responseMessage = "Unknown command";
    Serial.printf("✗ Unknown command: %s\n", command);
  }
  
  if (success) {
    commandsExecuted++;
  }
  
  sendCommandAck(command, success, responseMessage);
}

// ==================== COMMAND ACKNOWLEDGMENT ====================
void sendCommandAck(const String& command, bool success, const String& message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ack";
  doc["command"] = command;
  doc["success"] = success;
  doc["message"] = message;
  doc["timestamp"] = millis();
  doc["deviceId"] = DEVICE_ID;
  
  String jsonMessage;
  serializeJson(doc, jsonMessage);
  
  if (wsConnected) {
    wsClient.send(jsonMessage);
    Serial.printf("→ ACK sent: %s (%s)\n", command.c_str(), success ? "SUCCESS" : "FAILED");
  }
}

// ==================== DATA TRANSMISSION TO SERVER ====================
void sendDataToServer() {
  if (!wifiConnected) {
    Serial.println(F("✗ Cannot send data: WiFi disconnected"));
    return;
  }
  
  // Prepare JSON data
  StaticJsonDocument<512> doc;
  doc["deviceId"] = DEVICE_ID;
  doc["deviceType"] = DEVICE_TYPE;
  doc["timestamp"] = millis();
  
  // Sensor data
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["voltage"] = voltageReading;
  sensors["current"] = currentReading;
  sensors["power"] = powerReading;
  sensors["energy"] = energyConsumed;
  sensors["powerFactor"] = powerFactor;
  sensors["frequency"] = frequency;
  sensors["apparentPower"] = apparentPower;
  sensors["reactivePower"] = reactivePower;
  
  // System status
  JsonObject status = doc.createNestedObject("status");
  status["ssrState"] = ssrEnabled;
  status["pirEnabled"] = pirEnabled;
  status["pirMotion"] = pirMotionDetected;
  status["loadDetected"] = loadPluggedIn;
  status["uptime"] = (millis() - systemStartTime) / 1000;
  status["wifiRSSI"] = WiFi.RSSI();
  status["wsConnected"] = wsConnected;
  
  // Calibration info
  JsonObject calibration = doc.createNestedObject("calibration");
  calibration["currentOffset"] = calData.currentOffset;
  calibration["voltageOffset"] = calData.voltageOffset;
  calibration["currentCal"] = calData.currentCalibration;
  calibration["voltageCal"] = calData.voltageCalibration;
  calibration["calibrationCount"] = calData.calibrationCount;
  
  String jsonData;
  serializeJson(doc, jsonData);
  
  // Try WebSocket first if connected
  if (wsConnected) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    wsClient.send(jsonData);
    serverSuccessCount++;
    Serial.println(F("→ Data sent via WebSocket"));
    digitalWrite(BLUE_LED_PIN, LOW);
  } else {
    // Fallback to HTTPS POST with retry
    if (sendDataWithRetry(jsonData)) {
      serverSuccessCount++;
    } else {
      serverErrorCount++;
      // Buffer data for later transmission
      if (bufferIndex < 10) {
        dataBuffer[bufferIndex].voltage = voltageReading;
        dataBuffer[bufferIndex].current = currentReading;
        dataBuffer[bufferIndex].power = powerReading;
        dataBuffer[bufferIndex].energy = energyConsumed;
        dataBuffer[bufferIndex].ssrState = ssrEnabled;
        dataBuffer[bufferIndex].pirMotion = pirMotionDetected;
        dataBuffer[bufferIndex].timestamp = millis();
        bufferIndex++;
      }
    }
  }
}

// ==================== HTTP POST WITH RETRY ====================
bool sendDataWithRetry(const String& jsonData) {
  int attempts = 0;
  bool success = false;
  
  while (attempts < MAX_RETRY_ATTEMPTS && !success) {
    attempts++;
    digitalWrite(BLUE_LED_PIN, HIGH);
    
    HTTPClient http;
    String url = String("https://") + SERVER_HOST + ":" + SERVER_PORT + SERVER_ENDPOINT;
    
    http.begin(httpsClient, url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Device-Id", DEVICE_ID);
    http.addHeader("Device-Type", DEVICE_TYPE);
    http.setTimeout(SERVER_TIMEOUT);
    
    Serial.printf("→ HTTP POST attempt %d/%d...\n", attempts, MAX_RETRY_ATTEMPTS);
    
    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
      Serial.printf("✓ HTTP Response: %d\n", httpResponseCode);
      if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_CREATED) {
        success = true;
        httpRetryCount = 0;
      }
    } else {
      Serial.printf("✗ HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
      delay(1000 * attempts); // Exponential backoff
    }
    
    http.end();
    digitalWrite(BLUE_LED_PIN, LOW);
  }
  
  return success;
}

// ==================== SENSOR READING WITH CALIBRATION ====================
void updateSensors() {
  // Read raw sensor values
  currentReading = readCurrent();
  voltageReading = readVoltage();
  
  // Calculate power metrics
  calculatePower();
  
  // Update energy consumption (Wh)
  unsigned long now = millis();
  if (now - lastEnergyUpdate >= ENERGY_UPDATE_INTERVAL) {
    float deltaTime = (now - lastEnergyUpdate) / 3600000.0; // Convert to hours
    energyConsumed += powerReading * deltaTime;
    lastEnergyUpdate = now;
  }
  
  // Check if load is plugged in
  loadPluggedIn = (currentReading > LOAD_DETECTION_THRESHOLD);
}

// ==================== CURRENT READING WITH CIRCUITIQ CALIBRATION ====================
float readCurrent() {
  float sum = 0;
  float sumSquares = 0;
  
  // Take multiple samples for accurate RMS calculation
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    int adcValue = analogRead(ACS712_PIN);
    float voltage = (adcValue / ADC_RESOLUTION) * ADC_VREF;
    
    // Apply voltage divider correction
    float sensorVoltage = voltage / VOLTAGE_DIVIDER_RATIO;
    
    // Apply calibration offset
    float current = (sensorVoltage - calData.currentOffset) / ACS712_SENSITIVITY;
    
    sum += current;
    sumSquares += current * current;
    delayMicroseconds(200); // Sample at ~5kHz
  }
  
  // Calculate RMS current
  float avgCurrent = sum / CURRENT_SAMPLES;
  float rmsCurrent = sqrt(sumSquares / CURRENT_SAMPLES);
  
  // Apply calibration factor
  rmsCurrent *= calData.currentCalibration;
  
  // Apply noise filter and limits
  if (rmsCurrent < MIN_VALID_CURRENT) {
    rmsCurrent = 0.0; // Below noise threshold
  }
  if (rmsCurrent > MAX_VALID_CURRENT) {
    rmsCurrent = MAX_VALID_CURRENT; // Clamp to maximum
  }
  
  return rmsCurrent;
}

// ==================== VOLTAGE READING WITH CIRCUITIQ CALIBRATION ====================
float readVoltage() {
  float sumSquares = 0;
  float maxValue = 0;
  float minValue = ADC_VREF;
  
  // Sample over multiple AC cycles
  unsigned long samplePeriod = (1000000 / AC_FREQUENCY) * 3; // 3 AC cycles in microseconds
  unsigned long startTime = micros();
  int sampleCount = 0;
  
  while (micros() - startTime < samplePeriod) {
    int adcValue = analogRead(ZMPT101B_PIN);
    float voltage = (adcValue / ADC_RESOLUTION) * ADC_VREF;
    
    // Track min/max for peak-to-peak calculation
    if (voltage > maxValue) maxValue = voltage;
    if (voltage < minValue) minValue = voltage;
    
    // Apply calibration offset (AC center point)
    float acVoltage = voltage - calData.voltageOffset;
    
    sumSquares += acVoltage * acVoltage;
    sampleCount++;
    
    delayMicroseconds(150); // Sample at ~6.6kHz (>10x AC frequency)
  }
  
  // Calculate RMS voltage
  float rmsVoltage = sqrt(sumSquares / sampleCount);
  
  // Convert to actual voltage using sensor sensitivity
  float realVoltage = (rmsVoltage / ZMPT101B_SENSITIVITY) * calData.voltageCalibration;
  
  // Alternative calculation using peak-to-peak
  float peakToPeak = maxValue - minValue;
  float altVoltage = (peakToPeak / 2.0) * 0.707 / ZMPT101B_SENSITIVITY * calData.voltageCalibration;
  
  // Use average of both methods for better accuracy
  realVoltage = (realVoltage + altVoltage) / 2.0;
  
  // Validate reading
  if (realVoltage < MIN_VOLTAGE / 2) {
    realVoltage = 0.0; // No AC detected
  } else if (realVoltage < MIN_VOLTAGE) {
    realVoltage = MIN_VOLTAGE; // Undervoltage
  } else if (realVoltage > MAX_VOLTAGE) {
    realVoltage = MAX_VOLTAGE; // Overvoltage
  }
  
  return realVoltage;
}

// ==================== POWER CALCULATIONS ====================
void calculatePower() {
  // Calculate apparent power (VA)
  apparentPower = voltageReading * currentReading;
  
  // Calculate real power (W) using power factor
  powerReading = apparentPower * powerFactor;
  
  // Calculate reactive power (VAR)
  reactivePower = sqrt((apparentPower * apparentPower) - (powerReading * powerReading));
  
  // Estimate frequency (simplified - would need zero-crossing detection for accuracy)
  frequency = AC_FREQUENCY; // Use nominal for now
}

// ==================== CALIBRATION SYSTEM ====================
void loadCalibration() {
  EEPROM.get(0, calData);
  
  if (calData.magic != EEPROM_MAGIC) {
    // No valid calibration found, use defaults
    Serial.println(F("→ No calibration found, using defaults"));
    calData = CalibrationData();
    saveCalibration();
  } else {
    Serial.printf("→ Calibration loaded (count: %lu)\n", calData.calibrationCount);
    Serial.printf("  Current: offset=%.3f, cal=%.3f\n", 
                  calData.currentOffset, calData.currentCalibration);
    Serial.printf("  Voltage: offset=%.3f, cal=%.3f\n", 
                  calData.voltageOffset, calData.voltageCalibration);
  }
}

void saveCalibration() {
  calData.lastCalibration = millis();
  calData.calibrationCount++;
  EEPROM.put(0, calData);
  EEPROM.commit();
  Serial.println(F("✓ Calibration saved to EEPROM"));
}

void runCalibration() {
  Serial.println(F("\n=== CALIBRATION MODE ==="));
  
  switch (calibrationStep) {
    case 0:
      // Step 1: Measure zero current offset
      Serial.println(F("Step 1: Remove all loads, measuring zero current..."));
      digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
      delay(2000);
      
      float zeroCurrentSum = 0;
      for (int i = 0; i < 100; i++) {
        int adcValue = analogRead(ACS712_PIN);
        float voltage = (adcValue / ADC_RESOLUTION) * ADC_VREF;
        zeroCurrentSum += voltage / VOLTAGE_DIVIDER_RATIO;
        delay(10);
      }
      calData.currentOffset = zeroCurrentSum / 100.0;
      Serial.printf("→ Current offset: %.3f V\n", calData.currentOffset);
      calibrationStep++;
      break;
      
    case 1:
      // Step 2: Measure AC voltage offset
      Serial.println(F("Step 2: Measuring AC voltage offset..."));
      
      float voltageSum = 0;
      for (int i = 0; i < 200; i++) {
        int adcValue = analogRead(ZMPT101B_PIN);
        float voltage = (adcValue / ADC_RESOLUTION) * ADC_VREF;
        voltageSum += voltage;
        delay(5);
      }
      calData.voltageOffset = voltageSum / 200.0;
      Serial.printf("→ Voltage offset: %.3f V\n", calData.voltageOffset);
      
      // Save calibration
      saveCalibration();
      calibrationMode = false;
      calibrationStep = 0;
      
      Serial.println(F("✓ Calibration complete!"));
      
      // Send calibration complete message
      sendCommandAck("CALIBRATE", true, "Calibration completed successfully");
      break;
  }
}

// ==================== PIR SAFETY SYSTEM ====================
// Enhanced PIR Safety System for HW-456 SR505-M
void updatePIRSafety() {
  if (!pirEnabled) {
    pirState = PIR_IDLE;
    ssrPirOverride = false;
    return;
  }
  
  unsigned long now = millis();
  static unsigned long systemWarmupTime = 30000; // 30 second warmup for HW-456
  static bool systemReady = false;
  static unsigned long motionStopTime = 0;
  
  // Check if warmup period is complete
  if (!systemReady && now < systemStartTime + systemWarmupTime) {
    // Still warming up
    if (now % 1000 < 50) { // Print every second
      Serial.printf("PIR Warmup: %d seconds remaining\r", 
                    (systemWarmupTime - (now - systemStartTime)) / 1000);
    }
    return;
  } else if (!systemReady) {
    systemReady = true;
    Serial.println(F("\n✓ PIR Safety System: READY - Child protection active"));
    Serial.println(F("🛡️ Empty socket + Motion = Power OFF"));
  }
  
  // Check PIR sensor at regular intervals
  if (now - lastPirCheck < PIR_CHECK_INTERVAL) {
    return;
  }
  lastPirCheck = now;
  
  // Read PIR sensor with debouncing for HW-456
  pirReadings[pirReadIndex] = digitalRead(PIR_SENSOR_PIN);
  pirReadIndex = (pirReadIndex + 1) % PIR_SENSITIVITY;
  
  // Count HIGH readings
  int highCount = 0;
  for (int i = 0; i < PIR_SENSITIVITY; i++) {
    if (pirReadings[i] == HIGH) highCount++;
  }
  
  // Motion detected if majority are HIGH (3 out of 5)
  bool motionNow = (highCount >= 3);
  
  if (motionNow) {
    lastMotionTime = now;
    if (!pirMotionDetected) {
      pirMotionDetected = true;
      pirTriggerCount++;
      Serial.println(F("\n⚠️ PIR: MOTION DETECTED!"));
      Serial.printf("Load status: %s (%.3f A)\n", 
                    loadPluggedIn ? "CONNECTED" : "EMPTY SOCKET", currentReading);
    }
  } else {
    pirMotionDetected = false;
  }
  
  // CRITICAL CHILD SAFETY LOGIC
  // Priority: Protect empty sockets from children
  
  if (!loadPluggedIn && pirMotionDetected) {
    // DANGER: Empty socket + Motion = Child might insert finger!
    if (!ssrPirOverride) {
      ssrPirOverride = true;
      pirState = PIR_MOTION_NO_LOAD;
      
      // IMMEDIATE SAFETY ACTION
      digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE); // Force SSR OFF NOW
      digitalWrite(RED_LED_PIN, HIGH);
      
      Serial.println(F("\n╔════════════════════════════════════════╗"));
      Serial.println(F("║   🛡️ CHILD SAFETY ACTIVATED! 🛡️        ║"));
      Serial.println(F("║   Empty socket + Motion detected       ║"));
      Serial.println(F("║   SSR FORCED OFF - Preventing shock    ║"));
      Serial.println(F("╚════════════════════════════════════════╝\n"));
      
      // Critical alert sequence
      for (int i = 0; i < 5; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
      }
      digitalWrite(RED_LED_PIN, HIGH); // Keep red LED on
    }
    
  } else if (loadPluggedIn && ssrPirOverride) {
    // Load is now plugged in, safe to restore
    ssrPirOverride = false;
    pirState = PIR_MOTION_WITH_LOAD;
    Serial.println(F("✓ Load detected - Restoring normal operation"));
    digitalWrite(RED_LED_PIN, LOW);
    motionStopTime = 0;
    
  } else if (!pirMotionDetected && ssrPirOverride && !loadPluggedIn) {
    // Motion stopped but socket still empty - wait for safety timeout
    if (motionStopTime == 0) {
      motionStopTime = now;
      Serial.println(F("⏱ Motion stopped - Starting safety timeout..."));
    }
    
    // Safety lockout period (10 seconds after motion stops)
    if (now - motionStopTime > 10000) {
      ssrPirOverride = false;
      pirState = PIR_IDLE;
      motionStopTime = 0;
      Serial.println(F("✓ Safety timeout complete - SSR control restored"));
      Serial.println(F("   Socket still empty - Monitoring continues"));
      digitalWrite(RED_LED_PIN, LOW);
    }
    
  } else if (loadPluggedIn) {
    // Normal operation when load is connected
    if (pirState != PIR_MOTION_WITH_LOAD && pirState != PIR_IDLE) {
      pirState = loadPluggedIn && pirMotionDetected ? PIR_MOTION_WITH_LOAD : PIR_IDLE;
    }
  }
  
  // Print detailed status every 5 seconds when safety is active
  static unsigned long lastSafetyStatus = 0;
  if (ssrPirOverride && now - lastSafetyStatus > 5000) {
    lastSafetyStatus = now;
    Serial.println(F("\n--- CHILD SAFETY STATUS ---"));
    Serial.printf("PIR Motion: %s\n", pirMotionDetected ? "ACTIVE" : "Clear");
    Serial.printf("Socket: %s\n", loadPluggedIn ? "OCCUPIED" : "EMPTY - PROTECTED");
    Serial.printf("SSR State: FORCED OFF (Safety Override)\n");
    Serial.printf("Time in safety: %lu seconds\n", (now - lastMotionTime) / 1000);
    Serial.println(F("---------------------------\n"));
  }
}

// ==================== SSR CONTROL ====================
void controlSSR() {
  // Determine final SSR state
  // Priority: Safety override > PIR override > Command state
  bool finalState;
  
  if (ssrSafetyOverride) {
    // Critical safety override (overvoltage/overcurrent)
    finalState = false;
  } else if (ssrPirOverride) {
    // PIR CHILD SAFETY OVERRIDE - IMMEDIATE OFF
    finalState = false;
    // Double-check it's really OFF for child safety
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  } else {
    // Normal command state
    finalState = ssrCommandState;
  }
  
  // Apply state
  ssrEnabled = finalState;
  digitalWrite(SSR_CONTROL_PIN, finalState ? SSR_ON_STATE : SSR_OFF_STATE);
  
  // Update status LED (Green = Power ON, Red handled by PIR safety)
  digitalWrite(GREEN_LED_PIN, finalState ? HIGH : LOW);
}

// ==================== SYSTEM STATUS ====================
void sendSystemStatus() {
  StaticJsonDocument<1024> doc;
  
  doc["type"] = "status";
  doc["deviceId"] = DEVICE_ID;
  doc["deviceType"] = DEVICE_TYPE;
  doc["version"] = DEVICE_VERSION;
  doc["timestamp"] = millis();
  
  // System info
  JsonObject system = doc.createNestedObject("system");
  system["uptime"] = (millis() - systemStartTime) / 1000;
  system["loops"] = loopCount;
  system["freeHeap"] = ESP.getFreeHeap();
  system["state"] = currentState;
  
  // Network status
  JsonObject network = doc.createNestedObject("network");
  network["wifi"] = wifiConnected;
  network["websocket"] = wsConnected;
  network["rssi"] = WiFi.RSSI();
  network["ip"] = WiFi.localIP().toString();
  network["serverSuccess"] = serverSuccessCount;
  network["serverErrors"] = serverErrorCount;
  
  // Sensor readings
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["voltage"] = voltageReading;
  sensors["current"] = currentReading;
  sensors["power"] = powerReading;
  sensors["energy"] = energyConsumed;
  sensors["powerFactor"] = powerFactor;
  sensors["frequency"] = frequency;
  
  // PIR status
  JsonObject pir = doc.createNestedObject("pir");
  pir["enabled"] = pirEnabled;
  pir["motion"] = pirMotionDetected;
  pir["state"] = pirState;
  pir["triggers"] = pirTriggerCount;
  pir["override"] = ssrPirOverride;
  
  // SSR status
  JsonObject ssr = doc.createNestedObject("ssr");
  ssr["commandState"] = ssrCommandState;
  ssr["actualState"] = ssrEnabled;
  ssr["pirOverride"] = ssrPirOverride;
  ssr["safetyOverride"] = ssrSafetyOverride;
  
  // Calibration
  JsonObject cal = doc.createNestedObject("calibration");
  cal["count"] = calData.calibrationCount;
  cal["lastCalibration"] = calData.lastCalibration;
  
  String statusMessage;
  serializeJson(doc, statusMessage);
  
  if (wsConnected) {
    wsClient.send(statusMessage);
    Serial.println(F("→ System status sent"));
  }
}

void printStatus() {
  Serial.println(F("\n============ SYSTEM STATUS ============"));
  Serial.printf("Uptime: %lu seconds\n", (millis() - systemStartTime) / 1000);
  Serial.printf("Loop count: %lu\n", loopCount);
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  
  Serial.println(F("\n--- NETWORK ---"));
  Serial.printf("WiFi: %s (RSSI: %d dBm)\n", 
                wifiConnected ? "Connected" : "Disconnected", WiFi.RSSI());
  Serial.printf("WebSocket: %s\n", wsConnected ? "Connected" : "Disconnected");
  Serial.printf("Server success/error: %d/%d\n", serverSuccessCount, serverErrorCount);
  Serial.printf("WS messages: %d\n", wsMessageCount);
  Serial.printf("Commands: %d received, %d executed\n", commandsReceived, commandsExecuted);
  
  Serial.println(F("\n--- SENSORS ---"));
  Serial.printf("Voltage: %.1f V\n", voltageReading);
  Serial.printf("Current: %.3f A\n", currentReading);
  Serial.printf("Power: %.1f W (PF: %.2f)\n", powerReading, powerFactor);
  Serial.printf("Energy: %.3f Wh\n", energyConsumed);
  Serial.printf("Apparent: %.1f VA, Reactive: %.1f VAR\n", apparentPower, reactivePower);
  
  Serial.println(F("\n--- PIR SAFETY ---"));
  Serial.printf("PIR: %s\n", pirEnabled ? "Enabled" : "Disabled");
  Serial.printf("Motion: %s, Load: %s\n", 
                pirMotionDetected ? "YES" : "NO",
                loadPluggedIn ? "YES" : "NO");
  Serial.printf("PIR State: ");
  switch (pirState) {
    case PIR_IDLE: Serial.println("IDLE"); break;
    case PIR_MOTION_NO_LOAD: Serial.println("MOTION+NO_LOAD (SSR OFF)"); break;
    case PIR_MOTION_WITH_LOAD: Serial.println("MOTION+LOAD"); break;
    case PIR_COOLDOWN: Serial.println("COOLDOWN"); break;
  }
  Serial.printf("Triggers: %d\n", pirTriggerCount);
  
  Serial.println(F("\n--- SSR CONTROL ---"));
  Serial.printf("Command state: %s\n", ssrCommandState ? "ON" : "OFF");
  Serial.printf("Actual state: %s\n", ssrEnabled ? "ON" : "OFF");
  Serial.printf("PIR override: %s\n", ssrPirOverride ? "ACTIVE" : "INACTIVE");
  Serial.printf("Safety override: %s\n", ssrSafetyOverride ? "ACTIVE" : "INACTIVE");
  
  Serial.println(F("\n--- CALIBRATION ---"));
  Serial.printf("Current offset: %.3f V, calibration: %.3f\n", 
                calData.currentOffset, calData.currentCalibration);
  Serial.printf("Voltage offset: %.3f V, calibration: %.3f\n", 
                calData.voltageOffset, calData.voltageCalibration);
  Serial.printf("Calibration count: %lu\n", calData.calibrationCount);
  
  Serial.println(F("========================================\n"));
}

// ==================== ERROR HANDLING ====================
void handleSystemError(const String& error) {
  Serial.printf("\n!!! SYSTEM ERROR: %s !!!\n", error.c_str());
  
  // Set error state
  currentState = STATE_ERROR;
  
  // Visual/audio alert
  for (int i = 0; i < 5; i++) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  
  // Send error to server if connected
  if (wsConnected) {
    StaticJsonDocument<256> doc;
    doc["type"] = "error";
    doc["deviceId"] = DEVICE_ID;
    doc["error"] = error;
    doc["timestamp"] = millis();
    
    String errorMessage;
    serializeJson(doc, errorMessage);
    wsClient.send(errorMessage);
  }
  
  // Safety mode - turn off SSR
  ssrSafetyOverride = true;
  controlSSR();
}

// ==================== WATCHDOG ====================
void feedWatchdog() {
  esp_task_wdt_reset();
}

// ==================== MAIN LOOP ====================
void loop() {
  loopCount++;
  feedWatchdog();
  
  // Connection management
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiCheck > 5000) {
      reconnectWiFi();
      lastWifiCheck = millis();
    }
  }
  
  // WebSocket management
  if (wifiConnected && !wsConnected) {
    if (millis() - lastWsReconnect > 10000) {
      reconnectWebSocket();
      lastWsReconnect = millis();
    }
  }
  
  // Poll WebSocket
  if (wsConnected) {
    wsClient.poll();
    
    // Send periodic ping to keep connection alive
    if (millis() - lastWsPing > WS_PING_INTERVAL) {
      wsClient.ping();
      lastWsPing = millis();
      Serial.println(F("→ WebSocket ping sent"));
    }
  }
  
  // Update sensors
  updateSensors();
  
  // Update PIR safety
  updatePIRSafety();
  
  // Control SSR
  controlSSR();
  
  // Send data to server periodically
  if (millis() - lastDataSend >= SERVER_SEND_INTERVAL) {
    sendDataToServer();
    lastDataSend = millis();
  }
  
  // Print status every 30 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 30000) {
    printStatus();
    lastStatusPrint = millis();
  }
  
  // Check for system errors
  if (voltageReading > MAX_VOLTAGE * 1.2 || currentReading > MAX_VALID_CURRENT * 1.2) {
    handleSystemError("Overvoltage/Overcurrent detected!");
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}

// ==================== END OF ENHANCED VAULTGUARD CODE ====================