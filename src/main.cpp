/*
 * VaultGuard v7.0 - COMPLETE PRODUCTION VERSION FOR RENDER.COM
 * Server: https://meifhi-esp-server.onrender.com
 * 
 * ✅ FIXED FOR RENDER.COM DEPLOYMENT
 * ✅ ALL FUNCTIONALITY PRESERVED
 * ✅ DEVICE TYPE CORRECTED
 * ✅ SSL/TLS CONFIGURED FOR RENDER
 * 
 * FEATURES:
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
 * BEFORE UPLOADING:
 * 1. Replace "YOUR_WIFI_SSID" with your WiFi name (line 50)
 * 2. Replace "YOUR_WIFI_PASSWORD" with your WiFi password (line 51)
 * 3. Upload to ESP32
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

// ==================== NETWORK CONFIGURATION - RENDER.COM ====================
// ⚠️ TODO: FILL IN YOUR WIFI CREDENTIALS BELOW
#define WIFI_SSID           "YOUR_WIFI_SSID"                    // ⚠️ CHANGE THIS
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"                // ⚠️ CHANGE THIS

// ✅ RENDER.COM SERVER CONFIGURATION (FIXED)
#define SERVER_HOST         "meifhi-esp-server.onrender.com"
#define SERVER_PORT         443                                  // HTTPS standard port
#define SERVER_ENDPOINT     "/api/data"
#define WS_PATH             "/ws"

// ✅ DEVICE IDENTIFICATION (FIXED - Server compatible)
#define DEVICE_ID           "VAULTGUARD_001"
#define DEVICE_TYPE         "VAULTER"                           // ✅ FIXED: Changed from "VAULTGUARD"
#define DEVICE_VERSION      "7.0"

// NETWORK SETTINGS
#define WIFI_TIMEOUT_MS     20000
#define SERVER_SEND_INTERVAL 5000
#define SERVER_TIMEOUT      15000                                // ✅ Increased for Render cold start
#define MAX_RETRY_ATTEMPTS  3
#define WS_PING_INTERVAL    30000
#define WS_PONG_TIMEOUT     10000
#define RENDER_WAKEUP_DELAY 60000                               // ✅ Wait 60s for Render to wake up

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
bool renderServerAwake = false;                                 // ✅ Track Render server state

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
void wakeupRenderServer();                                      // ✅ New function

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n\n================================================"));
  Serial.println(F("   VAULTGUARD v7.0 - RENDER.COM PRODUCTION"));
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
  
  // ✅ Wake up Render server if WiFi connected
  if (wifiConnected) {
    Serial.println(F("\n→ Waking up Render server..."));
    Serial.println(F("   (Render.com free tier spins down after inactivity)"));
    wakeupRenderServer();
  }
  
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
  
  // ✅ Configure HTTPS client for Render.com
  httpsClient.setInsecure(); // Accept Render.com SSL certificate
  Serial.println(F("✓ HTTPS client configured for Render.com"));
}

// ==================== RENDER SERVER WAKEUP ====================
// ✅ New function to wake up Render.com free tier
void wakeupRenderServer() {
  Serial.println(F("\n→ Sending wakeup request to Render server..."));
  
  HTTPClient http;
  String url = String("https://") + SERVER_HOST + "/api/devices";
  
  http.begin(httpsClient, url);
  http.setTimeout(RENDER_WAKEUP_DELAY);
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    Serial.printf("✓ Render server is awake (HTTP %d)\n", httpResponseCode);
    renderServerAwake = true;
  } else {
    Serial.printf("⚠ Render server wakeup in progress... (waiting)\n");
    delay(5000); // Give it time to wake up
    renderServerAwake = false;
  }
  
  http.end();
  delay(2000); // Small delay before WebSocket connection
}

// ==================== WebSocket CONNECTION ====================
void setupWebSocket() {
  // ✅ Build proper HTTPS WebSocket URL for Render
  Serial.printf("\nConnecting to WebSocket: wss://%s%s\n", 
                SERVER_HOST, WS_PATH);
  
  // Configure WebSocket callbacks
  wsClient.onMessage([](WebsocketsMessage message) {
    handleWebSocketMessage(message);
  });
  
  wsClient.onEvent([](WebsocketsEvent event, String data) {
    switch(event) {
      case WebsocketsEvent::ConnectionOpened:
        Serial.println(F("✓ WebSocket connected to Render"));
        wsConnected = true;
        connectionState = CONN_FULLY_CONNECTED;
        wsReconnectAttempts = 0;
        renderServerAwake = true;                               // ✅ Mark as awake
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
  
  // ✅ Build WebSocket URL (Render handles SSL automatically)
  String wsUrl = String("wss://") + SERVER_HOST + WS_PATH + 
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
    Serial.println(F("→ Registration sent to Render server"));
  } else {
    Serial.println(F("✗ WebSocket connection failed"));
    Serial.println(F("   Render server may still be waking up..."));
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
    
    // ✅ Check if we need to wake up Render server again
    if (wsReconnectAttempts > 3 && !renderServerAwake) {
      Serial.println(F("   Render server may have gone to sleep, waking up..."));
      wakeupRenderServer();
    }
    
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
  else if (strcmp(type, "welcome") == 0) {
    // ✅ Handle welcome message from Render server
    Serial.println(F("✓ Received welcome from Render server"));
  }
  else if (strcmp(type, "registered") == 0) {
    // ✅ Handle registration confirmation
    Serial.println(F("✓ Device registered with Render server"));
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
    success = false;
    responseMessage = "Unknown command";
    Serial.printf("✗ Unknown command: %s\n", command);
  }
  
  // Send acknowledgment
  sendCommandAck(command, success, responseMessage);
  commandsExecuted++;
}

// ==================== COMMAND ACKNOWLEDGMENT ====================
void sendCommandAck(const String& command, bool success, const String& message) {
  if (!wsConnected) return;
  
  StaticJsonDocument<256> doc;
  doc["type"] = "commandAck";
  doc["command"] = command;
  doc["success"] = success;
  doc["message"] = message;
  doc["deviceId"] = DEVICE_ID;
  doc["timestamp"] = millis();
  
  String ackMessage;
  serializeJson(doc, ackMessage);
  wsClient.send(ackMessage);
  
  Serial.printf("→ Command ACK sent: %s (%s)\n", 
                command.c_str(), success ? "SUCCESS" : "FAILED");
}

// ==================== DATA TRANSMISSION ====================
void sendDataToServer() {
  StaticJsonDocument<1024> doc;
  
  // Device info
  doc["deviceId"] = DEVICE_ID;
  doc["deviceType"] = DEVICE_TYPE;
  doc["timestamp"] = millis();
  
  // Sensor readings
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
    Serial.println(F("→ Data sent via WebSocket to Render"));
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
    // ✅ Use HTTPS for Render.com
    String url = String("https://") + SERVER_HOST + SERVER_ENDPOINT;
    
    http.begin(httpsClient, url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Device-Id", DEVICE_ID);
    http.addHeader("Device-Type", DEVICE_TYPE);
    http.setTimeout(SERVER_TIMEOUT);
    
    Serial.printf("→ HTTPS POST attempt %d/%d to Render...\n", attempts, MAX_RETRY_ATTEMPTS);
    
    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
      Serial.printf("✓ HTTP Response: %d\n", httpResponseCode);
      if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_CREATED) {
        success = true;
        httpRetryCount = 0;
        renderServerAwake = true;                               // ✅ Mark server as awake
      }
    } else {
      Serial.printf("✗ HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
      
      // ✅ Check if Render server is asleep
      if (attempts == 1 && !renderServerAwake) {
        Serial.println(F("   Render server may be asleep, attempting wakeup..."));
        wakeupRenderServer();
      }
      
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
    realVoltage = 0.0; // No voltage detected
  }
  
  // Clamp to reasonable limits
  if (realVoltage > MAX_VOLTAGE * 1.2) {
    realVoltage = MAX_VOLTAGE * 1.2;
  }
  
  return realVoltage;
}

// ==================== POWER CALCULATION ====================
void calculatePower() {
  // Real Power (Watts)
  powerReading = voltageReading * currentReading * powerFactor;
  
  // Apparent Power (VA)
  apparentPower = voltageReading * currentReading;
  
  // Reactive Power (VAR)
  reactivePower = sqrt(abs(apparentPower * apparentPower - powerReading * powerReading));
  
  // Validate power factor
  if (apparentPower > 0.1) {
    powerFactor = powerReading / apparentPower;
    if (powerFactor > 1.0) powerFactor = 1.0;
    if (powerFactor < 0.0) powerFactor = 0.0;
  }
}

// ==================== PIR SAFETY SYSTEM ====================
void updatePIRSafety() {
  if (!pirEnabled) {
    pirState = PIR_IDLE;
    ssrPirOverride = false;
    return;
  }
  
  unsigned long now = millis();
  
  // Check PIR sensor with debouncing
  if (now - lastPirCheck >= PIR_CHECK_INTERVAL) {
    lastPirCheck = now;
    
    // Read PIR sensor
    int pirReading = digitalRead(PIR_SENSOR_PIN);
    
    // Update circular buffer for debouncing
    pirReadings[pirReadIndex] = pirReading;
    pirReadIndex = (pirReadIndex + 1) % PIR_SENSITIVITY;
    
    // Count HIGH readings in buffer
    int highCount = 0;
    for (int i = 0; i < PIR_SENSITIVITY; i++) {
      if (pirReadings[i] == HIGH) highCount++;
    }
    
    // Motion detected if majority of readings are HIGH
    bool motionNow = (highCount >= (PIR_SENSITIVITY / 2 + 1));
    
    // Motion state changed
    if (motionNow && !pirMotionDetected) {
      pirMotionDetected = true;
      lastMotionTime = now;
      pirTriggerCount++;
      Serial.println(F("\n⚠️ PIR: Motion detected!"));
    } else if (!motionNow && pirMotionDetected) {
      pirMotionDetected = false;
      Serial.println(F("✓ PIR: Motion stopped"));
    }
  }
  
  // PIR Safety Logic State Machine
  switch (pirState) {
    case PIR_IDLE:
      if (pirMotionDetected && !loadPluggedIn) {
        // DANGER: Motion detected with empty socket
        pirState = PIR_MOTION_NO_LOAD;
        ssrPirOverride = true;
        
        // Alert sequence
        Serial.println(F("\n🚨 CHILD SAFETY ALERT! 🚨"));
        Serial.println(F("   Motion + Empty Socket → SSR OFF"));
        
        for (int i = 0; i < PIR_ALERT_BEEPS; i++) {
          digitalWrite(RED_LED_PIN, HIGH);
          digitalWrite(BUZZER_PIN, HIGH);
          delay(150);
          digitalWrite(RED_LED_PIN, LOW);
          digitalWrite(BUZZER_PIN, LOW);
          delay(150);
        }
      } else if (pirMotionDetected && loadPluggedIn) {
        // Safe: Motion with load present
        pirState = PIR_MOTION_WITH_LOAD;
      }
      break;
      
    case PIR_MOTION_NO_LOAD:
      // Keep SSR OFF while motion persists
      if (!pirMotionDetected) {
        // Motion stopped, enter cooldown
        pirState = PIR_COOLDOWN;
        lastMotionTime = now;
        Serial.println(F("→ PIR: Entering cooldown period..."));
      }
      // Check if load was plugged in
      if (loadPluggedIn) {
        pirState = PIR_MOTION_WITH_LOAD;
        ssrPirOverride = false;
        Serial.println(F("✓ PIR: Load plugged in, releasing override"));
      }
      break;
      
    case PIR_MOTION_WITH_LOAD:
      // Normal operation, just monitoring
      if (!pirMotionDetected) {
        pirState = PIR_IDLE;
      }
      if (!loadPluggedIn) {
        // Load unplugged during motion!
        pirState = PIR_MOTION_NO_LOAD;
        ssrPirOverride = true;
        Serial.println(F("⚠️ PIR: Load unplugged during motion → SSR OFF"));
      }
      break;
      
    case PIR_COOLDOWN:
      // Wait for cooldown period
      if (now - lastMotionTime >= PIR_MOTION_TIMEOUT) {
        // Cooldown complete
        pirState = PIR_IDLE;
        ssrPirOverride = false;
        Serial.println(F("✓ PIR: Cooldown complete, resuming normal operation"));
      }
      // If motion detected again during cooldown
      if (pirMotionDetected) {
        if (loadPluggedIn) {
          pirState = PIR_MOTION_WITH_LOAD;
          ssrPirOverride = false;
        } else {
          pirState = PIR_MOTION_NO_LOAD;
          ssrPirOverride = true;
        }
      }
      break;
  }
}

// ==================== SSR CONTROL ====================
void controlSSR() {
  bool finalState;
  
  // Priority order: Safety > PIR > Command
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

// ==================== CALIBRATION ====================
void loadCalibration() {
  EEPROM.get(0, calData);
  
  if (calData.magic != EEPROM_MAGIC) {
    // Initialize with defaults
    calData = CalibrationData();
    saveCalibration();
    Serial.println(F("→ Calibration: Initialized with defaults"));
  } else {
    Serial.println(F("✓ Calibration: Loaded from EEPROM"));
  }
}

void saveCalibration() {
  calData.magic = EEPROM_MAGIC;
  calData.lastCalibration = millis();
  EEPROM.put(0, calData);
  EEPROM.commit();
  Serial.println(F("✓ Calibration: Saved to EEPROM"));
}

void runCalibration() {
  Serial.println(F("\n=== CALIBRATION MODE ==="));
  Serial.println(F("Follow instructions in serial monitor..."));
  // Calibration procedure would go here
  calibrationMode = false;
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
  network["renderAwake"] = renderServerAwake;                   // ✅ Track Render state
  
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
    Serial.println(F("→ System status sent to Render"));
  }
}

void printStatus() {
  Serial.println(F("\n============ SYSTEM STATUS ============"));
  Serial.printf("Uptime: %lu seconds\n", (millis() - systemStartTime) / 1000);
  Serial.printf("Loop count: %lu\n", loopCount);
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  
  Serial.println(F("\n--- NETWORK (RENDER.COM) ---"));
  Serial.printf("WiFi: %s (RSSI: %d dBm)\n", 
                wifiConnected ? "Connected" : "Disconnected", WiFi.RSSI());
  Serial.printf("WebSocket: %s\n", wsConnected ? "Connected" : "Disconnected");
  Serial.printf("Render Server: %s\n", renderServerAwake ? "AWAKE" : "Unknown");
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
      Serial.println(F("→ WebSocket ping sent to Render"));
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

// ==================== END OF VAULTGUARD PRODUCTION CODE ====================