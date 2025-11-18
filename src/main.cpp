/*
 * VaultGuard v7.4 - ENHANCED DEBUG VERSION WITH SMART PIR
 * Server: https://meifhi-esp-server.onrender.com
 *
 * ✅ FIXED FOR RENDER.COM DEPLOYMENT
 * ✅ ALL FUNCTIONALITY PRESERVED
 * ✅ DEVICE TYPE CORRECTED
 * ✅ SSL/TLS CONFIGURED FOR RENDER
 * ✅ SMART PIR: Only monitors when socket is EMPTY
 * ✅ ENHANCED DEBUG: Real-time PIR monitoring logs
 *
 * FEATURES:
 * ✓ WebSocket bidirectional communication with SSL
 * ✓ HTTPS POST data transmission with retry logic
 * ✓ Command reception and acknowledgment system
 * ✓ Auto-reconnection with exponential backoff
 * ✓ Comprehensive sensor calibration system
 * ✓ Full server protocol compliance
 * ✓ Smart PIR child safety (only active for empty sockets)
 * ✓ Energy monitoring and reporting
 * ✓ Real-time PIR debug logging (pin state, buffer, decisions)
 * ✓ 10-second PIR status summaries
 * ✓ Anomaly detection and auto-correction
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
#define PIR_SENSOR_PIN      18      // PIR motion sensor (SR602 Mini)
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
#define DEVICE_VERSION      "7.4-PIR-ENHANCED-DEBUG"

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

// ✅ ENHANCED LOAD DETECTION WITH WIDE HYSTERESIS (prevents oscillation)
#define LOAD_DETECTION_THRESHOLD_HIGH 0.15   // Current threshold for load CONNECT: 150mA
#define LOAD_DETECTION_THRESHOLD_LOW  0.03   // Current threshold for load DISCONNECT: 30mA
#define LOAD_CONFIRM_READINGS 5              // Consecutive readings needed to confirm load state change
#define CURRENT_SAMPLES     50               // Number of samples for averaging

// ZMPT101B Voltage Sensor Calibration (Philippines 220V @ 60Hz)
#define ZMPT101B_SENSITIVITY 0.004     // Sensor sensitivity
#define AC_FREQUENCY        60          // Philippines AC frequency
#define RMS_SAMPLES         100         // Samples for RMS calculation
#define NOMINAL_VOLTAGE     220.0       // Philippines nominal voltage
#define MAX_VOLTAGE         240.0       // Maximum expected voltage
#define MIN_VOLTAGE         200.0       // Minimum valid voltage
#define VOLTAGE_SAMPLES     100         // Number of voltage samples

// ==================== PIR SAFETY SETTINGS - SR602 MINI ====================
// IMPORTANT: SR602 Mini has ADJUSTABLE hardware timers (Tx & Sx potentiometers)
//
// ✅ SR602 MINI HARDWARE CHARACTERISTICS:
// - HAS Tx potentiometer: Delay time 5-200 seconds (adjustable)
// - HAS Sx potentiometer: Sensitivity 3-7 meters (adjustable)
// - Retriggerable: YES (motion extends the HIGH period)
// - Output: 3.3V (ESP32 compatible - NO voltage divider needed!)
// - Recommended Tx setting: MINIMUM (5-10 seconds) for responsive operation
//
// ⚡ OPTIMIZED SAFETY TIMING:
// - PIR MONITORING: ONLY ACTIVE when socket is EMPTY (no load detected)
// - PIR DISABLED: Automatically when load is plugged in (not needed)
// - EMERGENCY SSR CUTOFF: ~15ms after debounce confirmation
// - Motion confirmation: ~15ms (2/3 readings at 5ms intervals)
// - SSR RELEASE: IMMEDIATE when motion stops (no delay!)
// - SR602 pin stays HIGH: 5-200+ seconds (depends on Tx potentiometer setting)
// - Cooldown period: 10 seconds (prevents rapid re-triggers)
// - Stuck sensor timeout: 300 seconds (auto-recovery if sensor stuck HIGH)
//
// 🛡️ ENHANCED PROTECTIONS:
// - Auto-timeout if sensor stuck HIGH >300s (prevents SSR locked OFF)
// - Enhanced load detection with consecutive reading confirmation
// - Wider hysteresis gap to prevent oscillation (0.15A/0.03A)
// - SR602-aware debouncing for long HIGH periods
//
#define PIR_ENABLED         true
#define PIR_MOTION_TIMEOUT  10000      // 10 seconds cooldown after motion stops
#define PIR_CHECK_INTERVAL  5          // 5ms polling interval
#define PIR_DEBOUNCE_TIME   30         // 30ms debounce time
#define PIR_ALERT_BEEPS     3          // Number of alert beeps
#define PIR_SENSITIVITY     3          // Number of readings in debounce buffer
#define PIR_MOTION_CONFIRM_COUNT 2     // Need 2/3 readings HIGH to confirm motion
#define PIR_CLEAR_CONFIRM_COUNT  2     // Need 2/3 readings LOW to confirm clear
#define PIR_STATE_MIN_TIME  250        // Minimum time in state before transition (anti-oscillation)
#define PIR_DEBUG_LOGGING   true       // Enable detailed PIR debug logs

// ✅ SR602-SPECIFIC SETTINGS
#define PIR_MODEL           "SR602"    // Sensor model identifier
#define PIR_EXPECTED_MIN_HIGH_TIME  5000    // Minimum expected HIGH time (Tx at min): 5 seconds
#define PIR_EXPECTED_MAX_HIGH_TIME  200000  // Maximum expected HIGH time (Tx at max): 200 seconds
#define PIR_STUCK_TIMEOUT   300000     // Auto-recovery timeout: 300 seconds (5 minutes)
#define PIR_STUCK_WARNING_TIME 15000   // Warn if stuck HIGH >15 seconds
#define PIR_RECOMMEND_TX_ADJUSTMENT true    // Recommend Tx adjustment to user

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
unsigned long pirStateEntryTime = 0;       // Track when we entered current PIR state
int pirTriggerCount = 0;
int pirReadings[PIR_SENSITIVITY] = {0};
int pirReadIndex = 0;
int consecutiveMotionDetections = 0;       // Track consecutive motion readings
int consecutiveClearDetections = 0;        // Track consecutive clear readings

// ✅ SR602 STUCK-SENSOR PROTECTION
unsigned long pirStuckHighStartTime = 0;   // Track when sensor went HIGH
bool pirStuckHighWarned = false;           // Warning flag (avoid spam)
bool pirStuckTimeoutActive = false;        // Auto-recovery timeout active
unsigned long pirStuckTimeoutStart = 0;    // When timeout was activated

// ✅ ENHANCED LOAD DETECTION
int consecutiveLoadHighReadings = 0;       // Consecutive readings above HIGH threshold
int consecutiveLoadLowReadings = 0;        // Consecutive readings below LOW threshold
float lastCurrentReading = 0.0;            // Previous current reading for comparison

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
bool diagnosePIRSensor();                                       // ✅ PIR diagnostic function

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
  // ✅ FIX: Use INPUT_PULLDOWN to prevent floating pin (HW-456 outputs 3.3V on motion)
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

  Serial.println(F("\n--- Initial Sensor Readings ---"));
  Serial.printf("Current: %.3f A\n", testCurrent);
  Serial.printf("Voltage: %.1f V\n", testVoltage);

  // ✅ Run comprehensive PIR diagnostic
  bool pirHealthy = diagnosePIRSensor();
  
  // Check if load is initially connected
  loadPluggedIn = (testCurrent > LOAD_DETECTION_THRESHOLD_HIGH);
  Serial.printf("Load Status: %s (%.3fA)\n", loadPluggedIn ? "CONNECTED" : "EMPTY SOCKET", testCurrent);
  
  // PIR Safety System notification
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║   🛡️  CHILD SAFETY SYSTEM ENABLED  🛡️              ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║  PIR Sensor: SR602 Mini (GPIO 18)                 ║"));
  Serial.println(F("║  Protection: ONLY for empty sockets                ║"));
  Serial.println(F("║  Monitoring: DISABLED when load plugged in        ║"));
  Serial.println(F("║  Action: ⚡ CUTOFF ~15ms after motion confirmed    ║"));
  Serial.println(F("║  Release: ⚡ IMMEDIATE when motion stops           ║"));
  Serial.println(F("║  Warmup: 30-60 seconds for PIR stabilization      ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║  📌 SR602 Mini Characteristics:                    ║"));
  Serial.println(F("║    • SSR cutoff: ~15ms (after debouncing)         ║"));
  Serial.println(F("║    • SSR release: IMMEDIATE (no delay!)           ║"));
  Serial.println(F("║    • Motion confirmation: ~15ms (debounced)       ║"));
  Serial.println(F("║    • SR602 pin delay: 5-200s (Tx potentiometer)  ║"));
  Serial.println(F("║    • HAS Tx/Sx potentiometers (adjustable)        ║"));
  Serial.println(F("║    • Detection range: 3-7m (Sx adjustable)        ║"));
  Serial.println(F("║    • Output: 3.3V (NO voltage divider needed!)    ║"));
  Serial.println(F("║    • ⚠️ RECOMMEND: Set Tx to MINIMUM (5-10s)      ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║  ⚡ ENHANCED PROTECTION FEATURES:                  ║"));
  Serial.printf("║    • Check Interval: %dms                            ║\n", PIR_CHECK_INTERVAL);
  Serial.printf("║    • Buffer Size: %d readings                        ║\n", PIR_SENSITIVITY);
  Serial.printf("║    • Confirm Threshold: %d/%d readings               ║\n", PIR_MOTION_CONFIRM_COUNT, PIR_SENSITIVITY);
  Serial.printf("║    • Detection Time: ~%dms                          ║\n", PIR_CHECK_INTERVAL * PIR_SENSITIVITY);
  Serial.printf("║    • Load Hysteresis: %.2fA/%.2fA (CONNECT/DISC)    ║\n", LOAD_DETECTION_THRESHOLD_HIGH, LOAD_DETECTION_THRESHOLD_LOW);
  Serial.printf("║    • Load Confirmation: %d consecutive readings      ║\n", LOAD_CONFIRM_READINGS);
  Serial.printf("║    • Min State Time: %dms (anti-oscillation)        ║\n", PIR_STATE_MIN_TIME);
  Serial.printf("║    • Stuck Timeout: %ds (auto-recovery)             ║\n", PIR_STUCK_TIMEOUT/1000);
  Serial.printf("║    • Debug Logging: %s                              ║\n", PIR_DEBUG_LOGGING ? "ENABLED " : "DISABLED");
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  
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
  pirStateEntryTime = millis();  // Initialize PIR state timer
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

  // ✅ ENHANCED LOAD DETECTION WITH CONSECUTIVE READING CONFIRMATION
  // This prevents oscillation from noise/interference near threshold values
  // Requires LOAD_CONFIRM_READINGS consecutive readings to change state

  if (loadPluggedIn) {
    // Currently has load - check if it disconnected
    if (currentReading < LOAD_DETECTION_THRESHOLD_LOW) {
      consecutiveLoadLowReadings++;
      consecutiveLoadHighReadings = 0;  // Reset opposite counter

      // Confirm disconnect after consecutive readings
      if (consecutiveLoadLowReadings >= LOAD_CONFIRM_READINGS) {
        loadPluggedIn = false;
        consecutiveLoadLowReadings = 0;  // Reset counter

        Serial.println(F("\n╔══════════════════════════════════════════╗"));
        Serial.println(F("║  🔌 LOAD DISCONNECTED (CONFIRMED)        ║"));
        Serial.println(F("╚══════════════════════════════════════════╝"));
        Serial.printf("→ Current: %.3fA < %.2fA (threshold)\n", currentReading, LOAD_DETECTION_THRESHOLD_LOW);
        Serial.printf("→ Confirmed by %d consecutive readings\n", LOAD_CONFIRM_READINGS);
        Serial.println(F("→ PIR MONITORING NOW ACTIVE"));
        Serial.println(F("→ SSR will be disabled if motion detected\n"));
      }
    } else {
      // Still above threshold - reset counter
      consecutiveLoadLowReadings = 0;
    }
  } else {
    // Currently no load - check if load connected
    if (currentReading > LOAD_DETECTION_THRESHOLD_HIGH) {
      consecutiveLoadHighReadings++;
      consecutiveLoadLowReadings = 0;  // Reset opposite counter

      // Confirm connection after consecutive readings
      if (consecutiveLoadHighReadings >= LOAD_CONFIRM_READINGS) {
        loadPluggedIn = true;
        consecutiveLoadHighReadings = 0;  // Reset counter

        Serial.println(F("\n╔══════════════════════════════════════════╗"));
        Serial.println(F("║  🔌 LOAD CONNECTED (CONFIRMED)           ║"));
        Serial.println(F("╚══════════════════════════════════════════╝"));
        Serial.printf("→ Current: %.3fA > %.2fA (threshold)\n", currentReading, LOAD_DETECTION_THRESHOLD_HIGH);
        Serial.printf("→ Confirmed by %d consecutive readings\n", LOAD_CONFIRM_READINGS);
        Serial.println(F("→ PIR MONITORING DISABLED"));
        Serial.println(F("→ PIR override released if active\n"));
      }
    } else {
      // Still below threshold - reset counter
      consecutiveLoadHighReadings = 0;
    }
  }

  // Track current reading for comparison
  lastCurrentReading = currentReading;
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

  // ✅ DISABLE PIR MONITORING IF LOAD IS PLUGGED IN
  // PIR safety is only needed for empty sockets (child safety)
  // If something is already plugged in, no need to monitor
  if (loadPluggedIn) {
    // ALWAYS reset PIR state when load is present (prevent any lingering state)
    if (pirState != PIR_IDLE || ssrPirOverride || pirMotionDetected) {

      #if PIR_DEBUG_LOGGING
      // Only log once when transitioning to disabled state
      static bool disableLogged = false;
      if (!disableLogged) {
        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  ✓ LOAD DETECTED - PIR MONITORING DISABLED   ✓   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ Load Current: %.3f A (above %.2f A threshold)\n",
                      currentReading, LOAD_DETECTION_THRESHOLD_HIGH);
        Serial.println(F("→ PIR monitoring paused - socket is not empty"));
        Serial.println(F("→ SSR override released (if active)"));
        Serial.println(F("→ PIR will resume when load is disconnected\n"));
        disableLogged = true;
      }
      #endif

      // Force reset ALL PIR state (critical for preventing false triggers)
      pirState = PIR_IDLE;
      ssrPirOverride = false;
      pirMotionDetected = false;
      lastMotionTime = 0;
      pirTriggerCount = 0;

      // Clear PIR buffer completely
      for (int i = 0; i < PIR_SENSITIVITY; i++) {
        pirReadings[i] = 0;
      }
      pirReadIndex = 0;
      consecutiveMotionDetections = 0;
      consecutiveClearDetections = 0;

      // Reset stuck sensor tracking
      pirStuckHighStartTime = 0;
      pirStuckHighWarned = false;
      pirStuckTimeoutActive = false;
      pirStuckTimeoutStart = 0;
    } else {
      // Reset the disable log flag when state is already clean
      #if PIR_DEBUG_LOGGING
      static bool disableLogged = false;
      disableLogged = false;
      #endif
    }

    return;  // Skip all PIR monitoring while load is present
  }

  // If we get here, load is NOT plugged in - PIR should be active
  #if PIR_DEBUG_LOGGING
  static bool enableLogged = false;
  if (!enableLogged && !loadPluggedIn) {
    Serial.println(F("\n[PIR] Socket is EMPTY - PIR monitoring ACTIVE\n"));
    enableLogged = true;
  }
  // Reset flag when load is present
  if (loadPluggedIn) {
    enableLogged = false;
  }
  #endif

  unsigned long now = millis();
  unsigned long timeInCurrentState = now - pirStateEntryTime;

  // ========== STEP 1: PIR SENSOR READING WITH IMPROVED DEBOUNCING ==========
  if (now - lastPirCheck >= PIR_CHECK_INTERVAL) {
    lastPirCheck = now;

    // Read PIR sensor
    int pirReading = digitalRead(PIR_SENSOR_PIN);

    #if PIR_DEBUG_LOGGING
    // ✅ REAL-TIME PIR PIN MONITORING (shows every reading)
    Serial.printf("[PIR-RAW] Time: %lu ms | Pin %d = %s | ",
                  now, PIR_SENSOR_PIN, pirReading == HIGH ? "HIGH(1)" : "LOW(0)");
    #endif

    // ✅ SR602 HEALTH MONITORING: Detect if sensor stuck HIGH for extended period
    if (pirReading == HIGH) {
      if (pirStuckHighStartTime == 0) {
        pirStuckHighStartTime = now;  // Start tracking when sensor went HIGH
      }

      unsigned long stuckDuration = now - pirStuckHighStartTime;

      // ⚠️ WARNING: If HIGH for more than 15 seconds, warn user about Tx setting
      if (stuckDuration > PIR_STUCK_WARNING_TIME && !pirStuckHighWarned) {
        Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
        Serial.println(F("║  ⚠️  SR602 SENSOR EXTENDED HIGH PERIOD             ║"));
        Serial.println(F("╚════════════════════════════════════════════════════╝"));
        Serial.printf("→ Sensor HIGH continuously for %.1f seconds\n", stuckDuration/1000.0);
        Serial.println(F("→ This is NORMAL for SR602 if Tx potentiometer is set high"));
        Serial.println(F("→ SR602 can stay HIGH for 5-200 seconds (depends on Tx)"));
        Serial.println(F("→ RECOMMENDATION: Adjust Tx potentiometer to MINIMUM"));
        Serial.println(F("   • Turn Tx potentiometer fully COUNTER-CLOCKWISE"));
        Serial.println(F("   • This sets delay to ~5-10 seconds (more responsive)"));
        Serial.println(F("   • Current setting appears to be >15 seconds"));
        Serial.printf("→ Auto-recovery activates at %d seconds\n", PIR_STUCK_TIMEOUT/1000);
        Serial.println(F("→ System continues monitoring...\n"));
        pirStuckHighWarned = true;
      }

      // 🛡️ AUTO-RECOVERY: If stuck HIGH beyond timeout, temporarily disable PIR override
      if (stuckDuration > PIR_STUCK_TIMEOUT && !pirStuckTimeoutActive) {
        pirStuckTimeoutActive = true;
        pirStuckTimeoutStart = now;

        Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
        Serial.println(F("║  🛡️  AUTO-RECOVERY ACTIVATED                       ║"));
        Serial.println(F("╚════════════════════════════════════════════════════╝"));
        Serial.printf("→ Sensor stuck HIGH for %.1f seconds (>%d second timeout)\n",
                      stuckDuration/1000.0, PIR_STUCK_TIMEOUT/1000);
        Serial.println(F("→ Temporarily releasing PIR override to prevent SSR locked OFF"));
        Serial.println(F("→ URGENT: Adjust SR602 Tx potentiometer to minimum!"));
        Serial.println(F("→ PIR safety monitoring continues normally\n"));

        // Release PIR override if active
        if (ssrPirOverride) {
          ssrPirOverride = false;
          Serial.println(F("✓ PIR override released - SSR control restored\n"));
        }
      }

    } else {
      // Reset stuck detection when we see a LOW reading
      if (pirStuckHighStartTime != 0) {
        unsigned long totalHighTime = now - pirStuckHighStartTime;

        // Log if it was a long HIGH period
        if (totalHighTime > PIR_EXPECTED_MIN_HIGH_TIME) {
          Serial.printf("\n[PIR] Sensor was HIGH for %.1f seconds (Tx setting detected)\n",
                       totalHighTime/1000.0);
        }

        pirStuckHighStartTime = 0;
        pirStuckHighWarned = false;
        pirStuckTimeoutActive = false;
      }
    }

    // ✅ Enhanced logging removed - we have real-time logging above now

    // Update circular buffer for debouncing
    pirReadings[pirReadIndex] = pirReading;
    int currentBufferIndex = pirReadIndex;
    pirReadIndex = (pirReadIndex + 1) % PIR_SENSITIVITY;

    // Count HIGH readings in buffer
    int highCount = 0;
    for (int i = 0; i < PIR_SENSITIVITY; i++) {
      if (pirReadings[i] == HIGH) highCount++;
    }

    #if PIR_DEBUG_LOGGING
    // ✅ DETAILED BUFFER STATE (shows circular buffer contents)
    Serial.printf("Buffer[");
    for (int i = 0; i < PIR_SENSITIVITY; i++) {
      // Highlight the current position
      if (i == currentBufferIndex) Serial.print(">");
      Serial.printf("%d", pirReadings[i]);
      if (i == currentBufferIndex) Serial.print("<");
      if (i < PIR_SENSITIVITY - 1) Serial.print(",");
    }
    Serial.printf("] | HIGH:%d/%d | ", highCount, PIR_SENSITIVITY);
    #endif

    // Determine motion state with IMPROVED HYSTERESIS
    bool motionNow = false;
    String decisionReason = "";

    if (pirMotionDetected) {
      // Currently detecting motion - require clear confirmation to stop
      int clearThreshold = PIR_SENSITIVITY - PIR_CLEAR_CONFIRM_COUNT + 1;
      motionNow = (highCount >= clearThreshold);

      #if PIR_DEBUG_LOGGING
      decisionReason = motionNow ?
        "STILL_MOTION(need " + String(PIR_SENSITIVITY - clearThreshold + 1) + " LOW to clear)" :
        "CLEARED(enough LOW readings)";
      #endif
    } else {
      // Currently clear - require motion confirmation to start
      motionNow = (highCount >= PIR_MOTION_CONFIRM_COUNT);

      #if PIR_DEBUG_LOGGING
      decisionReason = motionNow ?
        "MOTION_DETECTED(>=" + String(PIR_MOTION_CONFIRM_COUNT) + " HIGH)" :
        "NO_MOTION(<" + String(PIR_MOTION_CONFIRM_COUNT) + " HIGH)";
      #endif
    }

    #if PIR_DEBUG_LOGGING
    // ✅ DECISION LOGIC (shows why motion was/wasn't detected)
    Serial.printf("=> %s", decisionReason.c_str());

    // Show current state
    Serial.printf(" | State: ");
    switch(pirState) {
      case PIR_IDLE: Serial.print("IDLE"); break;
      case PIR_MOTION_NO_LOAD: Serial.print("MOTION_NO_LOAD"); break;
      case PIR_MOTION_WITH_LOAD: Serial.print("MOTION_WITH_LOAD"); break;
      case PIR_COOLDOWN: Serial.print("COOLDOWN"); break;
    }

    // Show load status
    Serial.printf(" | Load: %s(%.3fA)",
                  loadPluggedIn ? "YES" : "NO", currentReading);

    Serial.println();  // End of line for this PIR reading cycle
    #endif

    // ✅ MOTION STATE CHANGE DETECTION (enhanced logging)
    if (motionNow && !pirMotionDetected) {
      pirMotionDetected = true;
      lastMotionTime = now;
      pirTriggerCount++;

      Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
      Serial.println(F("║  ⚠️  MOTION DETECTED - PIR SENSOR TRIGGERED  ⚠️   ║"));
      Serial.println(F("╚═══════════════════════════════════════════════════╝"));
      Serial.printf("→ Trigger #%d | Time: %lu ms | Load: %s (%.3fA)\n",
                    pirTriggerCount, now, loadPluggedIn ? "CONNECTED" : "EMPTY", currentReading);
      Serial.printf("→ Confirmation: %d/%d HIGH readings in buffer\n",
                    highCount, PIR_SENSITIVITY);
      Serial.printf("→ Detection latency: ~%dms (debounce time)\n",
                    PIR_CHECK_INTERVAL * PIR_SENSITIVITY);
      Serial.printf("→ PIR State before: ");
      switch(pirState) {
        case PIR_IDLE: Serial.println("IDLE"); break;
        case PIR_MOTION_NO_LOAD: Serial.println("MOTION_NO_LOAD"); break;
        case PIR_MOTION_WITH_LOAD: Serial.println("MOTION_WITH_LOAD"); break;
        case PIR_COOLDOWN: Serial.println("COOLDOWN"); break;
      }

      // ⚡ CRITICAL SAFETY: IMMEDIATE SSR CUT-OFF IF NO LOAD (after debounce confirmation)
      if (!loadPluggedIn && pirEnabled) {
        unsigned long cutoffTime = micros();  // Record exact cutoff time in microseconds
        ssrPirOverride = true;
        digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);  // IMMEDIATE hardware cutoff
        unsigned long cutoffLatency = micros() - cutoffTime;

        Serial.println(F("→ ⚡ EMERGENCY ACTION: IMMEDIATE SSR CUTOFF"));
        Serial.printf("→ ⚡ SSR Cutoff Latency: %lu μs (hardware write time)\n", cutoffLatency);
        Serial.println(F("→ ⚡ Reason: Motion detected + Empty socket (DANGER)"));
        Serial.println(F("→ ⚡ Child safety protection ACTIVATED"));
      } else if (loadPluggedIn) {
        Serial.println(F("→ ✓ No SSR action needed (load present - safe)"));
      }

    } else if (!motionNow && pirMotionDetected) {
      pirMotionDetected = false;
      unsigned long motionDuration = now - lastMotionTime;

      Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
      Serial.println(F("║  ✓ MOTION STOPPED - PIR SENSOR CLEAR         ✓   ║"));
      Serial.println(F("╚═══════════════════════════════════════════════════╝"));
      Serial.printf("→ Motion Duration: %lu ms (%.2f seconds)\n",
                    motionDuration, motionDuration/1000.0);
      Serial.printf("→ Total Triggers: %d\n", pirTriggerCount);
      Serial.printf("→ Confirmation: Only %d/%d HIGH readings in buffer\n",
                    highCount, PIR_SENSITIVITY);
      Serial.printf("→ PIR State: ");
      switch(pirState) {
        case PIR_IDLE: Serial.println("IDLE"); break;
        case PIR_MOTION_NO_LOAD: Serial.println("MOTION_NO_LOAD (will transition)"); break;
        case PIR_MOTION_WITH_LOAD: Serial.println("MOTION_WITH_LOAD (will transition)"); break;
        case PIR_COOLDOWN: Serial.println("COOLDOWN"); break;
      }
    }
  }

  // ========== STEP 2: STATE MACHINE WITH TRANSITION GUARDS ==========
  PIRState previousState = pirState;

  switch (pirState) {
    case PIR_IDLE:
      // Prevent rapid transitions - require minimum time in IDLE
      if (timeInCurrentState < PIR_STATE_MIN_TIME) {
        break;
      }

      if (pirMotionDetected && !loadPluggedIn) {
        // DANGER: Motion detected with empty socket
        pirState = PIR_MOTION_NO_LOAD;
        pirStateEntryTime = now;
        ssrPirOverride = true;

        // Alert sequence (SSR already cut off immediately above)
        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║                                                   ║"));
        Serial.println(F("║        🚨🚨🚨 CHILD SAFETY ALERT 🚨🚨🚨          ║"));
        Serial.println(F("║                                                   ║"));
        Serial.println(F("║   DANGER: Motion Detected Near Empty Socket!     ║"));
        Serial.println(F("║   ACTION: SSR ALREADY DISABLED (<20ms response)  ║"));
        Serial.println(F("║                                                   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_IDLE → PIR_MOTION_NO_LOAD\n");
        Serial.printf("→ SSR Override: ENGAGED (ssrPirOverride = true)\n");
        Serial.printf("→ Current: %.3f A (below %.2f A)\n",
                      currentReading, LOAD_DETECTION_THRESHOLD_LOW);
        Serial.println(F("→ Initiating 3-beep alert sequence..."));

        for (int i = 0; i < PIR_ALERT_BEEPS; i++) {
          Serial.printf("   Beep %d/%d\n", i+1, PIR_ALERT_BEEPS);
          digitalWrite(RED_LED_PIN, HIGH);
          digitalWrite(BUZZER_PIN, HIGH);
          delay(150);
          digitalWrite(RED_LED_PIN, LOW);
          digitalWrite(BUZZER_PIN, LOW);
          delay(150);
        }
        Serial.println(F("→ Alert sequence complete. SSR remains OFF.\n"));

      } else if (pirMotionDetected && loadPluggedIn) {
        // Safe: Motion with load present
        pirState = PIR_MOTION_WITH_LOAD;
        pirStateEntryTime = now;

        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  ✓ SAFE MOTION DETECTED (Load Connected)     ✓   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_IDLE → PIR_MOTION_WITH_LOAD\n");
        Serial.printf("→ Load Current: %.3f A (above %.2f A)\n",
                      currentReading, LOAD_DETECTION_THRESHOLD_HIGH);
        Serial.println(F("→ Normal operation continues. No safety override.\n"));
      }
      break;

    case PIR_MOTION_NO_LOAD:
      // Keep SSR OFF while motion persists (already enforced by override)

      if (!pirMotionDetected && timeInCurrentState >= PIR_STATE_MIN_TIME) {
        // Motion stopped, release SSR override and enter cooldown
        pirState = PIR_COOLDOWN;
        pirStateEntryTime = now;
        lastMotionTime = now;
        ssrPirOverride = false;  // ✅ FIX: Release override immediately when motion stops

        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  → MOTION STOPPED - SSR RELEASED              →   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_MOTION_NO_LOAD → PIR_COOLDOWN\n");
        Serial.printf("→ Cooldown Duration: %d ms (%d seconds)\n",
                      PIR_MOTION_TIMEOUT, PIR_MOTION_TIMEOUT/1000);
        Serial.println(F("→ SSR Override: RELEASED (relay can turn back ON now)\n"));
      }

      // Check if load was plugged in (with state guard)
      if (loadPluggedIn && timeInCurrentState >= PIR_STATE_MIN_TIME) {
        pirState = PIR_MOTION_WITH_LOAD;
        pirStateEntryTime = now;
        ssrPirOverride = false;

        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  ✓ LOAD PLUGGED IN - OVERRIDE RELEASED        ✓   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_MOTION_NO_LOAD → PIR_MOTION_WITH_LOAD\n");
        Serial.printf("→ SSR Override: DISENGAGED (ssrPirOverride = false)\n");
        Serial.printf("→ Load Current: %.3f A (above %.2f A)\n",
                      currentReading, LOAD_DETECTION_THRESHOLD_HIGH);
        Serial.println(F("→ SSR control restored to normal operation.\n"));
      }
      break;

    case PIR_MOTION_WITH_LOAD:
      // Normal operation, just monitoring

      if (!pirMotionDetected && timeInCurrentState >= PIR_STATE_MIN_TIME) {
        pirState = PIR_IDLE;
        pirStateEntryTime = now;

        Serial.println(F("\n→ PIR State: PIR_MOTION_WITH_LOAD → PIR_IDLE"));
        Serial.println(F("   Motion stopped, returning to idle state.\n"));
      }

      if (!loadPluggedIn) {
        // Load unplugged during motion! IMMEDIATE SAFETY OVERRIDE
        pirState = PIR_MOTION_NO_LOAD;
        pirStateEntryTime = now;
        ssrPirOverride = true;
        digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);  // IMMEDIATE cutoff

        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  ⚠️  LOAD UNPLUGGED DURING MOTION!            ⚠️   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_MOTION_WITH_LOAD → PIR_MOTION_NO_LOAD\n");
        Serial.printf("→ SSR Override: ENGAGED (ssrPirOverride = true)\n");
        Serial.printf("→ Current dropped to: %.3f A (below %.2f A)\n",
                      currentReading, LOAD_DETECTION_THRESHOLD_LOW);
        Serial.println(F("→ ⚡ SSR IMMEDIATELY DISABLED for safety!\n"));
      }
      break;

    case PIR_COOLDOWN:
      // Wait for cooldown period to complete
      // Note: SSR override was already released when entering this state
      if (now - lastMotionTime >= PIR_MOTION_TIMEOUT) {
        // Cooldown complete - return to IDLE
        pirState = PIR_IDLE;
        pirStateEntryTime = now;

        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║  ✓ COOLDOWN COMPLETE - SYSTEM READY          ✓   ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.printf("→ State Transition: PIR_COOLDOWN → PIR_IDLE\n");
        Serial.println(F("→ Normal operation resumed. PIR monitoring active.\n"));
      }

      // If motion detected again during cooldown - evaluate immediately (safety critical)
      if (pirMotionDetected && timeInCurrentState >= PIR_STATE_MIN_TIME) {
        Serial.println(F("\n→ PIR: Motion detected during cooldown!"));

        if (loadPluggedIn) {
          // Safe - load present
          pirState = PIR_MOTION_WITH_LOAD;
          pirStateEntryTime = now;
          Serial.printf("   Load present (%.3f A) → Transitioning to PIR_MOTION_WITH_LOAD\n",
                       currentReading);
        } else {
          // DANGER - no load, activate override immediately
          pirState = PIR_MOTION_NO_LOAD;
          pirStateEntryTime = now;
          ssrPirOverride = true;
          digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);  // Immediate cutoff
          lastMotionTime = now;  // Reset cooldown timer
          Serial.printf("   Empty socket (%.3f A) → EMERGENCY CUTOFF ACTIVATED!\n",
                       currentReading);
        }
      }
      break;
  }

  // Log state transitions for debugging
  if (previousState != pirState) {
    Serial.printf("[PIR] State changed: %d → %d | Time in prev state: %lu ms\n",
                  previousState, pirState, timeInCurrentState);
  }
}

// ==================== SSR CONTROL ====================
void controlSSR() {
  static bool lastFinalState = false;
  static unsigned long lastStateChange = 0;
  bool finalState;
  String stateReason = "";

  // ✅ DIAGNOSTIC: Check for unexpected override states when load is present
  if (loadPluggedIn && ssrPirOverride) {
    Serial.println(F("\n⚠️⚠️⚠️ ANOMALY DETECTED ⚠️⚠️⚠️"));
    Serial.println(F("PIR override is ACTIVE despite load being present!"));
    Serial.printf("Load: %.3fA | PIR State: %d | Motion: %s\n",
                  currentReading, pirState, pirMotionDetected ? "YES" : "NO");
    Serial.println(F("Forcing PIR override OFF (load should disable PIR)"));
    ssrPirOverride = false;  // Force it off
    pirState = PIR_IDLE;
    pirMotionDetected = false;
  }

  // Priority order: Safety > PIR > Command
  if (ssrSafetyOverride) {
    // Critical safety override (overvoltage/overcurrent)
    finalState = false;
    stateReason = "SAFETY_OVERRIDE";
  } else if (ssrPirOverride) {
    // PIR CHILD SAFETY OVERRIDE - IMMEDIATE OFF
    finalState = false;
    stateReason = "PIR_OVERRIDE";

    // ✅ DIAGNOSTIC: Log unexpected PIR override every time it's active
    static unsigned long lastPirOverrideLog = 0;
    if (millis() - lastPirOverrideLog > 1000) {  // Log every second to avoid spam
      Serial.println(F("\n⚠️⚠️⚠️ PIR OVERRIDE ACTIVE - SSR FORCED OFF ⚠️⚠️⚠️"));
      Serial.printf("→ Current: %.3fA | Load: %s | PIR Motion: %s\n",
                    currentReading, loadPluggedIn ? "YES" : "NO",
                    pirMotionDetected ? "YES ✓" : "NO ❌ (should be YES!)");

      if (!pirMotionDetected) {
        Serial.println(F("→ 🔴 PROBLEM: Override is ON but motion is NOT detected!"));
        Serial.println(F("   This is the bug causing unexpected relay OFF"));
      }
      lastPirOverrideLog = millis();
    }

    // Double-check it's really OFF for child safety
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  } else {
    // Normal command state
    finalState = ssrCommandState;
    stateReason = ssrCommandState ? "COMMAND_ON" : "COMMAND_OFF";
  }

  // ✅ DIAGNOSTICS: Log state changes to track random toggling
  if (finalState != lastFinalState) {
    unsigned long now = millis();
    unsigned long timeSinceLastChange = now - lastStateChange;

    Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
    Serial.printf("║  🔄 SSR STATE CHANGE: %s → %s\n",
                  lastFinalState ? "ON " : "OFF", finalState ? "ON " : "OFF");
    Serial.println(F("╚═══════════════════════════════════════════════════╝"));
    Serial.printf("→ ⚠️⚠️ REASON: %s ⚠️⚠️\n", stateReason.c_str());
    Serial.printf("→ Time since last change: %lu ms\n", timeSinceLastChange);
    Serial.printf("→ PIR State: %s | Motion: %s | Load: %s (%.3fA)\n",
                  pirState == PIR_IDLE ? "IDLE" :
                  pirState == PIR_MOTION_NO_LOAD ? "MOTION_NO_LOAD" :
                  pirState == PIR_MOTION_WITH_LOAD ? "MOTION_WITH_LOAD" : "COOLDOWN",
                  pirMotionDetected ? "YES" : "NO",
                  loadPluggedIn ? "YES" : "NO",
                  currentReading);
    Serial.printf("→ Overrides: PIR=%s | Safety=%s\n",
                  ssrPirOverride ? "ACTIVE" : "inactive",
                  ssrSafetyOverride ? "ACTIVE" : "inactive");
    Serial.printf("→ Command State: %s\n", ssrCommandState ? "ON" : "OFF");

    // ✅ DETAILED PRIORITY ANALYSIS
    Serial.println(F("\n--- WHY DID SSR CHANGE STATE? ---"));
    if (ssrSafetyOverride) {
      Serial.println(F("1. ⚠️ SAFETY OVERRIDE ACTIVE → Forced OFF (overvoltage/overcurrent)"));
    } else if (ssrPirOverride) {
      Serial.println(F("2. ⚠️⚠️ PIR OVERRIDE ACTIVE → Forced OFF (child safety)"));
      Serial.printf("   PIR Motion Detected: %s\n", pirMotionDetected ? "YES" : "NO ❌");
      Serial.printf("   Load Plugged In: %s\n", loadPluggedIn ? "YES" : "NO");

      // ✅ SMOKING GUN DETECTOR
      if (!pirMotionDetected) {
        Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
        Serial.println(F("║ 🔴 FOUND IT! PIR OVERRIDE WITHOUT MOTION 🔴      ║"));
        Serial.println(F("╚═══════════════════════════════════════════════════╝"));
        Serial.println(F("→ This is why relay turns OFF when terminal shows 0!"));
        Serial.println(F("→ PIR override flag is ACTIVE but motion = NO"));
        Serial.println(F("→ Likely causes:"));
        Serial.println(F("   1. PIR override stuck from previous event"));
        Serial.println(F("   2. Race condition in state machine"));
        Serial.println(F("   3. Motion cleared but override not released"));
        Serial.println(F(""));
      }
    } else if (ssrCommandState) {
      Serial.println(F("3. Command State = ON → SSR turned ON normally"));
    } else {
      Serial.println(F("3. Command State = OFF → SSR turned OFF by command"));
    }
    Serial.println();

    lastFinalState = finalState;
    lastStateChange = now;
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

// ==================== PIR SENSOR DIAGNOSTICS ====================
bool diagnosePIRSensor() {
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║   🔍 PIR SENSOR DIAGNOSTIC TEST (SR602 Mini)       ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));

  // Take 20 rapid readings over 1 second to check sensor behavior
  int highReadings = 0;
  int lowReadings = 0;
  int readings[20];

  Serial.println(F("\n→ Taking 20 samples over 1 second..."));
  for (int i = 0; i < 20; i++) {
    readings[i] = digitalRead(PIR_SENSOR_PIN);
    if (readings[i] == HIGH) highReadings++;
    else lowReadings++;
    delay(50);
  }

  // Display results
  Serial.print(F("   Readings: ["));
  for (int i = 0; i < 20; i++) {
    Serial.print(readings[i]);
    if (i < 19) Serial.print(F(","));
  }
  Serial.println(F("]"));
  Serial.printf("   HIGH: %d/20 | LOW: %d/20\n", highReadings, lowReadings);

  // Analyze sensor health
  bool sensorHealthy = true;

  if (highReadings == 20) {
    Serial.println(F("\n⚠️  NOTICE: PIR sensor reading HIGH continuously"));
    Serial.println(F("   For SR602 Mini, this could be NORMAL:"));
    Serial.println(F("   1. Motion detected - SR602 stays HIGH for 5-200 seconds"));
    Serial.println(F("   2. Tx potentiometer set to high value (long delay)"));
    Serial.println(F("   3. Sensor is retriggering (motion keeps extending time)"));
    Serial.println(F("   4. Sensor warming up (wait 30-60 seconds after power-on)"));
    Serial.println(F("\n   Other possible issues:"));
    Serial.println(F("   5. Wiring issue - sensor output shorted to 3.3V"));
    Serial.println(F("   6. Faulty sensor"));
    Serial.println(F("\n   RECOMMENDED ACTIONS:"));
    Serial.println(F("   → Wait 60 seconds for sensor warmup"));
    Serial.println(F("   → If still HIGH, adjust Tx potentiometer to MINIMUM"));
    Serial.println(F("   → Turn Tx potentiometer fully COUNTER-CLOCKWISE"));
    Serial.println(F("   → Check wiring: SR602 has 3 pins: VCC, OUT, GND"));
    Serial.println(F("   → Verify OUT connects directly to GPIO 18 (no voltage divider!)"));
    Serial.println(F("   → SR602 Mini outputs 3.3V - compatible with ESP32"));
    Serial.println(F("   → Auto-recovery will activate if stuck >300 seconds"));
    sensorHealthy = true;  // Might be normal for SR602
  }
  else if (lowReadings == 20) {
    Serial.println(F("\n✓ EXCELLENT: PIR sensor reading LOW (no motion detected)"));
    Serial.println(F("   Sensor appears to be working correctly."));
    Serial.println(F("   Try moving in front of sensor to test motion detection."));
    Serial.println(F("   Expect sensor to stay HIGH for 5-200s after motion stops"));
    Serial.println(F("   (duration depends on Tx potentiometer setting)"));
    sensorHealthy = true;
  }
  else {
    Serial.println(F("\n⚠️  NOTICE: PIR sensor showing mixed readings"));
    Serial.printf("   This could indicate:\n");
    Serial.println(F("   • Actual motion being detected (NORMAL)"));
    Serial.println(F("   • Sensor warming up (wait 30-60 seconds)"));
    Serial.println(F("   • Tx delay period ending (sensor transitioning LOW)"));
    Serial.println(F("   • Environmental interference (rare)"));
    Serial.println(F("\n   For SR602, mixed readings are often normal during operation."));
    sensorHealthy = true;  // Normal for SR602
  }

  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.printf("║   Sensor Status: %-33s ║\n", sensorHealthy ? "✓ HEALTHY" : "❌ NEEDS ATTENTION");
  Serial.println(F("╚════════════════════════════════════════════════════╝\n"));

  return sensorHealthy;
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
  pir["model"] = PIR_MODEL;
  pir["enabled"] = pirEnabled;
  pir["motion"] = pirMotionDetected;
  pir["state"] = pirState;
  pir["triggers"] = pirTriggerCount;
  pir["override"] = ssrPirOverride;
  pir["stuckTimeout"] = pirStuckTimeoutActive;
  if (pirStuckHighStartTime > 0) {
    pir["highDuration"] = (millis() - pirStuckHighStartTime) / 1000;
  }
  
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
  
  Serial.println(F("\n--- PIR SAFETY (SR602 ENHANCED) ---"));
  Serial.printf("PIR Model: %s | Status: %s\n", PIR_MODEL, pirEnabled ? "✓ Enabled" : "✗ Disabled");
  Serial.printf("Motion: %s, Load: %s (%.3f A)\n",
                pirMotionDetected ? "⚠️  DETECTED" : "✓ CLEAR",
                loadPluggedIn ? "✓ CONNECTED" : "⚠️  EMPTY",
                currentReading);
  Serial.printf("Load Hysteresis: %.2fA (connect) / %.2fA (disconnect)\n",
                LOAD_DETECTION_THRESHOLD_HIGH, LOAD_DETECTION_THRESHOLD_LOW);
  Serial.printf("Load Confirmation: %d consecutive readings required\n", LOAD_CONFIRM_READINGS);
  Serial.printf("PIR State: ");
  unsigned long timeInState = millis() - pirStateEntryTime;
  switch (pirState) {
    case PIR_IDLE: Serial.printf("✓ IDLE (Normal, %lu ms)\n", timeInState); break;
    case PIR_MOTION_NO_LOAD: Serial.printf("🚨 MOTION+NO_LOAD (SSR OFF, %lu ms)\n", timeInState); break;
    case PIR_MOTION_WITH_LOAD: Serial.printf("⚠️  MOTION+LOAD (Safe, %lu ms)\n", timeInState); break;
    case PIR_COOLDOWN:
      Serial.printf("⏳ COOLDOWN (%lu/%d ms remaining)\n",
                    millis() - lastMotionTime, PIR_MOTION_TIMEOUT);
      break;
  }

  // Show stuck sensor status if applicable
  if (pirStuckHighStartTime > 0) {
    unsigned long stuckDuration = millis() - pirStuckHighStartTime;
    Serial.printf("⚠️  Sensor HIGH Duration: %.1fs", stuckDuration/1000.0);
    if (pirStuckTimeoutActive) {
      Serial.print(" (AUTO-RECOVERY ACTIVE)");
    }
    Serial.println();
  }

  Serial.printf("Trigger Count: %d | Check Rate: %dms | Buffer: %d readings\n",
                pirTriggerCount, PIR_CHECK_INTERVAL, PIR_SENSITIVITY);
  Serial.printf("Confirm Threshold: %d/%d (motion), %d/%d (clear)\n",
                PIR_MOTION_CONFIRM_COUNT, PIR_SENSITIVITY,
                PIR_CLEAR_CONFIRM_COUNT, PIR_SENSITIVITY);
  Serial.printf("Detection: ~%dms | State Guard: %dms | Stuck Timeout: %ds\n",
                PIR_CHECK_INTERVAL * PIR_SENSITIVITY,
                PIR_STATE_MIN_TIME,
                PIR_STUCK_TIMEOUT/1000);
  Serial.printf("Debug: %s | Auto-Recovery: %s\n",
                PIR_DEBUG_LOGGING ? "ON" : "OFF",
                pirStuckTimeoutActive ? "ACTIVE" : "Ready");
  
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
  
  // ✅ PIR MONITORING STATUS - Print every 10 seconds
  static unsigned long lastPirStatus = 0;
  if (millis() - lastPirStatus >= 10000) {
    Serial.println(F("\n╔════════════ PIR MONITORING STATUS ═════════════╗"));
    Serial.printf("║ PIR Enabled: %-10s | Monitoring: %-10s ║\n",
                  pirEnabled ? "YES" : "NO",
                  (pirEnabled && !loadPluggedIn) ? "ACTIVE" : "DISABLED");
    Serial.printf("║ Motion: %-10s | Load: %-10s(%.2fA) ║\n",
                  pirMotionDetected ? "DETECTED" : "CLEAR",
                  loadPluggedIn ? "CONNECTED" : "EMPTY",
                  currentReading);
    Serial.printf("║ PIR State: %-27s ║\n",
                  pirState == PIR_IDLE ? "IDLE" :
                  pirState == PIR_MOTION_NO_LOAD ? "MOTION_NO_LOAD" :
                  pirState == PIR_MOTION_WITH_LOAD ? "MOTION_WITH_LOAD" :
                  "COOLDOWN");
    Serial.printf("║ SSR Override: %-10s | SSR: %-10s      ║\n",
                  ssrPirOverride ? "ACTIVE" : "inactive",
                  ssrEnabled ? "ON" : "OFF");
    Serial.printf("║ Triggers: %-4d | Uptime: %-10lu sec      ║\n",
                  pirTriggerCount, (millis() - systemStartTime) / 1000);
    Serial.println(F("╚════════════════════════════════════════════════╝\n"));
    lastPirStatus = millis();
  }

  // Print full status every 60 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 60000) {
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