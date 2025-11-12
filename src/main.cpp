/*
 * Vaulter v5.3 - PHILIPPINES FIXED VERSION - VOLTAGE CORRECTED
 * Server: https://meifhi-esp-server.onrender.com
 * Features: PIR Child Safety, Auto-Calibration, Self-Reliance, WebSocket Connectivity
 * 
 * PHILIPPINES ELECTRICAL STANDARDS:
 * - Voltage: 220V nominal (210-230V range)
 * - Frequency: 60Hz
 * - CRITICAL FIX: Corrected ZMPT101B sensitivity and calibration limits
 * 
 * VERSION 5.3 FIXES:
 * - CRITICAL: Fixed ZMPT101B sensitivity from 0.0053 to 0.004 (correct for standard module)
 * - CRITICAL: Added maximum calibration factor limits (max 10.0 instead of unlimited)
 * - CRITICAL: Added voltage reading sanity checks to prevent 20kV errors
 * - CRITICAL: Reset default voltage calibration to 1.0 for safety
 * - Improved calibration validation with stricter checks
 * - Added real-time voltage validation during operation
 * - All PIR improvements from v5.2 preserved
 * - Enhanced error reporting for voltage issues
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <WebSocketsClient.h>

#define ACS712_PIN          34
#define ZMPT101B_PIN        35
#define SSR_CONTROL_PIN     5
#define RED_LED_PIN         2
#define GREEN_LED_PIN       4
#define BUZZER_PIN          19
#define PIR_SENSOR_PIN      18
#define SD_CS_PIN           15

#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"
#define SERVER_HOST         "meifhi-esp-server.onrender.com"
#define SERVER_PORT         443
#define SERVER_ENDPOINT     "/api/data"
#define WS_PATH             "/ws"
#define DEVICE_ID           "VAULTER_001"
#define DEVICE_TYPE         "VAULTER"
#define WIFI_TIMEOUT_MS     20000
#define SERVER_SEND_INTERVAL 5000
#define SERVER_TIMEOUT      10000
#define MAX_RETRY_ATTEMPTS  3

#define ADC_RESOLUTION      4096.0
#define ADC_VREF            3.3
#define ADC_SAMPLES_FAST    2
#define ADC_SAMPLES_SLOW    12

// ACS712 Current Sensor (5A version)
#define ACS712_SENSITIVITY  0.185        // 185mV per Ampere for 5A version
#define ACS712_ZERO_CURRENT 2.5          // Zero current at 2.5V
#define ACS712_SUPPLY_VOLTAGE 5.0
#define VOLTAGE_DIVIDER_RATIO 0.667      // 3.3V ADC / 5V sensor
#define VOLTAGE_DIVIDER_SCALE 1.5
#define EXPECTED_OFFSET_MIN 1.35
#define EXPECTED_OFFSET_MAX 1.95
#define MAX_VALID_CURRENT   6.0

// ZMPT101B Voltage Sensor - CORRECTED FOR PHILIPPINES v5.3
// Standard ZMPT101B module specifications:
// - Transformer ratio: 1000:1 typically
// - For 220V RMS input, output is ~0.88V RMS centered at VCC/2 (1.65V)
// - Sensitivity calculation: 0.88V / 220V = 0.004 V/V (4mV per volt)
// CRITICAL FIX: Changed from 0.0053 to 0.004 (correct for standard ZMPT101B)
#define ZMPT101B_SENSITIVITY 0.004       // FIXED: 4mV per volt (was 0.0053)
#define AC_FREQUENCY        60           // Philippines uses 60Hz
#define AC_PERIOD_MS        16.67        // 1000ms/60Hz = 16.67ms
#define RMS_SAMPLES         100
#define RMS_SAMPLE_DELAY_US 150
#define MIN_AC_SWING        0.1
#define MIN_VALID_VOLTAGE   50.0

// Philippines Voltage Standards
#define NOMINAL_VOLTAGE     220.0        // Philippines nominal voltage
#define MAX_VOLTAGE         240.0        // Upper safe limit for Philippines
#define MIN_VOLTAGE         200.0        // Lower safe limit for Philippines
#define SENSOR_FAULT_VOLTAGE 280.0       // Fault detection threshold
#define BROWNOUT_VOLTAGE    180.0        // Brownout threshold
#define MAX_SENSOR_CURRENT  20.0
#define OVERCURRENT_TIME    1000
#define OVERVOLTAGE_TIME    500
#define SAFETY_DEBOUNCE_COUNT 5
#define BROWNOUT_TIME       2000

// CRITICAL: Calibration safety limits (v5.3)
#define MAX_VOLTAGE_CALIBRATION 10.0     // NEW: Prevent extreme calibration values
#define MIN_VOLTAGE_CALIBRATION 0.1      // NEW: Prevent division by near-zero
#define MAX_CURRENT_CALIBRATION 10.0     // NEW: Prevent extreme calibration values
#define MIN_CURRENT_CALIBRATION 0.1      // NEW: Prevent division by near-zero

// PIR SETTINGS - OPTIMIZED FOR FASTER RESPONSE AND STABLE OPERATION
#define PIR_ENABLED         true
#define PIR_MOTION_TIMEOUT  10000        // Time to wait after motion detected
#define PIR_NO_MOTION_TIME  3000         // Faster recovery when no load
#define PIR_CHECK_INTERVAL  50           // Check every 50ms for faster response
#define PIR_DEBOUNCE_TIME   100          // Faster debouncing
#define LOAD_DETECTION_THRESHOLD 0.08    // Slightly lower threshold for better detection
#define LOAD_STABLE_TIME    500          // Faster load detection
#define LOAD_LOST_TIME      1000         // Time to confirm load is actually lost
#define PIR_ALERT_BEEPS     2

#define SSR_ON_STATE        LOW
#define SSR_OFF_STATE       HIGH
#define SSR_VERIFY_INTERVAL 5000

#define MAX_CURRENT         4.5

#define AUTO_RECOVERY_ENABLED true
#define MAX_AUTO_RECOVERY_ATTEMPTS 3
#define RECOVERY_DELAY_MS   5000
#define SENSOR_FAULT_RECOVERY_TIME 30000
#define CRITICAL_ERROR_RESTART_TIME 300000

#define SAMPLE_RATE         1000
#define DISPLAY_INTERVAL    2000
#define CALIBRATION_SAMPLES 500
#define CALIBRATION_INTERVAL_MS 120000
#define MOVING_AVERAGE_SIZE 10
#define WATCHDOG_TIMEOUT_S  30
#define LOG_INTERVAL_MS     5000
#define MAX_CMD_LENGTH      64
#define SERIAL_RX_BUFFER    256
#define EEPROM_SIZE         512
#define EEPROM_MAGIC        0xDEADBEEF
#define DEFAULT_POWER_FACTOR 0.95
#define PERF_SAMPLES        100

enum SystemState {STATE_INITIALIZING, STATE_CALIBRATING, STATE_MONITORING, STATE_MANUAL_CONTROL, STATE_ERROR, STATE_RECOVERY, STATE_SHUTDOWN};
enum CalibrationState {CAL_IDLE, CAL_CURRENT_SAMPLING, CAL_VOLTAGE_SAMPLING, CAL_VERIFICATION, CAL_COMPLETE};
enum PIRState {PIR_INACTIVE, PIR_MONITORING, PIR_MOTION_DETECTED, PIR_COOLDOWN};

SystemState currentState = STATE_INITIALIZING;
CalibrationState calState = CAL_IDLE;
PIRState pirState = PIR_INACTIVE;

bool ssrEnabled = true;
bool ssrCommandState = true;
bool ssrStateBeforeCalibration = false;
bool autoCalibrationInProgress = false;
bool manualControl = false;
bool safetyEnabled = true;
bool buzzerEnabled = true;
bool sdCardAvailable = false;
bool wifiEnabled = false;
bool wifiConnected = false;
bool serverConnected = false;
bool wsConnected = false;
bool sensorsValid = false;
bool criticalError = false;

bool pirEnabled = PIR_ENABLED;
bool pirMotionDetected = false;
bool loadDetected = false;
bool pirManualOverride = false;
unsigned long lastMotionTime = 0;
unsigned long lastNoMotionTime = 0;
unsigned long lastPirCheck = 0;
unsigned long lastPirChange = 0;
unsigned long loadDetectedTime = 0;
unsigned long loadLostTime = 0;
int pirTriggerCount = 0;
int pirFalseAlarmCount = 0;

int autoRecoveryAttempts = 0;
unsigned long lastRecoveryAttempt = 0;
unsigned long sensorFaultStartTime = 0;
unsigned long criticalErrorStartTime = 0;
bool autoRecoveryMode = false;

float currentReading = 0.0;
float voltageReading = 0.0;
float powerReading = 0.0;
float apparentPower = 0.0;
float energyConsumed = 0.0;
float powerFactor = DEFAULT_POWER_FACTOR;

struct CalibrationData {
  uint32_t magic;
  float currentOffset;
  float voltageOffset;
  float currentCalibration;
  float voltageCalibration;
  float powerFactor;
  uint32_t crc;
};

// CRITICAL: Reset voltage calibration to 1.0 for safety (v5.3)
CalibrationData calData = {
  .magic = EEPROM_MAGIC,
  .currentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO,
  .voltageOffset = 1.65,
  .currentCalibration = 1.0,
  .voltageCalibration = 1.0,  // RESET: Was potentially corrupted causing 20kV readings
  .powerFactor = DEFAULT_POWER_FACTOR,
  .crc = 0
};

unsigned long overcurrentStartTime = 0;
unsigned long overvoltageStartTime = 0;
unsigned long brownoutStartTime = 0;
bool overcurrentDetected = false;
bool overvoltageDetected = false;
bool undervoltageDetected = false;
bool brownoutDetected = false;
bool sensorFaultDetected = false;
int overcurrentDebounceCount = 0;
int overvoltageDebounceCount = 0;
int sensorFaultDebounceCount = 0;

float currentBuffer[MOVING_AVERAGE_SIZE];
float voltageBuffer[MOVING_AVERAGE_SIZE];
int bufferIndex = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long lastEnergyUpdate = 0;
unsigned long lastLogUpdate = 0;
unsigned long lastSSRVerify = 0;
unsigned long lastCalibrationTime = 0;
unsigned long lastServerSend = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastWSReconnect = 0;
unsigned long systemStartTime = 0;
unsigned long lastWatchdogFeed = 0;
unsigned long lastSampleTime = 0;
unsigned long lastLoopTime = 0;

unsigned long totalOperatingTime = 0;
float maxCurrent = 0.0;
float maxVoltage = 0.0;
float minVoltage = 999.0;
float maxPower = 0.0;
unsigned long shutdownCount = 0;
unsigned long loopCount = 0;
int serverErrorCount = 0;
int serverSuccessCount = 0;
int voltageErrorCount = 0;  // NEW: Track voltage reading errors

struct PerformanceMetrics {
  unsigned long avgLoopTime;
  unsigned long maxLoopTime;
  unsigned long minLoopTime;
  uint32_t loopTimeSum;
  uint16_t sampleCount;
};
PerformanceMetrics perfMetrics = {0, 0, 999999, 0, 0};

struct CalibrationProgress {
  int samplesCollected;
  float currentSum;
  float voltageSum;
  unsigned long startTime;
};
CalibrationProgress calProgress = {0, 0.0, 0.0, 0};

WiFiClientSecure secureClient;
WebSocketsClient webSocket;

void startupSequence();
void autoCalibrateSensorsNonBlocking();
void calibrateVoltageWithReference();
void readSensors();
float calculateRMSVoltage(int sensorPin);
float calculateRMSCurrent();
void updateMovingAverages();
float getAverageCurrent();
float getAverageVoltage();
void performSafetyChecks();
void triggerSafetyShutdown(const char* reason);
void updateEnergyCalculation();
void updateStatistics();
void updateStatusLEDs();
void updateDisplay();
String getStateString();
String getPIRStateString();
void handleSerialCommands();
void processCommand(const char* command);
void printMenu();
void printStatistics();
void printDiagnostics();
void printMemoryUsage();
void saveCalibrationData();
bool loadCalibrationData();
void testBuzzer();
bool validateSensorReadings();
void initSDCard();
void logToSD(const char* data);
void setupWatchdog();
void feedWatchdog();
void emergencyReset();
float readRawSensorVoltage(int pin, int samples);
uint32_t calculateCRC32(const uint8_t* data, size_t length);
void verifySSRState();
void verifyCalibration();
void updatePerformanceMetrics(unsigned long loopTime);
bool isSafeToOperate();
void handleBrownout();
void connectToWiFi();
void checkWiFiConnection();
void sendDataToServer();
void initWebSocket();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void initPIRSensor();
void updatePIRChildSafety();
bool checkLoadPluggedIn();
void handlePIRMotion();
void handlePIRNoMotion();
void enablePIR();
void disablePIR();
void pirAlert();
void printPIRStatus();
void attemptAutoRecovery();
void recoverFromSensorFault();
void handleCriticalError();
bool performSelfDiagnostic();
void resetErrorCounters();
void validateCalibrationData();  // NEW: Validate calibration before use

void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(SERIAL_RX_BUFFER);
  delay(2000);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("ESP32 Vaulter v5.3 - VOLTAGE FIXED"));
  Serial.println(F("Server: meifhi-esp-server.onrender.com"));
  Serial.println(F("========================================"));
  Serial.println(F("PHILIPPINES ELECTRICAL STANDARDS:"));
  Serial.printf("Nominal Voltage: %.0fV\n", NOMINAL_VOLTAGE);
  Serial.printf("Frequency: %dHz\n", AC_FREQUENCY);
  Serial.printf("Voltage Range: %.0f-%.0fV\n", MIN_VOLTAGE, MAX_VOLTAGE);
  Serial.println(F("========================================"));
  Serial.println(F("VERSION 5.3 CRITICAL FIXES:"));
  Serial.println(F("✓ ZMPT101B sensitivity: 0.0053 → 0.004"));
  Serial.println(F("✓ Added calibration limits (max 10.0x)"));
  Serial.println(F("✓ Real-time voltage validation"));
  Serial.println(F("✓ Voltage calibration reset to 1.0"));
  Serial.println(F("✓ All PIR improvements preserved"));
  Serial.println(F("========================================"));
  
  EEPROM.begin(EEPROM_SIZE);
  
  pinMode(SSR_CONTROL_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_SENSOR_PIN, INPUT);
  
  digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    currentBuffer[i] = 0.0;
    voltageBuffer[i] = 0.0;
  }
  
  setupWatchdog();
  initSDCard();
  initPIRSensor();
  
  Serial.println(F("\n=== WIFI INITIALIZATION ==="));
  connectToWiFi();
  
  if (wifiConnected) {
    initWebSocket();
  }
  
  if (!loadCalibrationData()) {
    Serial.println(F("Using default calibration"));
  } else {
    // NEW: Validate loaded calibration data
    validateCalibrationData();
  }
  
  systemStartTime = millis();
  lastCalibrationTime = millis();
  currentState = STATE_CALIBRATING;
  calState = CAL_IDLE;
  
  startupSequence();
  printMenu();
  printMemoryUsage();
  
  Serial.println(F("\n*** SYSTEM READY ***"));
  Serial.printf("*** Server: %s ***\n", SERVER_HOST);
  Serial.println(F("Type 'help' for commands\n"));
}

void loop() {
  unsigned long loopStart = millis();
  
  if (loopStart - lastLoopTime < 10) return;
  
  feedWatchdog();
  handleSerialCommands();
  
  if (wifiConnected && wsConnected) {
    webSocket.loop();
  }
  
  if (currentState == STATE_RECOVERY) {
    attemptAutoRecovery();
    return;
  }
  
  if (currentState == STATE_ERROR && criticalError && AUTO_RECOVERY_ENABLED) {
    handleCriticalError();
  }
  
  if (currentState == STATE_CALIBRATING && calState != CAL_COMPLETE) {
    autoCalibrateSensorsNonBlocking();
    return;
  }
  
  if (sensorsValid && pirEnabled && !manualControl && currentState != STATE_ERROR) {
    updatePIRChildSafety();
  }
  
  if (sensorsValid && (loopStart - lastSampleTime >= SAMPLE_RATE)) {
    readSensors();
    updateMovingAverages();
    lastSampleTime = loopStart;
    
    if (!validateSensorReadings()) {
      sensorFaultDebounceCount++;
      if (sensorFaultDebounceCount >= SAFETY_DEBOUNCE_COUNT && !sensorFaultDetected) {
        sensorFaultDetected = true;
        sensorFaultStartTime = loopStart;
        
        if (AUTO_RECOVERY_ENABLED) {
          Serial.println(F("Sensor fault - initiating recovery"));
          currentState = STATE_RECOVERY;
        } else {
          triggerSafetyShutdown("SENSOR_FAULT");
        }
      }
    } else {
      sensorFaultDebounceCount = 0;
      if (sensorFaultDetected) {
        sensorFaultDetected = false;
        sensorFaultStartTime = 0;
        Serial.println(F("Sensor fault cleared"));
        resetErrorCounters();
      }
    }
    
    if (safetyEnabled && !manualControl && currentState != STATE_ERROR && pirState != PIR_MOTION_DETECTED) {
      performSafetyChecks();
    }
    
    updateEnergyCalculation();
  }
  
  if (loopStart - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    updateDisplay();
    updateStatusLEDs();
    lastDisplayUpdate = loopStart;
  }
  
  if (loopStart - lastSSRVerify >= SSR_VERIFY_INTERVAL) {
    verifySSRState();
    lastSSRVerify = loopStart;
  }
  
  if (loopStart - lastWiFiCheck >= 30000) {
    checkWiFiConnection();
    lastWiFiCheck = loopStart;
    
    if (wifiConnected && !wsConnected && (loopStart - lastWSReconnect > 30000)) {
      Serial.println(F("Reconnecting WebSocket..."));
      initWebSocket();
      lastWSReconnect = loopStart;
    }
  }
  
  if (sensorsValid && wifiConnected && (loopStart - lastServerSend >= SERVER_SEND_INTERVAL)) {
    sendDataToServer();
    lastServerSend = loopStart;
  }
  
  if (loopStart - lastLogUpdate >= LOG_INTERVAL_MS) {
    if (sdCardAvailable && sensorsValid) {
      char logBuffer[128];
      snprintf(logBuffer, sizeof(logBuffer), "%lu,%.1f,%.3f,%.1f,%d,%d,%d,%s",
               loopStart, voltageReading, currentReading, powerReading,
               ssrEnabled, pirEnabled, loadDetected, getStateString().c_str());
      logToSD(logBuffer);
    }
    lastLogUpdate = loopStart;
  }
  
  unsigned long loopTime = millis() - loopStart;
  updatePerformanceMetrics(loopTime);
  lastLoopTime = loopStart;
  loopCount++;
}

void startupSequence() {
  Serial.println(F("\n=== STARTUP SEQUENCE ==="));
  
  Serial.print(F("Testing RED LED... "));
  digitalWrite(RED_LED_PIN, HIGH);
  delay(300);
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println(F("OK"));
  
  Serial.print(F("Testing GREEN LED... "));
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(300);
  digitalWrite(GREEN_LED_PIN, LOW);
  Serial.println(F("OK"));
  
  Serial.print(F("Testing SSR... "));
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  delay(300);
  digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
  Serial.println(F("OK"));
  
  if (buzzerEnabled) {
    Serial.print(F("Testing BUZZER... "));
    testBuzzer();
    Serial.println(F("OK"));
  }
  
  Serial.println(F("========================\n"));
}

void autoCalibrateSensorsNonBlocking() {
  static unsigned long phaseStartTime = 0;
  unsigned long now = millis();
  
  switch (calState) {
    case CAL_IDLE:
      Serial.println(F("\n=== AUTO CALIBRATION ==="));
      Serial.println(F("Ensure NO LOAD connected"));
      Serial.println(F("[1/4] Sampling current offset..."));
      
      calProgress.samplesCollected = 0;
      calProgress.currentSum = 0.0;
      calProgress.startTime = now;
      phaseStartTime = now;
      calState = CAL_CURRENT_SAMPLING;
      break;
      
    case CAL_CURRENT_SAMPLING:
      if (calProgress.samplesCollected < CALIBRATION_SAMPLES) {
        float rawCurrent = readRawSensorVoltage(ACS712_PIN, ADC_SAMPLES_SLOW);
        calProgress.currentSum += rawCurrent;
        calProgress.samplesCollected++;
        
        if (calProgress.samplesCollected % 100 == 0) {
          Serial.printf("[1/4] Current: %d/%d samples\n", 
                       calProgress.samplesCollected, CALIBRATION_SAMPLES);
          feedWatchdog();
        }
      } else {
        calData.currentOffset = calProgress.currentSum / CALIBRATION_SAMPLES;
        Serial.printf("[1/4] Current offset: %.3fV\n", calData.currentOffset);
        
        if (calData.currentOffset < EXPECTED_OFFSET_MIN || calData.currentOffset > EXPECTED_OFFSET_MAX) {
          Serial.println(F("WARNING: Current offset out of expected range (1.35-1.95V)"));
          Serial.println(F("This may indicate ACS712 sensor issue"));
        }
        
        calProgress.samplesCollected = 0;
        calProgress.voltageSum = 0.0;
        phaseStartTime = now;
        calState = CAL_VOLTAGE_SAMPLING;
      }
      break;
      
    case CAL_VOLTAGE_SAMPLING:
      if (calProgress.samplesCollected < CALIBRATION_SAMPLES) {
        float rawVoltage = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
        calProgress.voltageSum += rawVoltage;
        calProgress.samplesCollected++;
        
        if (calProgress.samplesCollected % 100 == 0) {
          Serial.printf("[2/4] Voltage: %d/%d samples\n", 
                       calProgress.samplesCollected, CALIBRATION_SAMPLES);
          feedWatchdog();
        }
      } else {
        calData.voltageOffset = calProgress.voltageSum / CALIBRATION_SAMPLES;
        Serial.printf("[2/4] Voltage offset: %.3fV\n", calData.voltageOffset);
        
        if (calData.voltageOffset < 1.5 || calData.voltageOffset > 1.8) {
          Serial.println(F("WARNING: Voltage offset out of expected range (1.5-1.8V)"));
        }
        
        phaseStartTime = now;
        calState = CAL_VERIFICATION;
      }
      break;
      
    case CAL_VERIFICATION:
      Serial.println(F("[3/4] Verifying calibration..."));
      
      readSensors();
      
      Serial.printf("Initial readings:\n");
      Serial.printf("  Current: %.3f A\n", currentReading);
      Serial.printf("  Voltage: %.1f V\n", voltageReading);
      
      if (currentReading > 0.5) {
        Serial.println(F("ERROR: Load detected during calibration!"));
        Serial.println(F("Please disconnect load and restart calibration"));
        calState = CAL_IDLE;
      } else if (voltageReading < MIN_VALID_VOLTAGE || voltageReading > SENSOR_FAULT_VOLTAGE) {
        Serial.println(F("ERROR: Invalid voltage reading!"));
        Serial.printf("  Read: %.1fV (Expected: %.0f-%.0fV)\n", 
                     voltageReading, MIN_VALID_VOLTAGE, MAX_VOLTAGE);
        Serial.println(F("  Resetting voltage calibration to 1.0"));
        calData.voltageCalibration = 1.0;  // NEW: Reset if verification fails
        calState = CAL_IDLE;
      } else {
        Serial.println(F("[4/4] Calibration verification PASSED"));
        calState = CAL_COMPLETE;
      }
      break;
      
    case CAL_COMPLETE:
      Serial.println(F("\n=== CALIBRATION COMPLETE ==="));
      Serial.printf("Current offset: %.3fV\n", calData.currentOffset);
      Serial.printf("Voltage offset: %.3fV\n", calData.voltageOffset);
      Serial.printf("Current cal factor: %.3f\n", calData.currentCalibration);
      Serial.printf("Voltage cal factor: %.3f\n", calData.voltageCalibration);
      Serial.println(F("============================\n"));
      
      saveCalibrationData();
      sensorsValid = true;
      currentState = STATE_MONITORING;
      lastCalibrationTime = now;
      autoCalibrationInProgress = false;
      
      for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        currentBuffer[i] = 0.0;
        voltageBuffer[i] = 0.0;
      }
      break;
  }
}

// NEW v5.3: Improved calibration wizard with strict validation
void calibrateVoltageWithReference() {
  Serial.println(F("\n=== VOLTAGE CALIBRATION WIZARD ==="));
  Serial.println(F("This will help you calibrate voltage reading"));
  Serial.println(F("You need a multimeter to measure actual voltage\n"));
  
  Serial.print(F("Taking 20 voltage samples... "));
  float sum = 0.0;
  for (int i = 0; i < 20; i++) {
    sum += calculateRMSVoltage(ZMPT101B_PIN);
    delay(100);
    feedWatchdog();
  }
  float measuredVoltage = sum / 20.0;
  Serial.println(F("Done"));
  
  Serial.printf("\nCurrent reading: %.1f V\n", measuredVoltage);
  
  // NEW: Check if current reading is reasonable before proceeding
  if (measuredVoltage < 50.0 || measuredVoltage > 500.0) {
    Serial.println(F("\n*** ERROR: Current voltage reading out of range! ***"));
    Serial.printf("Reading: %.1fV is not reasonable for Philippines (should be ~220V)\n", measuredVoltage);
    Serial.println(F("Possible issues:"));
    Serial.println(F("  1. ZMPT101B sensor not connected properly"));
    Serial.println(F("  2. No AC voltage present"));
    Serial.println(F("  3. Sensor faulty"));
    Serial.println(F("\nCalibration aborted. Please check hardware.\n"));
    return;
  }
  
  Serial.println(F("\nEnter actual voltage from multimeter (e.g., 220):"));
  
  while (Serial.available() == 0) {
    feedWatchdog();
    delay(100);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  float actualVoltage = input.toFloat();
  
  if (actualVoltage < 100 || actualVoltage > 300) {
    Serial.println(F("Invalid voltage value! Must be between 100-300V"));
    return;
  }
  
  float newCalibration = actualVoltage / measuredVoltage * calData.voltageCalibration;
  
  // CRITICAL NEW: Apply calibration limits
  if (newCalibration > MAX_VOLTAGE_CALIBRATION) {
    Serial.println(F("\n*** WARNING: Calibration factor too high! ***"));
    Serial.printf("Calculated: %.3f (would cause extreme readings)\n", newCalibration);
    Serial.printf("Limiting to maximum: %.1f\n", MAX_VOLTAGE_CALIBRATION);
    newCalibration = MAX_VOLTAGE_CALIBRATION;
  } else if (newCalibration < MIN_VOLTAGE_CALIBRATION) {
    Serial.println(F("\n*** WARNING: Calibration factor too low! ***"));
    Serial.printf("Calculated: %.3f\n", newCalibration);
    Serial.printf("Limiting to minimum: %.1f\n", MIN_VOLTAGE_CALIBRATION);
    newCalibration = MIN_VOLTAGE_CALIBRATION;
  }
  
  Serial.printf("\nActual voltage: %.1f V\n", actualVoltage);
  Serial.printf("Measured voltage: %.1f V\n", measuredVoltage);
  Serial.printf("Old calibration: %.3f\n", calData.voltageCalibration);
  Serial.printf("New calibration: %.3f\n", newCalibration);
  Serial.printf("Error: %.1f%%\n", (measuredVoltage - actualVoltage) / actualVoltage * 100.0);
  
  calData.voltageCalibration = newCalibration;
  saveCalibrationData();
  
  Serial.println(F("\n✓ Voltage calibration updated"));
  Serial.println(F("Testing new calibration...\n"));
  
  delay(1000);
  sum = 0.0;
  for (int i = 0; i < 20; i++) {
    sum += calculateRMSVoltage(ZMPT101B_PIN);
    delay(100);
  }
  float newReading = sum / 20.0;
  
  Serial.printf("New reading: %.1f V\n", newReading);
  Serial.printf("Error: %.1f V (%.1f%%)\n", 
               newReading - actualVoltage,
               (newReading - actualVoltage) / actualVoltage * 100.0);
  
  // NEW: Final sanity check
  if (newReading < 150.0 || newReading > 300.0) {
    Serial.println(F("\n*** WARNING: Calibrated reading still out of range! ***"));
    Serial.println(F("Consider resetting calibration or checking hardware."));
  } else {
    Serial.println(F("✓ Calibration appears successful"));
  }
  
  Serial.println(F("============================\n"));
}

void readSensors() {
  currentReading = calculateRMSCurrent();
  
  if (currentReading > MAX_VALID_CURRENT) {
    Serial.printf("Invalid current: %.2fA\n", currentReading);
    currentReading = 0.0;
    sensorFaultDebounceCount++;
  }
  
  voltageReading = calculateRMSVoltage(ZMPT101B_PIN);
  
  // NEW v5.3: Real-time voltage validation
  if (voltageReading > SENSOR_FAULT_VOLTAGE) {
    voltageErrorCount++;
    if (voltageErrorCount >= 3) {
      Serial.printf("\n*** CRITICAL ERROR: Voltage reading too high! ***\n");
      Serial.printf("Reading: %.1fV (Normal: %.0fV)\n", voltageReading, NOMINAL_VOLTAGE);
      Serial.println(F("This indicates:"));
      Serial.println(F("  1. Incorrect ZMPT101B sensitivity setting"));
      Serial.println(F("  2. Corrupted voltage calibration factor"));
      Serial.println(F("  3. Sensor hardware fault"));
      Serial.println(F("\nResetting voltage calibration to 1.0 for safety..."));
      calData.voltageCalibration = 1.0;
      saveCalibrationData();
      voltageReading = calculateRMSVoltage(ZMPT101B_PIN);
      Serial.printf("New reading after reset: %.1fV\n\n", voltageReading);
      voltageErrorCount = 0;
    }
  } else {
    voltageErrorCount = 0;  // Reset counter if reading is valid
  }
  
  apparentPower = voltageReading * currentReading;
  powerReading = apparentPower * calData.powerFactor;
  updateStatistics();
}

float calculateRMSCurrent() {
  uint32_t sumSquares = 0;
  
  for (int i = 0; i < RMS_SAMPLES; i++) {
    float sensorVoltage = readRawSensorVoltage(ACS712_PIN, ADC_SAMPLES_FAST);
    float offsetVoltage = sensorVoltage - calData.currentOffset;
    float instantaneousCurrent = (offsetVoltage * VOLTAGE_DIVIDER_SCALE) / ACS712_SENSITIVITY;
    sumSquares += (instantaneousCurrent * instantaneousCurrent);
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  float rms = sqrt((float)sumSquares / RMS_SAMPLES);
  return rms * calData.currentCalibration;
}

// CRITICAL v5.3: Corrected voltage calculation with proper sensitivity
float calculateRMSVoltage(int sensorPin) {
  uint32_t sumSquares = 0;
  float maxV = 0, minV = 3.3;
  
  for (int i = 0; i < RMS_SAMPLES; i++) {
    float sensorVoltage = readRawSensorVoltage(sensorPin, ADC_SAMPLES_FAST);
    
    if (sensorVoltage > maxV) maxV = sensorVoltage;
    if (sensorVoltage < minV) minV = sensorVoltage;
    
    float offsetVoltage = sensorVoltage - calData.voltageOffset;
    
    // CRITICAL FIX v5.3: Using corrected sensitivity
    // With ZMPT101B_SENSITIVITY = 0.004:
    // For 220V RMS input → ~0.88V RMS sensor output
    // offsetVoltage = 0.88V, instantaneousVoltage = 0.88 / 0.004 = 220V ✓
    // Previous sensitivity 0.0053 would give: 0.88 / 0.0053 = 166V (wrong!)
    float instantaneousVoltage = offsetVoltage / ZMPT101B_SENSITIVITY;
    sumSquares += (instantaneousVoltage * instantaneousVoltage);
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  float rms = sqrt((float)sumSquares / RMS_SAMPLES);
  float scaledVoltage = rms * calData.voltageCalibration;
  
  // NEW v5.3: Additional runtime validation
  if (scaledVoltage > 500.0 && calData.voltageCalibration > 1.5) {
    // Extremely high reading with high calibration factor - likely error
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 5000) {
      Serial.println(F("\n*** WARNING: Abnormally high voltage detected! ***"));
      Serial.printf("Reading: %.1fV with calibration factor: %.3f\n", 
                   scaledVoltage, calData.voltageCalibration);
      Serial.println(F("Consider running 'calibrate_voltage' command"));
      lastWarning = millis();
    }
  }
  
  float swing = maxV - minV;
  if (swing < MIN_AC_SWING) {
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 10000) {
      Serial.printf("Low AC swing: %.3fV\n", swing);
      lastWarning = millis();
    }
  }
  
  return scaledVoltage;
}

void updateMovingAverages() {
  currentBuffer[bufferIndex] = currentReading;
  voltageBuffer[bufferIndex] = voltageReading;
  bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;
}

float getAverageCurrent() {
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) sum += currentBuffer[i];
  return sum / MOVING_AVERAGE_SIZE;
}

float getAverageVoltage() {
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) sum += voltageBuffer[i];
  return sum / MOVING_AVERAGE_SIZE;
}

// NEW v5.3: Enhanced validation with better error detection
bool validateSensorReadings() {
  // Check for extreme values that indicate sensor faults
  if (voltageReading > SENSOR_FAULT_VOLTAGE || currentReading > MAX_SENSOR_CURRENT ||
      isnan(voltageReading) || isnan(currentReading) || 
      isinf(voltageReading) || isinf(currentReading)) {
    sensorFaultDetected = true;
    
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 5000) {
      Serial.println(F("\n*** SENSOR VALIDATION FAILED ***"));
      Serial.printf("Voltage: %.1fV (Limit: %.0fV)\n", voltageReading, SENSOR_FAULT_VOLTAGE);
      Serial.printf("Current: %.3fA (Limit: %.0fA)\n", currentReading, MAX_SENSOR_CURRENT);
      lastErrorLog = millis();
    }
    
    return false;
  }
  
  // Check for unreasonably low voltage when AC should be present
  if (voltageReading < MIN_VALID_VOLTAGE && voltageReading > 10.0) {
    static unsigned long lastLowVoltageWarning = 0;
    if (millis() - lastLowVoltageWarning > 10000) {
      Serial.printf("WARNING: Low voltage detected: %.1fV\n", voltageReading);
      lastLowVoltageWarning = millis();
    }
  }
  
  sensorFaultDetected = false;
  return true;
}

void performSafetyChecks() {
  float avgCurrent = getAverageCurrent();
  float avgVoltage = getAverageVoltage();
  
  if (!sensorsValid) return;
  
  if (avgCurrent > MAX_CURRENT) {
    overcurrentDebounceCount++;
    if (!overcurrentDetected) {
      overcurrentDetected = true;
      overcurrentStartTime = millis();
      Serial.printf("Overcurrent: %.2fA\n", avgCurrent);
    }
    
    if (overcurrentDebounceCount >= SAFETY_DEBOUNCE_COUNT && 
        (millis() - overcurrentStartTime > OVERCURRENT_TIME)) {
      triggerSafetyShutdown("OVERCURRENT");
      return;
    }
  } else {
    overcurrentDetected = false;
    overcurrentDebounceCount = 0;
  }
  
  if (avgVoltage > MAX_VOLTAGE) {
    overvoltageDebounceCount++;
    if (!overvoltageDetected) {
      overvoltageDetected = true;
      overvoltageStartTime = millis();
      Serial.printf("Overvoltage: %.1fV\n", avgVoltage);
    }
    
    if (overvoltageDebounceCount >= SAFETY_DEBOUNCE_COUNT && 
        (millis() - overvoltageStartTime > OVERVOLTAGE_TIME)) {
      triggerSafetyShutdown("OVERVOLTAGE");
      return;
    }
  } else {
    overvoltageDetected = false;
    overvoltageDebounceCount = 0;
  }
  
  if (avgVoltage < MIN_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!undervoltageDetected) {
      undervoltageDetected = true;
      Serial.printf("Undervoltage: %.1fV\n", avgVoltage);
    }
  } else {
    undervoltageDetected = false;
  }
  
  if (avgVoltage < BROWNOUT_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!brownoutDetected) {
      brownoutDetected = true;
      brownoutStartTime = millis();
      Serial.printf("Brownout: %.1fV\n", avgVoltage);
    }
    
    if (millis() - brownoutStartTime > BROWNOUT_TIME) {
      handleBrownout();
    }
  } else {
    brownoutDetected = false;
  }
}

void triggerSafetyShutdown(const char* reason) {
  Serial.printf("\n*** SAFETY SHUTDOWN: %s ***\n", reason);
  
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  ssrEnabled = false;
  ssrCommandState = false;
  currentState = STATE_ERROR;
  
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  if (buzzerEnabled) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  }
  
  Serial.printf("Voltage: %.1fV\n", voltageReading);
  Serial.printf("Current: %.3fA\n", currentReading);
  Serial.printf("Power: %.1fW\n", powerReading);
  Serial.println(F("Type 'recovery' or 'reset' to recover\n"));
  
  shutdownCount++;
  
  if (sdCardAvailable) {
    char logBuffer[128];
    snprintf(logBuffer, sizeof(logBuffer), "SHUTDOWN,%s,%.1f,%.3f,%lu",
             reason, voltageReading, currentReading, millis());
    logToSD(logBuffer);
  }
}

void handleBrownout() {
  Serial.println(F("\n*** BROWNOUT CONDITION ***"));
  Serial.println(F("Extended low voltage detected"));
  
  if (autoRecoveryMode) {
    triggerSafetyShutdown("BROWNOUT");
  } else {
    Serial.println(F("Monitoring voltage..."));
  }
}

void updateEnergyCalculation() {
  unsigned long now = millis();
  if (lastEnergyUpdate == 0) {
    lastEnergyUpdate = now;
    return;
  }
  
  float hours = (now - lastEnergyUpdate) / 3600000.0;
  energyConsumed += (powerReading * hours);
  lastEnergyUpdate = now;
}

void updateStatistics() {
  if (currentReading > maxCurrent) maxCurrent = currentReading;
  if (voltageReading > maxVoltage) maxVoltage = voltageReading;
  if (voltageReading < minVoltage && voltageReading > MIN_VALID_VOLTAGE) minVoltage = voltageReading;
  if (powerReading > maxPower) maxPower = powerReading;
}

void updateStatusLEDs() {
  if (currentState == STATE_ERROR || sensorFaultDetected) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else if (pirState == PIR_MOTION_DETECTED) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else if (ssrEnabled && sensorsValid) {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}

void updateDisplay() {
  Serial.printf("V:%.1f I:%.3f P:%.1f SSR:%s PIR:%s Load:%s E:%.2f State:%s\n",
                voltageReading, currentReading, powerReading,
                ssrEnabled ? "ON" : "OFF",
                getPIRStateString().c_str(),
                loadDetected ? "YES" : "NO",
                energyConsumed,
                getStateString().c_str());
}

String getStateString() {
  switch (currentState) {
    case STATE_INITIALIZING: return "INIT";
    case STATE_CALIBRATING: return "CAL";
    case STATE_MONITORING: return "MONITOR";
    case STATE_MANUAL_CONTROL: return "MANUAL";
    case STATE_ERROR: return "ERROR";
    case STATE_RECOVERY: return "RECOVERY";
    case STATE_SHUTDOWN: return "SHUTDOWN";
    default: return "UNKNOWN";
  }
}

String getPIRStateString() {
  switch (pirState) {
    case PIR_INACTIVE: return "INACTIVE";
    case PIR_MONITORING: return "MONITOR";
    case PIR_MOTION_DETECTED: return "MOTION";
    case PIR_COOLDOWN: return "COOLDOWN";
    default: return "UNKNOWN";
  }
}

float readRawSensorVoltage(int pin, int samples) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float avgADC = (float)sum / samples;
  return (avgADC / ADC_RESOLUTION) * ADC_VREF;
}

void verifySSRState() {
  if (ssrCommandState != ssrEnabled) {
    Serial.println(F("SSR state mismatch - correcting..."));
    digitalWrite(SSR_CONTROL_PIN, ssrCommandState ? SSR_ON_STATE : SSR_OFF_STATE);
    ssrEnabled = ssrCommandState;
  }
}

// NEW v5.3: Comprehensive calibration verification
void verifyCalibration() {
  Serial.println(F("\n=== CALIBRATION CHECK ==="));
  Serial.printf("Current offset: %.3fV\n", calData.currentOffset);
  Serial.printf("Voltage offset: %.3fV\n", calData.voltageOffset);
  Serial.printf("Current cal: %.3f\n", calData.currentCalibration);
  Serial.printf("Voltage cal: %.3f\n", calData.voltageCalibration);
  Serial.printf("ZMPT Sensitivity: %.4f V/V\n", ZMPT101B_SENSITIVITY);
  
  bool valid = true;
  
  if (calData.currentOffset < EXPECTED_OFFSET_MIN || calData.currentOffset > EXPECTED_OFFSET_MAX) {
    Serial.println(F("WARNING: Current offset out of range"));
    valid = false;
  }
  
  if (calData.voltageOffset < 1.5 || calData.voltageOffset > 1.8) {
    Serial.println(F("WARNING: Voltage offset out of range"));
    valid = false;
  }
  
  // NEW: Check calibration factors
  if (calData.voltageCalibration > MAX_VOLTAGE_CALIBRATION || 
      calData.voltageCalibration < MIN_VOLTAGE_CALIBRATION) {
    Serial.println(F("*** ERROR: Voltage calibration factor out of safe range! ***"));
    Serial.printf("Current: %.3f (Safe range: %.1f - %.1f)\n", 
                 calData.voltageCalibration, MIN_VOLTAGE_CALIBRATION, MAX_VOLTAGE_CALIBRATION);
    Serial.println(F("Resetting to 1.0 for safety..."));
    calData.voltageCalibration = 1.0;
    saveCalibrationData();
    valid = false;
  }
  
  if (calData.currentCalibration > MAX_CURRENT_CALIBRATION || 
      calData.currentCalibration < MIN_CURRENT_CALIBRATION) {
    Serial.println(F("WARNING: Current calibration factor out of safe range"));
    valid = false;
  }
  
  Serial.printf("Status: %s\n", valid ? "OK" : "NEEDS CALIBRATION");
  Serial.println(F("========================\n"));
}

// NEW v5.3: Validate calibration data before use
void validateCalibrationData() {
  bool needsReset = false;
  
  Serial.println(F("\n=== VALIDATING CALIBRATION ==="));
  
  // Check voltage calibration factor
  if (calData.voltageCalibration > MAX_VOLTAGE_CALIBRATION) {
    Serial.printf("ERROR: Voltage cal too high: %.3f (max: %.1f)\n", 
                 calData.voltageCalibration, MAX_VOLTAGE_CALIBRATION);
    Serial.println(F("This would cause extremely high voltage readings!"));
    calData.voltageCalibration = 1.0;
    needsReset = true;
  } else if (calData.voltageCalibration < MIN_VOLTAGE_CALIBRATION) {
    Serial.printf("ERROR: Voltage cal too low: %.3f (min: %.1f)\n", 
                 calData.voltageCalibration, MIN_VOLTAGE_CALIBRATION);
    calData.voltageCalibration = 1.0;
    needsReset = true;
  }
  
  // Check current calibration factor
  if (calData.currentCalibration > MAX_CURRENT_CALIBRATION || 
      calData.currentCalibration < MIN_CURRENT_CALIBRATION) {
    Serial.printf("WARNING: Current cal out of range: %.3f\n", calData.currentCalibration);
    calData.currentCalibration = 1.0;
    needsReset = true;
  }
  
  if (needsReset) {
    Serial.println(F("\n*** CALIBRATION RESET TO SAFE DEFAULTS ***"));
    Serial.println(F("Reason: Potentially corrupted calibration data detected"));
    Serial.println(F("This prevents voltage readings like 20kV"));
    Serial.println(F("\nRecommendation: Run 'calibrate_voltage' after testing"));
    saveCalibrationData();
  } else {
    Serial.println(F("✓ Calibration data valid"));
  }
  
  Serial.println(F("==============================\n"));
}

void updatePerformanceMetrics(unsigned long loopTime) {
  perfMetrics.loopTimeSum += loopTime;
  perfMetrics.sampleCount++;
  
  if (loopTime > perfMetrics.maxLoopTime) perfMetrics.maxLoopTime = loopTime;
  if (loopTime < perfMetrics.minLoopTime) perfMetrics.minLoopTime = loopTime;
  
  if (perfMetrics.sampleCount >= PERF_SAMPLES) {
    perfMetrics.avgLoopTime = perfMetrics.loopTimeSum / perfMetrics.sampleCount;
    perfMetrics.loopTimeSum = 0;
    perfMetrics.sampleCount = 0;
  }
}

bool isSafeToOperate() {
  return sensorsValid && 
         !sensorFaultDetected && 
         !overcurrentDetected && 
         !overvoltageDetected && 
         voltageReading > MIN_VALID_VOLTAGE &&
         voltageReading < SENSOR_FAULT_VOLTAGE;
}

void setupWatchdog() {
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("Watchdog: %ds timeout\n", WATCHDOG_TIMEOUT_S);
}

void feedWatchdog() {
  unsigned long now = millis();
  if (now - lastWatchdogFeed >= 1000) {
    esp_task_wdt_reset();
    lastWatchdogFeed = now;
  }
}

void connectToWiFi() {
  if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
    Serial.println(F("WiFi: Disabled (no SSID configured)"));
    wifiEnabled = false;
    return;
  }
  
  wifiEnabled = true;
  Serial.print(F("Connecting to WiFi"));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(F("."));
    feedWatchdog();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println(F(" OK"));
    Serial.print(F("IP: "));
    Serial.println(WiFi.localIP());
    Serial.printf("Signal: %d dBm\n", WiFi.RSSI());
  } else {
    wifiConnected = false;
    Serial.println(F(" FAILED"));
    Serial.println(F("Running in standalone mode"));
  }
}

void checkWiFiConnection() {
  if (!wifiEnabled) return;
  
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println(F("WiFi disconnected"));
      wifiConnected = false;
      wsConnected = false;
    }
    
    Serial.print(F("Reconnecting to WiFi..."));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(F("."));
      feedWatchdog();
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F(" OK"));
      wifiConnected = true;
    } else {
      Serial.println(F(" FAILED"));
    }
  } else if (!wifiConnected) {
    wifiConnected = true;
    Serial.println(F("WiFi reconnected"));
  }
}

void initWebSocket() {
  if (!wifiConnected) return;
  
  Serial.println(F("\n=== WEBSOCKET INITIALIZATION ==="));
  secureClient.setInsecure();
  
  webSocket.beginSSL(SERVER_HOST, SERVER_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  Serial.printf("Connecting to: wss://%s:%d%s\n", SERVER_HOST, SERVER_PORT, WS_PATH);
  Serial.println(F("================================\n"));
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println(F("✗ WebSocket Disconnected"));
      wsConnected = false;
      break;
      
    case WStype_CONNECTED:
      {
        Serial.println(F("✓ WebSocket Connected"));
        Serial.printf("URL: %s\n", payload);
        wsConnected = true;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "register";
        doc["deviceId"] = DEVICE_ID;
        doc["deviceType"] = DEVICE_TYPE;
        
        String output;
        serializeJson(doc, output);
        webSocket.sendTXT(output);
        Serial.println(F("→ Registration sent"));
      }
      break;
      
    case WStype_TEXT:
      {
        Serial.printf("← WebSocket message: %s\n", payload);
        
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload);
        
        if (!error) {
          const char* type = doc["type"];
          
          if (strcmp(type, "welcome") == 0) {
            Serial.println(F("✓ Welcome message received"));
          }
          else if (strcmp(type, "registered") == 0) {
            Serial.println(F("✓ Device registered with server"));
          }
          else if (strcmp(type, "pong") == 0) {
            // Pong response
          }
          else if (doc.containsKey("command")) {
            const char* command = doc["command"];
            Serial.printf("→ Executing command: %s\n", command);
            processCommand(command);
            
            StaticJsonDocument<256> ackDoc;
            ackDoc["type"] = "commandAck";
            ackDoc["command"] = command;
            ackDoc["success"] = true;
            ackDoc["timestamp"] = millis();
            
            String ackOutput;
            serializeJson(ackDoc, ackOutput);
            webSocket.sendTXT(ackOutput);
          }
        }
      }
      break;
      
    case WStype_ERROR:
      Serial.println(F("✗ WebSocket Error"));
      break;
      
    case WStype_BIN:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    case WStype_PING:
    case WStype_PONG:
      break;
  }
}

void sendDataToServer() {
  if (!wifiConnected || !sensorsValid) return;
  
  StaticJsonDocument<512> doc;
  doc["deviceId"] = DEVICE_ID;
  doc["deviceType"] = DEVICE_TYPE;
  doc["voltage"] = round(voltageReading * 10.0) / 10.0;
  doc["current"] = round(currentReading * 1000.0) / 1000.0;
  doc["power"] = round(powerReading * 10.0) / 10.0;
  doc["energy"] = round(energyConsumed * 1000.0) / 1000.0;
  doc["ssrState"] = ssrEnabled;
  doc["state"] = getStateString();
  doc["sensors"] = "valid";
  doc["pirEnabled"] = pirEnabled;
  doc["pirState"] = getPIRStateString();
  doc["loadDetected"] = loadDetected;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  HTTPClient http;
  secureClient.setInsecure();
  
  String url = String("https://") + SERVER_HOST + SERVER_ENDPOINT;
  http.begin(secureClient, url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(SERVER_TIMEOUT);
  
  int httpCode = http.POST(jsonString);
  
  if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
    serverConnected = true;
    serverSuccessCount++;
    
    String response = http.getString();
    StaticJsonDocument<256> responseDoc;
    DeserializationError error = deserializeJson(responseDoc, response);
    
    if (!error && responseDoc.containsKey("command")) {
      const char* command = responseDoc["command"];
      Serial.printf("→ Server command: %s\n", command);
      processCommand(command);
    }
  } else {
    serverConnected = false;
    serverErrorCount++;
    
    static unsigned long lastError = 0;
    if (millis() - lastError > 30000) {
      Serial.printf("✗ Server error: %d\n", httpCode);
      lastError = millis();
    }
  }
  
  http.end();
}

void initPIRSensor() {
  pinMode(PIR_SENSOR_PIN, INPUT);
  delay(2000);
  
  Serial.print(F("PIR sensor: "));
  if (digitalRead(PIR_SENSOR_PIN) == LOW) {
    Serial.println(F("OK (Ready)"));
    if (pirEnabled) {
      pirState = PIR_MONITORING;
    }
  } else {
    Serial.println(F("WARNING (HIGH state)"));
  }
}

// IMPROVED PIR CHILD SAFETY WITH STABLE LOAD DETECTION
void updatePIRChildSafety() {
  unsigned long now = millis();
  
  // Check PIR at faster interval for quicker response
  if (now - lastPirCheck < PIR_CHECK_INTERVAL) return;
  lastPirCheck = now;
  
  // Read PIR sensor
  int pirValue = digitalRead(PIR_SENSOR_PIN);
  bool currentMotion = (pirValue == HIGH);
  
  // Check if load is plugged in (more stable detection)
  bool currentLoadState = checkLoadPluggedIn();
  
  // Handle load state changes with proper hysteresis
  if (currentLoadState != loadDetected) {
    if (currentLoadState) {
      // Load was just detected
      loadDetected = true;
      loadLostTime = 0;  // Reset lost timer
      Serial.println(F("Load DETECTED - PIR monitoring active"));
    } else {
      // Load might be lost - start confirmation timer
      if (loadLostTime == 0) {
        loadLostTime = now;
      } else if (now - loadLostTime > LOAD_LOST_TIME) {
        // Load confirmed lost after LOAD_LOST_TIME
        loadDetected = false;
        loadLostTime = 0;
        Serial.println(F("Load REMOVED - PIR standby"));
        
        // Restore power if it was off and no other safety issues
        if (!ssrEnabled && pirState != PIR_MOTION_DETECTED && 
            !sensorFaultDetected && !overcurrentDetected && 
            !overvoltageDetected && safetyEnabled) {
          digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
          ssrEnabled = true;
          ssrCommandState = true;
          Serial.println(F("Power restored (no load)"));
        }
        
        // Reset PIR state when no load
        if (pirState != PIR_INACTIVE) {
          pirState = PIR_MONITORING;
        }
      }
    }
  } else if (currentLoadState) {
    // Load is stable - reset lost timer
    loadLostTime = 0;
  }
  
  // Only process PIR motion if load is detected
  if (!loadDetected) {
    return;  // No load, no need to monitor motion
  }
  
  // Debounce PIR motion detection
  if (currentMotion != pirMotionDetected && now - lastPirChange > PIR_DEBOUNCE_TIME) {
    pirMotionDetected = currentMotion;
    lastPirChange = now;
    
    if (currentMotion) {
      // Motion detected
      lastMotionTime = now;
      if (pirState == PIR_MONITORING) {
        handlePIRMotion();
      }
    } else {
      // Motion stopped
      lastNoMotionTime = now;
    }
  }
  
  // Handle timeout after motion detected
  if (pirState == PIR_MOTION_DETECTED && now - lastMotionTime > PIR_MOTION_TIMEOUT) {
    handlePIRNoMotion();
  }
}

// IMPROVED: More stable load detection
bool checkLoadPluggedIn() {
  float avgCurrent = getAverageCurrent();
  
  // Check if current is above threshold
  if (avgCurrent > LOAD_DETECTION_THRESHOLD) {
    // Start tracking load detection time
    if (loadDetectedTime == 0) {
      loadDetectedTime = millis();
    } 
    // Confirm load after stable time
    else if (millis() - loadDetectedTime > LOAD_STABLE_TIME) {
      return true;  // Load confirmed
    }
    // Still waiting for stability
    return loadDetected;  // Keep previous state while waiting
  } else {
    // Current below threshold - reset detection timer
    loadDetectedTime = 0;
    
    // If load was detected, we need to wait for LOAD_LOST_TIME
    // before declaring it lost (handled in updatePIRChildSafety)
    return loadDetected;
  }
}

void handlePIRMotion() {
  Serial.println(F("\n*** PIR ALERT ***"));
  Serial.println(F("MOTION DETECTED with LOAD!"));
  Serial.println(F("Shutting down for safety"));
  
  pirState = PIR_MOTION_DETECTED;
  pirTriggerCount++;
  lastMotionTime = millis();
  
  // Turn off power immediately when motion detected with load
  if (ssrEnabled) {
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
    ssrEnabled = false;
    ssrCommandState = false;
    Serial.println(F("SSR OFF"));
  }
  
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  pirAlert();
  
  Serial.printf("OFF for %d sec\n", PIR_MOTION_TIMEOUT / 1000);
  Serial.println();
  
  if (sdCardAvailable) {
    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer), "PIR_MOTION,%lu", millis());
    logToSD(logBuffer);
  }
}

void handlePIRNoMotion() {
  Serial.println(F("\n*** PIR CLEAR ***"));
  Serial.println(F("No motion detected"));
  
  // Only re-enable if load is still connected
  if (loadDetected) {
    Serial.println(F("Re-enabling power (load still connected)"));
    
    pirState = PIR_MONITORING;
    
    if (!sensorFaultDetected && !overcurrentDetected && !overvoltageDetected && 
        currentState != STATE_ERROR && safetyEnabled) {
      digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
      ssrEnabled = true;
      ssrCommandState = true;
      Serial.println(F("SSR ON"));
    }
    
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    
    if (buzzerEnabled) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    Serial.println(F("Load removed - staying in monitoring mode"));
    pirState = PIR_MONITORING;
  }
  
  Serial.println();
}

void enablePIR() {
  pirEnabled = true;
  pirState = PIR_MONITORING;
  Serial.println(F("PIR: ENABLED"));
}

void disablePIR() {
  pirEnabled = false;
  pirState = PIR_INACTIVE;
  pirMotionDetected = false;
  loadDetected = false;
  Serial.println(F("PIR: DISABLED"));
}

void pirAlert() {
  if (!buzzerEnabled) return;
  for (int i = 0; i < PIR_ALERT_BEEPS; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

void printPIRStatus() {
  Serial.println(F("\n=== PIR STATUS ==="));
  Serial.printf("Enabled: %s\n", pirEnabled ? "YES" : "NO");
  Serial.printf("State: %s\n", getPIRStateString().c_str());
  Serial.printf("Motion: %s\n", pirMotionDetected ? "YES" : "NO");
  Serial.printf("Load: %s\n", loadDetected ? "YES" : "NO");
  Serial.printf("Triggers: %d\n", pirTriggerCount);
  Serial.printf("Current: %.3fA (Threshold: %.2fA)\n", getAverageCurrent(), LOAD_DETECTION_THRESHOLD);
  Serial.println(F("==================\n"));
}

void attemptAutoRecovery() {
  unsigned long now = millis();
  
  if (autoRecoveryAttempts >= MAX_AUTO_RECOVERY_ATTEMPTS) {
    Serial.println(F("\n*** MAX RECOVERY ATTEMPTS REACHED ***"));
    Serial.println(F("Manual intervention required"));
    Serial.println(F("Type 'reset' to restart system\n"));
    currentState = STATE_ERROR;
    criticalError = true;
    criticalErrorStartTime = now;
    return;
  }
  
  if (now - lastRecoveryAttempt < RECOVERY_DELAY_MS) {
    return;
  }
  
  Serial.printf("\n=== AUTO RECOVERY ATTEMPT %d/%d ===\n", 
               autoRecoveryAttempts + 1, MAX_AUTO_RECOVERY_ATTEMPTS);
  
  autoRecoveryAttempts++;
  lastRecoveryAttempt = now;
  
  if (sensorFaultDetected) {
    recoverFromSensorFault();
  }
  
  if (performSelfDiagnostic()) {
    Serial.println(F("✓ Recovery successful\n"));
    currentState = STATE_MONITORING;
    autoRecoveryMode = false;
    autoRecoveryAttempts = 0;
    resetErrorCounters();
  } else {
    Serial.println(F("✗ Recovery failed, will retry\n"));
  }
}

void recoverFromSensorFault() {
  Serial.println(F("Attempting sensor fault recovery..."));
  
  // Reset sensor readings
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    currentBuffer[i] = 0.0;
    voltageBuffer[i] = 0.0;
  }
  
  // Take fresh readings
  delay(1000);
  readSensors();
  
  Serial.printf("New readings: V=%.1fV I=%.3fA\n", voltageReading, currentReading);
}

void handleCriticalError() {
  unsigned long now = millis();
  
  if (now - criticalErrorStartTime > CRITICAL_ERROR_RESTART_TIME) {
    Serial.println(F("\n*** CRITICAL ERROR TIMEOUT ***"));
    Serial.println(F("Automatic restart in 5 seconds..."));
    delay(5000);
    ESP.restart();
  }
}

bool performSelfDiagnostic() {
  Serial.println(F("\nRunning self-diagnostic..."));
  
  bool passed = true;
  
  // Check sensor readings
  readSensors();
  if (!validateSensorReadings()) {
    Serial.println(F("✗ Sensor validation failed"));
    passed = false;
  } else {
    Serial.println(F("✓ Sensors OK"));
  }
  
  // Check voltage range
  if (voltageReading < MIN_VALID_VOLTAGE || voltageReading > SENSOR_FAULT_VOLTAGE) {
    Serial.println(F("✗ Voltage out of range"));
    passed = false;
  } else {
    Serial.println(F("✓ Voltage OK"));
  }
  
  // Check current
  if (currentReading > MAX_SENSOR_CURRENT) {
    Serial.println(F("✗ Current too high"));
    passed = false;
  } else {
    Serial.println(F("✓ Current OK"));
  }
  
  return passed;
}

void resetErrorCounters() {
  overcurrentDebounceCount = 0;
  overvoltageDebounceCount = 0;
  sensorFaultDebounceCount = 0;
  voltageErrorCount = 0;
  overcurrentDetected = false;
  overvoltageDetected = false;
  sensorFaultDetected = false;
}

void handleSerialCommands() {
  static char cmdBuffer[MAX_CMD_LENGTH];
  static uint8_t cmdIndex = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < MAX_CMD_LENGTH - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

void processCommand(const char* command) {
  String cmd = String(command);
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd == "help") {
    printMenu();
  }
  else if (cmd == "status") {
    printDiagnostics();
  }
  else if (cmd == "stats") {
    printStatistics();
  }
  else if (cmd == "calibrate") {
    currentState = STATE_CALIBRATING;
    calState = CAL_IDLE;
    autoCalibrationInProgress = true;
    Serial.println(F("Starting calibration..."));
  }
  else if (cmd == "calibrate_voltage") {
    calibrateVoltageWithReference();
  }
  else if (cmd == "verify" || cmd == "verify_calibration") {
    verifyCalibration();
  }
  else if (cmd == "on") {
    if (!manualControl) {
      Serial.println(F("Enable manual mode first"));
      return;
    }
    digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
    ssrEnabled = true;
    ssrCommandState = true;
    Serial.println(F("SSR: ON"));
  }
  else if (cmd == "off") {
    if (!manualControl) {
      Serial.println(F("Enable manual mode first"));
      return;
    }
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
    ssrEnabled = false;
    ssrCommandState = false;
    Serial.println(F("SSR: OFF"));
  }
  else if (cmd == "reset") {
    Serial.println(F("Resetting system..."));
    delay(1000);
    ESP.restart();
  }
  else if (cmd == "manual") {
    manualControl = !manualControl;
    currentState = manualControl ? STATE_MANUAL_CONTROL : STATE_MONITORING;
    Serial.printf("Manual mode: %s\n", manualControl ? "ON" : "OFF");
  }
  else if (cmd == "safety") {
    safetyEnabled = !safetyEnabled;
    Serial.printf("Safety: %s\n", safetyEnabled ? "ON" : "OFF");
  }
  else if (cmd == "buzzer") {
    buzzerEnabled = !buzzerEnabled;
    Serial.printf("Buzzer: %s\n", buzzerEnabled ? "ON" : "OFF");
  }
  else if (cmd == "pir_on" || cmd == "pir_enable") {
    enablePIR();
  }
  else if (cmd == "pir_off" || cmd == "pir_disable") {
    disablePIR();
  }
  else if (cmd == "pir_status" || cmd == "pir") {
    printPIRStatus();
  }
  else if (cmd == "recovery" || cmd == "recover") {
    Serial.println(F("Attempting recovery..."));
    currentState = STATE_RECOVERY;
    autoRecoveryAttempts = 0;
  }
  else if (cmd == "clear") {
    maxCurrent = 0.0;
    maxVoltage = 0.0;
    minVoltage = 999.0;
    maxPower = 0.0;
    energyConsumed = 0.0;
    shutdownCount = 0;
    Serial.println(F("Statistics cleared"));
  }
  else if (cmd.startsWith("power_factor ")) {
    float pf = cmd.substring(13).toFloat();
    if (pf > 0 && pf <= 1.0) {
      calData.powerFactor = pf;
      powerFactor = pf;
      saveCalibrationData();
      Serial.printf("Power factor: %.2f\n", pf);
    } else {
      Serial.println(F("Invalid power factor (0-1.0)"));
    }
  }
  else if (cmd == "test_buzzer") {
    testBuzzer();
  }
  else if (cmd == "mem" || cmd == "memory") {
    printMemoryUsage();
  }
  else if (cmd == "reset_calibration") {
    Serial.println(F("\n*** RESETTING CALIBRATION TO DEFAULTS ***"));
    calData.voltageCalibration = 1.0;
    calData.currentCalibration = 1.0;
    saveCalibrationData();
    Serial.println(F("✓ Calibration reset complete"));
    Serial.println(F("Run 'calibrate_voltage' to recalibrate\n"));
  }
  else {
    Serial.println(F("Unknown command. Type 'help'"));
  }
}

void printMenu() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    VAULTER v5.3 COMMANDS"));
  Serial.println(F("========================================"));
  Serial.println(F("STATUS:"));
  Serial.println(F("  help     - Show commands"));
  Serial.println(F("  status   - Show diagnostics"));
  Serial.println(F("  stats    - Show statistics"));
  Serial.println(F("  mem      - Memory usage"));
  Serial.println();
  Serial.println(F("CONTROL:"));
  Serial.println(F("  on       - Turn on SSR"));
  Serial.println(F("  off      - Turn off SSR"));
  Serial.println(F("  reset    - Restart system"));
  Serial.println();
  Serial.println(F("PIR:"));
  Serial.println(F("  pir_on   - Enable PIR"));
  Serial.println(F("  pir_off  - Disable PIR"));
  Serial.println(F("  pir      - PIR status"));
  Serial.println();
  Serial.println(F("CALIBRATION (v5.3 FIXES):"));
  Serial.println(F("  calibrate         - Auto calibration"));
  Serial.println(F("  calibrate_voltage - Manual voltage cal"));
  Serial.println(F("  verify            - Check calibration"));
  Serial.println(F("  reset_calibration - Reset to defaults"));
  Serial.println();
  Serial.println(F("SETTINGS:"));
  Serial.println(F("  manual       - Toggle manual"));
  Serial.println(F("  safety       - Toggle safety"));
  Serial.println(F("  buzzer       - Toggle buzzer"));
  Serial.println(F("  power_factor - Set power factor"));
  Serial.println(F("  clear        - Clear stats"));
  Serial.println(F("========================================\n"));
}

void printStatistics() {
  Serial.println(F("\n=== STATISTICS ==="));
  unsigned long uptime = (millis() - systemStartTime) / 1000;
  Serial.printf("Uptime: %02lu:%02lu:%02lu\n", uptime/3600, (uptime%3600)/60, uptime%60);
  Serial.printf("Loops: %lu\n", loopCount);
  Serial.printf("Energy: %.3f Wh (%.2f kWh)\n", energyConsumed, energyConsumed / 1000.0);
  Serial.printf("Max I: %.3f A\n", maxCurrent);
  Serial.printf("Max V: %.1f V\n", maxVoltage);
  Serial.printf("Min V: %.1f V\n", minVoltage);
  Serial.printf("Max P: %.1f W\n", maxPower);
  Serial.printf("Shutdowns: %lu\n", shutdownCount);
  Serial.printf("PIR triggers: %d\n", pirTriggerCount);
  Serial.printf("Recovery attempts: %d\n", autoRecoveryAttempts);
  Serial.printf("Sensors: %s\n", sensorsValid ? "Valid" : "Invalid");
  Serial.println(F("==================\n"));
}

void printDiagnostics() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    DIAGNOSTICS"));
  Serial.println(F("========================================"));
  
  Serial.println(F("\n--- SYSTEM ---"));
  Serial.printf("Version: v5.3 Philippines Fixed + Voltage Corrected\n");
  Serial.printf("Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("CPU: %d MHz\n", ESP.getCpuFreqMHz());
  
  printMemoryUsage();
  
  Serial.println(F("\n--- ELECTRICAL STANDARDS ---"));
  Serial.printf("Nominal Voltage: %.0fV\n", NOMINAL_VOLTAGE);
  Serial.printf("Frequency: %dHz\n", AC_FREQUENCY);
  Serial.printf("Voltage Range: %.0f-%.0fV\n", MIN_VOLTAGE, MAX_VOLTAGE);
  
  Serial.println(F("\n--- SENSORS ---"));
  Serial.printf("Valid: %s\n", sensorsValid ? "YES" : "NO");
  Serial.printf("Current: %.3f A (Avg: %.3f)\n", currentReading, getAverageCurrent());
  Serial.printf("Voltage: %.1f V (Avg: %.1f)\n", voltageReading, getAverageVoltage());
  Serial.printf("Power: %.1f W\n", powerReading);
  Serial.printf("Voltage errors: %d\n", voltageErrorCount);
  
  Serial.println(F("\n--- PIR ---"));
  Serial.printf("Enabled: %s\n", pirEnabled ? "YES" : "NO");
  Serial.printf("State: %s\n", getPIRStateString().c_str());
  Serial.printf("Load: %s\n", loadDetected ? "YES" : "NO");
  Serial.printf("Triggers: %d\n", pirTriggerCount);
  Serial.printf("Check Interval: %dms\n", PIR_CHECK_INTERVAL);
  Serial.printf("Debounce: %dms\n", PIR_DEBOUNCE_TIME);
  Serial.printf("Load Threshold: %.2fA\n", LOAD_DETECTION_THRESHOLD);
  
  Serial.println(F("\n--- CALIBRATION (v5.3) ---"));
  Serial.printf("Current offset: %.3f V\n", calData.currentOffset);
  Serial.printf("Voltage offset: %.3f V\n", calData.voltageOffset);
  Serial.printf("Current cal: %.3f (Max: %.1f)\n", calData.currentCalibration, MAX_CURRENT_CALIBRATION);
  Serial.printf("Voltage cal: %.3f (Max: %.1f)\n", calData.voltageCalibration, MAX_VOLTAGE_CALIBRATION);
  Serial.printf("Power factor: %.2f\n", calData.powerFactor);
  Serial.printf("ZMPT Sensitivity: %.4f V/V (FIXED)\n", ZMPT101B_SENSITIVITY);
  
  Serial.println(F("\n--- SAFETY ---"));
  Serial.printf("Enabled: %s\n", safetyEnabled ? "YES" : "NO");
  Serial.printf("Manual: %s\n", manualControl ? "YES" : "NO");
  Serial.printf("Overcurrent: %s\n", overcurrentDetected ? "YES" : "NO");
  Serial.printf("Overvoltage: %s\n", overvoltageDetected ? "YES" : "NO");
  Serial.printf("Sensor fault: %s\n", sensorFaultDetected ? "YES" : "NO");
  
  Serial.println(F("\n--- SSR ---"));
  Serial.printf("Enabled: %s\n", ssrEnabled ? "YES" : "NO");
  Serial.printf("Command: %s\n", ssrCommandState ? "ON" : "OFF");
  
  Serial.println(F("\n--- NETWORK ---"));
  Serial.printf("WiFi: %s\n", wifiConnected ? "YES" : "NO");
  if (wifiConnected) {
    Serial.print(F("IP: "));
    Serial.println(WiFi.localIP());
    Serial.printf("Signal: %d dBm\n", WiFi.RSSI());
  }
  Serial.printf("Server: %s\n", serverConnected ? "YES" : "NO");
  Serial.printf("WebSocket: %s\n", wsConnected ? "YES" : "NO");
  Serial.printf("Host: %s\n", SERVER_HOST);
  Serial.printf("Device: %s\n", DEVICE_ID);
  
  int totalRequests = serverSuccessCount + serverErrorCount;
  float successRate = totalRequests > 0 ? (float)serverSuccessCount / totalRequests * 100.0 : 0;
  Serial.printf("\nServer Stats:\n");
  Serial.printf("  Success: %d\n", serverSuccessCount);
  Serial.printf("  Errors: %d\n", serverErrorCount);
  Serial.printf("  Rate: %.1f%%\n", successRate);
  
  Serial.println(F("\n--- PERFORMANCE ---"));
  if (perfMetrics.avgLoopTime > 0) {
    Serial.printf("Avg: %lu ms\n", perfMetrics.avgLoopTime);
    Serial.printf("Min: %lu ms\n", perfMetrics.minLoopTime);
    Serial.printf("Max: %lu ms\n", perfMetrics.maxLoopTime);
  }
  
  Serial.println(F("\n========================================\n"));
}

void printMemoryUsage() {
  Serial.println(F("\n--- MEMORY ---"));
  Serial.printf("Free: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Size: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Min free: %d bytes\n", ESP.getMinFreeHeap());
  
  float usage = 100.0 * (1.0 - (float)ESP.getFreeHeap() / ESP.getHeapSize());
  Serial.printf("Usage: %.1f%%\n", usage);
  
  if (usage > 80.0) Serial.println(F("WARNING: High memory"));
}

uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

void saveCalibrationData() {
  calData.magic = EEPROM_MAGIC;
  calData.crc = calculateCRC32((uint8_t*)&calData, sizeof(CalibrationData) - sizeof(uint32_t));
  EEPROM.put(0, calData);
  EEPROM.commit();
  Serial.println(F("Saved to EEPROM"));
}

bool loadCalibrationData() {
  CalibrationData loadedData;
  EEPROM.get(0, loadedData);
  
  if (loadedData.magic != EEPROM_MAGIC) {
    Serial.println(F("EEPROM: Invalid magic"));
    return false;
  }
  
  uint32_t calculatedCRC = calculateCRC32((uint8_t*)&loadedData, sizeof(CalibrationData) - sizeof(uint32_t));
  if (calculatedCRC != loadedData.crc) {
    Serial.println(F("EEPROM: CRC mismatch"));
    return false;
  }
  
  if (isnan(loadedData.currentOffset) || loadedData.currentOffset < 0.5 || loadedData.currentOffset > 3.0 ||
      isnan(loadedData.voltageOffset) || loadedData.voltageOffset < 0.5 || loadedData.voltageOffset > 3.0 ||
      isnan(loadedData.currentCalibration) || loadedData.currentCalibration <= 0 || loadedData.currentCalibration > 100.0 ||
      isnan(loadedData.voltageCalibration) || loadedData.voltageCalibration <= 0 || loadedData.voltageCalibration > 1000.0 ||
      isnan(loadedData.powerFactor) || loadedData.powerFactor <= 0 || loadedData.powerFactor > 1.0) {
    Serial.println(F("EEPROM: Invalid data"));
    return false;
  }
  
  calData = loadedData;
  Serial.println(F("Loaded from EEPROM"));
  return true;
}

void initSDCard() {
  Serial.print(F("SD card: "));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAIL"));
    sdCardAvailable = false;
    return;
  }
  
  sdCardAvailable = true;
  Serial.println(F("OK"));
  
  if (!SD.exists("/datalog.csv")) {
    File file = SD.open("/datalog.csv", FILE_WRITE);
    if (file) {
      file.println("Time,V,I,P,SSR,PIR,Load,State");
      file.close();
      Serial.println(F("Created datalog.csv"));
    }
  }
}

void logToSD(const char* data) {
  if (!sdCardAvailable) return;
  File file = SD.open("/datalog.csv", FILE_APPEND);
  if (!file) return;
  file.println(data);
  file.close();
}

void testBuzzer() {
  if (!buzzerEnabled) return;
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
    feedWatchdog();
  }
}

void emergencyReset() {
  Serial.println(F("\nEMERGENCY RESET"));
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  ssrEnabled = false;
  delay(2000);
  ESP.restart();
}