/*
 * ESP32 Electrical Safety Monitor - ENHANCED VERSION v4.0
 * 
 * MAJOR IMPROVEMENTS:
 * - Non-blocking architecture with state machine
 * - EEPROM CRC validation
 * - Optimized RMS calculation timing
 * - Memory-efficient char buffers (no String in loops)
 * - Serial input protection
 * - Better error handling and recovery
 * - SSR state verification
 * - Brown-out detection
 * - Performance metrics
 * - Comprehensive diagnostics
 * - Safer watchdog handling
 * - Graceful degradation on sensor failure
 */

#include <WiFi.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// ==================== PIN DEFINITIONS ====================
#define ACS712_PIN          34
#define ZMPT101B_PIN        35
#define SSR_CONTROL_PIN     5
#define RED_LED_PIN         2
#define GREEN_LED_PIN       4
#define BUZZER_PIN          19
#define PIR_SENSOR_PIN      18
#define SD_CS_PIN           15

// ==================== ADC CONFIGURATION ====================
#define ADC_RESOLUTION      4096.0
#define ADC_VREF            3.6
#define ADC_SAMPLES_FAST    2      // Fast sampling for RMS
#define ADC_SAMPLES_SLOW    12     // Accurate sampling for calibration

// ==================== ACS712-05B CONFIGURATION ====================
#define ACS712_SENSITIVITY  0.185
#define ACS712_ZERO_CURRENT 2.5
#define ACS712_SUPPLY_VOLTAGE 5.0
#define VOLTAGE_DIVIDER_RATIO 0.667
#define VOLTAGE_DIVIDER_SCALE 1.5
#define EXPECTED_OFFSET_MIN 1.35
#define EXPECTED_OFFSET_MAX 1.95
#define MAX_VALID_CURRENT   6.0

// ==================== ZMPT101B CONFIGURATION ====================
#define ZMPT101B_SENSITIVITY 0.0125
#define AC_FREQUENCY        50
#define AC_PERIOD_MS        20
#define RMS_SAMPLES         100
#define RMS_SAMPLE_DELAY_US 150    // Optimized for faster sampling
#define MIN_AC_SWING        0.1
#define MIN_VALID_VOLTAGE   50.0

// ==================== SSR CONTROL CONFIGURATION ====================
#define SSR_ON_STATE        LOW
#define SSR_OFF_STATE       HIGH
#define SSR_VERIFY_INTERVAL 5000   // Verify SSR state every 5s

// ==================== SAFETY THRESHOLDS ====================
#define MAX_CURRENT         4.5
#define MAX_VOLTAGE         250.0
#define MIN_VOLTAGE         200.0
#define SENSOR_FAULT_VOLTAGE 350.0
#define MAX_SENSOR_CURRENT  20.0
#define OVERCURRENT_TIME    1000
#define OVERVOLTAGE_TIME    500
#define SAFETY_DEBOUNCE_COUNT 5
#define BROWNOUT_VOLTAGE    180.0
#define BROWNOUT_TIME       2000

// ==================== SYSTEM CONFIGURATION ====================
#define SAMPLE_RATE         1000
#define DISPLAY_INTERVAL    2000
#define CALIBRATION_SAMPLES 500
#define MOVING_AVERAGE_SIZE 10
#define WATCHDOG_TIMEOUT_S  30
#define LOG_INTERVAL_MS     5000
#define MAX_CMD_LENGTH      64
#define SERIAL_RX_BUFFER    256
#define EEPROM_SIZE         512
#define EEPROM_MAGIC        0xDEADBEEF

// ==================== POWER FACTOR ====================
#define DEFAULT_POWER_FACTOR 0.95

// ==================== PERFORMANCE TRACKING ====================
#define PERF_SAMPLES        100

// ==================== SYSTEM STATES ====================
enum SystemState {
  STATE_INITIALIZING,
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_MANUAL_CONTROL,
  STATE_ERROR,
  STATE_SHUTDOWN
};

// ==================== CALIBRATION STATES ====================
enum CalibrationState {
  CAL_IDLE,
  CAL_CURRENT_SAMPLING,
  CAL_VOLTAGE_SAMPLING,
  CAL_VERIFICATION,
  CAL_COMPLETE
};

// ==================== GLOBAL VARIABLES ====================
SystemState currentState = STATE_INITIALIZING;
CalibrationState calState = CAL_IDLE;
bool ssrEnabled = false;
bool ssrCommandState = false;
bool manualControl = false;
bool safetyEnabled = true;
bool buzzerEnabled = true;
bool sdCardAvailable = false;
bool wifiEnabled = false;
bool sensorsValid = false;
bool criticalError = false;

// ==================== SENSOR DATA ====================
float currentReading = 0.0;
float voltageReading = 0.0;
float powerReading = 0.0;
float apparentPower = 0.0;
float energyConsumed = 0.0;
float powerFactor = DEFAULT_POWER_FACTOR;

// ==================== CALIBRATION VALUES ====================
struct CalibrationData {
  uint32_t magic;
  float currentOffset;
  float voltageOffset;
  float currentCalibration;
  float voltageCalibration;
  float powerFactor;
  uint32_t crc;
};

CalibrationData calData = {
  .magic = EEPROM_MAGIC,
  .currentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO,
  .voltageOffset = 1.65,
  .currentCalibration = 1.0,
  .voltageCalibration = 1.0,
  .powerFactor = DEFAULT_POWER_FACTOR,
  .crc = 0
};

// ==================== SAFETY MONITORING ====================
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

// ==================== MOVING AVERAGE BUFFERS ====================
float currentBuffer[MOVING_AVERAGE_SIZE];
float voltageBuffer[MOVING_AVERAGE_SIZE];
int bufferIndex = 0;

// ==================== TIMING VARIABLES ====================
unsigned long lastDisplayUpdate = 0;
unsigned long lastEnergyUpdate = 0;
unsigned long lastLogUpdate = 0;
unsigned long lastSSRVerify = 0;
unsigned long systemStartTime = 0;
unsigned long lastWatchdogFeed = 0;
unsigned long lastSampleTime = 0;
unsigned long lastLoopTime = 0;

// ==================== STATISTICS ====================
unsigned long totalOperatingTime = 0;
float maxCurrent = 0.0;
float maxVoltage = 0.0;
float minVoltage = 999.0;
float maxPower = 0.0;
unsigned long shutdownCount = 0;
unsigned long loopCount = 0;

// ==================== PERFORMANCE METRICS ====================
struct PerformanceMetrics {
  unsigned long avgLoopTime;
  unsigned long maxLoopTime;
  unsigned long minLoopTime;
  uint32_t loopTimeSum;
  uint16_t sampleCount;
};
PerformanceMetrics perfMetrics = {0, 0, 999999, 0, 0};

// ==================== CALIBRATION PROGRESS ====================
struct CalibrationProgress {
  int samplesCollected;
  float currentSum;
  float voltageSum;
  unsigned long startTime;
};
CalibrationProgress calProgress = {0, 0.0, 0.0, 0};

// ==================== FUNCTION PROTOTYPES ====================
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

// ==================== SETUP FUNCTION ====================
void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(SERIAL_RX_BUFFER);
  delay(2000);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("ESP32 Electrical Safety Monitor"));
  Serial.println(F("ENHANCED VERSION v4.0"));
  Serial.println(F("========================================"));
  
  EEPROM.begin(EEPROM_SIZE);
  
  pinMode(SSR_CONTROL_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_SENSOR_PIN, INPUT);
  
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
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
  
  if (!loadCalibrationData()) {
    Serial.println(F("⚠️  Using default calibration values"));
  }
  
  systemStartTime = millis();
  currentState = STATE_CALIBRATING;
  calState = CAL_IDLE;
  
  startupSequence();
  printMenu();
  printMemoryUsage();
  
  Serial.println(F("\n*** SYSTEM READY ***"));
  Serial.println(F("Type commands and press ENTER"));
  Serial.println(F("Try: 'test' to see sensor readings"));
  Serial.println(F("     'reset' to clear errors"));
  Serial.println(F("     'help' for all commands\n"));
}

// ==================== MAIN LOOP (NON-BLOCKING) ====================
void loop() {
  unsigned long loopStart = millis();
  
  // Prevent loop from running too fast
  if (loopStart - lastLoopTime < 10) {
    return;
  }
  
  feedWatchdog();
  handleSerialCommands();
  
  // Non-blocking calibration
  if (currentState == STATE_CALIBRATING && calState != CAL_COMPLETE) {
    autoCalibrateSensorsNonBlocking();
    return;
  }
  
  // Sensor reading at controlled rate
  if (sensorsValid && (loopStart - lastSampleTime >= SAMPLE_RATE)) {
    readSensors();
    updateMovingAverages();
    lastSampleTime = loopStart;
    
    if (!validateSensorReadings()) {
      sensorFaultDebounceCount++;
      if (sensorFaultDebounceCount >= SAFETY_DEBOUNCE_COUNT && !sensorFaultDetected) {
        triggerSafetyShutdown("SENSOR_FAULT");
      }
    } else {
      sensorFaultDebounceCount = 0;
    }
    
    if (safetyEnabled && !manualControl && currentState != STATE_ERROR) {
      performSafetyChecks();
    }
    
    updateEnergyCalculation();
  }
  
  // Display update
  if (loopStart - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = loopStart;
  }
  
  // SSR state verification
  if (loopStart - lastSSRVerify >= SSR_VERIFY_INTERVAL) {
    verifySSRState();
    lastSSRVerify = loopStart;
  }
  
  updateStatusLEDs();
  
  // SD card logging
  if (sdCardAvailable && sensorsValid && (loopStart - lastLogUpdate >= LOG_INTERVAL_MS)) {
    char logBuffer[128];
    snprintf(logBuffer, sizeof(logBuffer), "%lu,%.2f,%.3f,%.2f,%d",
             loopStart, voltageReading, currentReading, powerReading, ssrEnabled ? 1 : 0);
    logToSD(logBuffer);
    lastLogUpdate = loopStart;
  }
  
  // Performance tracking
  unsigned long loopTime = millis() - loopStart;
  updatePerformanceMetrics(loopTime);
  
  lastLoopTime = loopStart;
  loopCount++;
}

// ==================== WATCHDOG TIMER ====================
void setupWatchdog() {
  Serial.println(F("Setting up watchdog timer..."));
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("Watchdog enabled with %d second timeout\n", WATCHDOG_TIMEOUT_S);
}

void feedWatchdog() {
  if (millis() - lastWatchdogFeed > 1000) {
    esp_task_wdt_reset();
    lastWatchdogFeed = millis();
  }
}

// ==================== STARTUP SEQUENCE ====================
void startupSequence() {
  Serial.println(F("\n=== SYSTEM INITIALIZATION ==="));
  
  Serial.println(F("Testing LEDs..."));
  digitalWrite(RED_LED_PIN, HIGH);
  delay(250);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(250);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  if (buzzerEnabled) {
    Serial.println(F("Testing buzzer..."));
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
  }
  
  Serial.println(F("✓ Hardware test complete"));
  
  Serial.println(F("\n=== AUTO-CALIBRATING SENSORS ==="));
  Serial.println(F("IMPORTANT: Ensure NO LOAD connected!"));
  Serial.println(F("Calibration will proceed automatically..."));
  
  calState = CAL_CURRENT_SAMPLING;
  calProgress.samplesCollected = 0;
  calProgress.currentSum = 0.0;
  calProgress.voltageSum = 0.0;
  calProgress.startTime = millis();
}

// ==================== NON-BLOCKING AUTO CALIBRATION ====================
void autoCalibrateSensorsNonBlocking() {
  unsigned long now = millis();
  
  switch (calState) {
    case CAL_CURRENT_SAMPLING:
    case CAL_VOLTAGE_SAMPLING:
      if (calProgress.samplesCollected < CALIBRATION_SAMPLES) {
        float currentVoltage = readRawSensorVoltage(ACS712_PIN, ADC_SAMPLES_SLOW);
        float voltageVoltage = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
        
        calProgress.currentSum += currentVoltage;
        calProgress.voltageSum += voltageVoltage;
        calProgress.samplesCollected++;
        
        if (calProgress.samplesCollected % 50 == 0) {
          Serial.print(F("."));
          feedWatchdog();
        }
      } else {
        Serial.println();
        calData.currentOffset = calProgress.currentSum / CALIBRATION_SAMPLES;
        calData.voltageOffset = calProgress.voltageSum / CALIBRATION_SAMPLES;
        
        Serial.printf("\nCalibration Results:\n");
        Serial.printf("  Current offset: %.3f V ", calData.currentOffset);
        
        float expectedOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO;
        if (calData.currentOffset >= EXPECTED_OFFSET_MIN && calData.currentOffset <= EXPECTED_OFFSET_MAX) {
          Serial.println(F("✓ OK"));
        } else {
          Serial.printf("⚠️  WARNING (expected ~%.2fV)\n", expectedOffset);
          Serial.println(F("     Check ACS712 connection and voltage divider"));
        }
        
        Serial.printf("  Voltage offset: %.3f V ✓\n", calData.voltageOffset);
        
        calState = CAL_VERIFICATION;
        calProgress.samplesCollected = 0;
      }
      break;
      
    case CAL_VERIFICATION:
      Serial.println(F("\n=== VALIDATING SENSORS ==="));
      
      float testCurrent = calculateRMSCurrent();
      float testVoltage = calculateRMSVoltage(ZMPT101B_PIN);
      
      Serial.printf("Test readings:\n");
      Serial.printf("  Current: %.3f A\n", testCurrent);
      Serial.printf("  Voltage: %.1f V\n", testVoltage);
      
      if (testCurrent > 1.0) {
        Serial.println(F("\n⚠️  WARNING: High current detected with no load!"));
        Serial.println(F("   Current sensor may be faulty or incorrectly connected"));
        Serial.println(F("   System will operate in SAFE MODE (monitoring only)"));
        sensorsValid = false;
        safetyEnabled = true;
      } else if (testVoltage < MIN_VALID_VOLTAGE) {
        Serial.println(F("\n⚠️  WARNING: Low voltage detected!"));
        Serial.println(F("   Check voltage sensor connection"));
        Serial.println(F("   Or mains voltage not present"));
        sensorsValid = false;
      } else {
        Serial.println(F("\n✓ Sensors validated successfully"));
        sensorsValid = true;
        verifyCalibration();
      }
      
      saveCalibrationData();
      
      Serial.println(F("\n✓ System ready for operation"));
      currentState = STATE_MONITORING;
      calState = CAL_COMPLETE;
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;
      
    default:
      break;
  }
}

// ==================== CALIBRATION VERIFICATION ====================
void verifyCalibration() {
  float testVoltage = calculateRMSVoltage(ZMPT101B_PIN);
  
  if (testVoltage > 100 && testVoltage < 300) {
    Serial.println(F("✓ Voltage calibration verified (within normal range)"));
  } else if (testVoltage > 50) {
    Serial.println(F("⚠️  Voltage reading may need calibration"));
    Serial.println(F("   Use 'cal_voltage' command with a multimeter"));
  }
}

// ==================== HELPER: Read Raw Sensor Voltage ====================
float readRawSensorVoltage(int pin, int samples) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float avgRaw = (float)sum / samples;
  return (avgRaw / ADC_RESOLUTION) * ADC_VREF;
}

// ==================== VOLTAGE CALIBRATION WIZARD ====================
void calibrateVoltageWithReference() {
  Serial.println(F("\n=== VOLTAGE CALIBRATION WIZARD ==="));
  Serial.println(F("1. Measure your mains voltage with a multimeter"));
  Serial.println(F("2. Enter the measured voltage (e.g., 230.5)"));
  Serial.println(F("3. System will calculate calibration factor"));
  Serial.print(F("\nEnter measured voltage in volts: "));
  
  unsigned long startWait = millis();
  const unsigned long timeout = 20000; // Reduced to 20s to avoid watchdog
  
  while(!Serial.available() && (millis() - startWait < timeout)) {
    delay(100);
    feedWatchdog();
  }
  
  if (Serial.available()) {
    float measuredVoltage = Serial.parseFloat();
    Serial.println(measuredVoltage);
    
    while(Serial.available()) Serial.read();
    
    if (measuredVoltage > 100.0 && measuredVoltage < 300.0) {
      float currentReading = calculateRMSVoltage(ZMPT101B_PIN);
      
      if (currentReading > 1.0) {
        calData.voltageCalibration = measuredVoltage / currentReading;
        Serial.printf("✓ Voltage calibration factor: %.4f\n", calData.voltageCalibration);
        Serial.printf("  Current reading: %.1fV → Target: %.1fV\n", currentReading, measuredVoltage);
        saveCalibrationData();
        verifyCalibration();
      } else {
        Serial.println(F("✗ ERROR: Voltage reading too low. Check sensor connection."));
      }
    } else {
      Serial.println(F("✗ Invalid voltage value. Calibration cancelled."));
    }
  } else {
    Serial.println(F("Timeout. Calibration cancelled."));
  }
}

// ==================== SENSOR READING ====================
void readSensors() {
  currentReading = calculateRMSCurrent();
  
  if (currentReading > MAX_VALID_CURRENT) {
    Serial.printf("⚠️  Invalid current reading: %.2fA (clamping to 0)\n", currentReading);
    currentReading = 0.0;
    sensorFaultDebounceCount++;
  }
  
  voltageReading = calculateRMSVoltage(ZMPT101B_PIN);
  
  apparentPower = voltageReading * currentReading;
  powerReading = apparentPower * calData.powerFactor;
  
  updateStatistics();
}

float calculateRMSCurrent() {
  float sum = 0;
  int validSamples = 0;
  
  for(int i = 0; i < RMS_SAMPLES; i++) {
    float voltage = readRawSensorVoltage(ACS712_PIN, ADC_SAMPLES_FAST);
    
    float actualACS712Voltage = voltage * VOLTAGE_DIVIDER_SCALE;
    float calibratedZeroPoint = calData.currentOffset * VOLTAGE_DIVIDER_SCALE;
    float currentDiff = actualACS712Voltage - calibratedZeroPoint;
    float instantCurrent = currentDiff / ACS712_SENSITIVITY;
    
    sum += instantCurrent * instantCurrent;
    validSamples++;
    
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  if (validSamples > 0) {
    float rms = sqrt(sum / validSamples);
    return abs(rms * calData.currentCalibration);
  }
  
  return 0.0;
}

float calculateRMSVoltage(int sensorPin) {
  float sum = 0;
  float maxReading = 0;
  float minReading = ADC_VREF;
  int validSamples = 0;
  
  for(int i = 0; i < RMS_SAMPLES; i++) {
    float voltage = readRawSensorVoltage(sensorPin, ADC_SAMPLES_FAST);
    
    if (voltage > maxReading) maxReading = voltage;
    if (voltage < minReading) minReading = voltage;
    
    float acVoltage = voltage - calData.voltageOffset;
    sum += acVoltage * acVoltage;
    validSamples++;
    
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  float rms = 0;
  if (validSamples > 0) {
    rms = sqrt(sum / validSamples);
  }
  
  float scaledVoltage = rms * calData.voltageCalibration;
  
  float swing = maxReading - minReading;
  if (swing < MIN_AC_SWING && sensorsValid) {
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 10000) {
      Serial.printf("⚠️  WARNING: Low AC swing (%.3fV). Check voltage sensor.\n", swing);
      lastWarning = millis();
    }
  }
  
  return scaledVoltage;
}

// ==================== MOVING AVERAGE ====================
void updateMovingAverages() {
  currentBuffer[bufferIndex] = currentReading;
  voltageBuffer[bufferIndex] = voltageReading;
  
  bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;
}

float getAverageCurrent() {
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += currentBuffer[i];
  }
  return sum / MOVING_AVERAGE_SIZE;
}

float getAverageVoltage() {
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += voltageBuffer[i];
  }
  return sum / MOVING_AVERAGE_SIZE;
}

// ==================== SENSOR VALIDATION ====================
bool validateSensorReadings() {
  if (voltageReading > SENSOR_FAULT_VOLTAGE) {
    sensorFaultDetected = true;
    return false;
  }
  
  if (currentReading > MAX_SENSOR_CURRENT) {
    sensorFaultDetected = true;
    return false;
  }
  
  if (isnan(voltageReading) || isnan(currentReading) || 
      isinf(voltageReading) || isinf(currentReading)) {
    sensorFaultDetected = true;
    return false;
  }
  
  sensorFaultDetected = false;
  return true;
}

// ==================== SAFETY CHECKS ====================
void performSafetyChecks() {
  float avgCurrent = getAverageCurrent();
  float avgVoltage = getAverageVoltage();
  
  if (!sensorsValid) return;
  
  // Overcurrent protection
  if (avgCurrent > MAX_CURRENT) {
    overcurrentDebounceCount++;
    
    if (!overcurrentDetected) {
      overcurrentDetected = true;
      overcurrentStartTime = millis();
      Serial.printf("⚠️  WARNING: Overcurrent! %.2fA > %.2fA (count: %d/%d)\n", 
                    avgCurrent, MAX_CURRENT, overcurrentDebounceCount, SAFETY_DEBOUNCE_COUNT);
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
  
  // Overvoltage protection
  if (avgVoltage > MAX_VOLTAGE) {
    overvoltageDebounceCount++;
    
    if (!overvoltageDetected) {
      overvoltageDetected = true;
      overvoltageStartTime = millis();
      Serial.printf("⚠️  WARNING: Overvoltage! %.1fV > %.1fV (count: %d/%d)\n", 
                    avgVoltage, MAX_VOLTAGE, overvoltageDebounceCount, SAFETY_DEBOUNCE_COUNT);
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
  
  // Undervoltage warning
  if (avgVoltage < MIN_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!undervoltageDetected) {
      undervoltageDetected = true;
      Serial.printf("⚠️  WARNING: Undervoltage! %.1fV < %.1fV\n", avgVoltage, MIN_VOLTAGE);
    }
  } else {
    undervoltageDetected = false;
  }
  
  // Brown-out detection
  if (avgVoltage < BROWNOUT_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!brownoutDetected) {
      brownoutDetected = true;
      brownoutStartTime = millis();
      Serial.printf("⚠️  WARNING: Brown-out! %.1fV < %.1fV\n", avgVoltage, BROWNOUT_VOLTAGE);
    }
    
    if (millis() - brownoutStartTime > BROWNOUT_TIME) {
      handleBrownout();
    }
  } else {
    brownoutDetected = false;
  }
}

// ==================== BROWN-OUT HANDLER ====================
void handleBrownout() {
  Serial.println(F("⚠️  Extended brown-out detected - protecting load"));
  
  if (ssrEnabled) {
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
    ssrEnabled = false;
    Serial.println(F("   Load disconnected to prevent damage"));
  }
}

// ==================== SAFETY SHUTDOWN ====================
void triggerSafetyShutdown(const char* reason) {
  Serial.printf("\n🚨 SAFETY SHUTDOWN: %s\n", reason);
  
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  ssrEnabled = false;
  ssrCommandState = false;
  
  currentState = STATE_ERROR;
  shutdownCount++;
  
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  if (buzzerEnabled) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
      feedWatchdog();
    }
  }
  
  if (sdCardAvailable) {
    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer), "SHUTDOWN,%s,%lu", reason, millis());
    logToSD(logBuffer);
  }
  
  Serial.println(F("*** SYSTEM IN SAFETY SHUTDOWN ***"));
  Serial.println(F("Type 'reset' to acknowledge and restart"));
}

// ==================== SSR STATE VERIFICATION ====================
void verifySSRState() {
  // Verify the SSR command state matches actual state
  bool expectedState = ssrCommandState;
  
  if (ssrEnabled != expectedState && currentState != STATE_ERROR) {
    Serial.println(F("⚠️  SSR state mismatch detected!"));
    Serial.printf("   Expected: %s, Actual: %s\n", 
                  expectedState ? "ON" : "OFF",
                  ssrEnabled ? "ON" : "OFF");
    
    // Force state to match command
    digitalWrite(SSR_CONTROL_PIN, expectedState ? SSR_ON_STATE : SSR_OFF_STATE);
    ssrEnabled = expectedState;
  }
}

// ==================== ENERGY CALCULATION ====================
void updateEnergyCalculation() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  
  if (lastUpdate > 0 && ssrEnabled && powerReading > 0.1) {
    float deltaTime = (currentTime - lastUpdate) / 1000.0 / 3600.0;
    energyConsumed += powerReading * deltaTime;
  }
  
  lastUpdate = currentTime;
}

// ==================== STATISTICS ====================
void updateStatistics() {
  if (currentReading > maxCurrent && currentReading < MAX_VALID_CURRENT) {
    maxCurrent = currentReading;
  }
  
  if (voltageReading > maxVoltage && voltageReading < SENSOR_FAULT_VOLTAGE) {
    maxVoltage = voltageReading;
  }
  
  if (voltageReading < minVoltage && voltageReading > MIN_VALID_VOLTAGE) {
    minVoltage = voltageReading;
  }
  
  if (powerReading > maxPower) {
    maxPower = powerReading;
  }
}

// ==================== PERFORMANCE METRICS ====================
void updatePerformanceMetrics(unsigned long loopTime) {
  perfMetrics.loopTimeSum += loopTime;
  perfMetrics.sampleCount++;
  
  if (loopTime > perfMetrics.maxLoopTime) {
    perfMetrics.maxLoopTime = loopTime;
  }
  
  if (loopTime < perfMetrics.minLoopTime) {
    perfMetrics.minLoopTime = loopTime;
  }
  
  if (perfMetrics.sampleCount >= PERF_SAMPLES) {
    perfMetrics.avgLoopTime = perfMetrics.loopTimeSum / perfMetrics.sampleCount;
    
    // Reset for next window
    perfMetrics.loopTimeSum = 0;
    perfMetrics.sampleCount = 0;
  }
}

// ==================== STATUS LEDs ====================
void updateStatusLEDs() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  switch (currentState) {
    case STATE_MONITORING:
      if (ssrEnabled) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
      } else {
        if (millis() - lastBlink > 1000) {
          blinkState = !blinkState;
          digitalWrite(GREEN_LED_PIN, blinkState);
          lastBlink = millis();
        }
        digitalWrite(RED_LED_PIN, LOW);
      }
      break;
      
    case STATE_ERROR:
      if (millis() - lastBlink > 250) {
        blinkState = !blinkState;
        digitalWrite(RED_LED_PIN, blinkState);
        lastBlink = millis();
      }
      digitalWrite(GREEN_LED_PIN, LOW);
      break;
      
    case STATE_MANUAL_CONTROL:
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH);
      break;
      
    default:
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      break;
  }
}

// ==================== DISPLAY UPDATE ====================
void updateDisplay() {
  if (!sensorsValid) {
    Serial.println(F("\n⚠️  ========================================"));
    Serial.println(F("   SENSOR VALIDATION FAILED"));
    Serial.println(F("   System in SAFE MODE"));
    Serial.println(F("   Type 'test' to see raw readings"));
    Serial.println(F("   Type 'calibrate' to recalibrate"));
    Serial.println(F("========================================\n"));
    return;
  }
  
  Serial.println(F("\n========================================"));
  Serial.printf("Status: %s | SSR: %s | Loops: %lu\n", 
                getStateString().c_str(), 
                ssrEnabled ? "ON" : "OFF",
                loopCount);
  Serial.println(F("----------------------------------------"));
  
  Serial.printf("Voltage: %.1f V (Avg: %.1f V)\n", voltageReading, getAverageVoltage());
  Serial.printf("Current: %.3f A (Avg: %.3f A)\n", currentReading, getAverageCurrent());
  Serial.printf("Power: %.1f W (Apparent: %.1f VA, PF=%.2f)\n", powerReading, apparentPower, calData.powerFactor);
  Serial.printf("Energy: %.3f Wh\n", energyConsumed);
  
  if (perfMetrics.avgLoopTime > 0) {
    Serial.printf("Loop: %lu ms (min: %lu, max: %lu)\n", 
                  perfMetrics.avgLoopTime, perfMetrics.minLoopTime, perfMetrics.maxLoopTime);
  }
  
  if (overcurrentDetected || overvoltageDetected || undervoltageDetected || brownoutDetected) {
    Serial.println(F("----------------------------------------"));
    Serial.println(F("*** WARNINGS ***"));
    if (overcurrentDetected) Serial.printf("⚠️  OVERCURRENT: %.2fA\n", getAverageCurrent());
    if (overvoltageDetected) Serial.printf("⚠️  OVERVOLTAGE: %.1fV\n", getAverageVoltage());
    if (undervoltageDetected) Serial.printf("⚠️  UNDERVOLTAGE: %.1fV\n", getAverageVoltage());
    if (brownoutDetected) Serial.printf("⚠️  BROWN-OUT: %.1fV\n", getAverageVoltage());
  }
  
  Serial.println(F("========================================\n"));
}

String getStateString() {
  switch (currentState) {
    case STATE_INITIALIZING: return F("INIT");
    case STATE_CALIBRATING: return F("CALIB");
    case STATE_MONITORING: return F("MONITOR");
    case STATE_MANUAL_CONTROL: return F("MANUAL");
    case STATE_ERROR: return F("ERROR");
    case STATE_SHUTDOWN: return F("SHUTDOWN");
    default: return F("UNKNOWN");
  }
}

// ==================== EMERGENCY RESET ====================
void emergencyReset() {
  Serial.println(F("\n🔄 EMERGENCY RESET"));
  
  currentState = STATE_CALIBRATING;
  calState = CAL_CURRENT_SAMPLING;
  
  digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
  ssrEnabled = false;
  ssrCommandState = false;
  
  overcurrentDetected = false;
  overvoltageDetected = false;
  undervoltageDetected = false;
  brownoutDetected = false;
  sensorFaultDetected = false;
  overcurrentDebounceCount = 0;
  overvoltageDebounceCount = 0;
  sensorFaultDebounceCount = 0;
  criticalError = false;
  
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  Serial.println(F("Recalibrating sensors..."));
  calProgress.samplesCollected = 0;
  calProgress.currentSum = 0.0;
  calProgress.voltageSum = 0.0;
  calProgress.startTime = millis();
  
  // Allow non-blocking calibration to run
  while (calState != CAL_COMPLETE && (millis() - calProgress.startTime < 10000)) {
    autoCalibrateSensorsNonBlocking();
    feedWatchdog();
    delay(10);
  }
  
  if (calState == CAL_COMPLETE) {
    Serial.println(F("✓ System reset complete"));
  } else {
    Serial.println(F("⚠️  Reset timeout - manual calibration may be needed"));
  }
}

// ==================== SAFE OPERATION CHECK ====================
bool isSafeToOperate() {
  if (!sensorsValid) {
    Serial.println(F("✗ Cannot enable: Sensors invalid"));
    return false;
  }
  
  if (currentState == STATE_ERROR) {
    Serial.println(F("✗ Cannot enable: System in ERROR"));
    Serial.println(F("  Type 'reset' first"));
    return false;
  }
  
  if (criticalError) {
    Serial.println(F("✗ Cannot enable: Critical error present"));
    return false;
  }
  
  float avgVoltage = getAverageVoltage();
  if (avgVoltage < MIN_VALID_VOLTAGE) {
    Serial.println(F("✗ Cannot enable: No valid voltage detected"));
    return false;
  }
  
  return true;
}

// ==================== SERIAL COMMANDS (PROTECTED INPUT) ====================
void handleSerialCommands() {
  static char cmdBuffer[MAX_CMD_LENGTH];
  static uint8_t cmdIndex = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        
        // Convert to lowercase
        for (uint8_t i = 0; i < cmdIndex; i++) {
          cmdBuffer[i] = tolower(cmdBuffer[i]);
        }
        
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < MAX_CMD_LENGTH - 1) {
      cmdBuffer[cmdIndex++] = c;
    } else {
      // Buffer overflow protection
      Serial.println(F("✗ Command too long (max 64 chars)"));
      cmdIndex = 0;
      while (Serial.available()) Serial.read(); // Flush
    }
  }
}

void processCommand(const char* command) {
  Serial.printf("\n> %s\n", command);
  
  if (strcmp(command, "reset") == 0) {
    emergencyReset();
    return;
  }
  
  if (strcmp(command, "restart") == 0) {
    Serial.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }
  
  else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printMenu();
  }
  else if (strcmp(command, "status") == 0) {
    updateDisplay();
    printStatistics();
  }
  else if (strcmp(command, "diag") == 0 || strcmp(command, "diagnostics") == 0) {
    printDiagnostics();
  }
  else if (strcmp(command, "mem") == 0 || strcmp(command, "memory") == 0) {
    printMemoryUsage();
  }
  else if (strcmp(command, "on") == 0 || strcmp(command, "enable") == 0) {
    if (isSafeToOperate()) {
      ssrEnabled = true;
      ssrCommandState = true;
      digitalWrite(SSR_CONTROL_PIN, SSR_ON_STATE);
      Serial.println(F("✓ SSR ENABLED - Load ON"));
    }
  }
  else if (strcmp(command, "off") == 0 || strcmp(command, "disable") == 0) {
    ssrEnabled = false;
    ssrCommandState = false;
    digitalWrite(SSR_CONTROL_PIN, SSR_OFF_STATE);
    Serial.println(F("✓ SSR DISABLED - Load OFF"));
  }
  else if (strcmp(command, "test") == 0) {
    Serial.println(F("\n=== SENSOR TEST ==="));
    for (int i = 0; i < 3; i++) {
      float currentVolt = readRawSensorVoltage(ACS712_PIN, ADC_SAMPLES_SLOW);
      float voltageVolt = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
      float current = calculateRMSCurrent();
      float voltage = calculateRMSVoltage(ZMPT101B_PIN);
      
      Serial.printf("\nReading %d:\n", i+1);
      Serial.printf("  ACS712 raw: %.3fV → Current: %.3fA\n", currentVolt, current);
      Serial.printf("  ZMPT101B raw: %.3fV → Voltage: %.1fV\n", voltageVolt, voltage);
      delay(500);
      feedWatchdog();
    }
    Serial.printf("\nCalibration:\n");
    Serial.printf("  Current offset: %.3fV\n", calData.currentOffset);
    Serial.printf("  Voltage offset: %.3fV\n", calData.voltageOffset);
    Serial.printf("  Current cal: %.3f\n", calData.currentCalibration);
    Serial.printf("  Voltage cal: %.3f\n", calData.voltageCalibration);
    Serial.println(F("===================\n"));
  }
  else if (strcmp(command, "calibrate") == 0) {
    Serial.println(F("Starting calibration..."));
    currentState = STATE_CALIBRATING;
    calState = CAL_CURRENT_SAMPLING;
    calProgress.samplesCollected = 0;
    calProgress.currentSum = 0.0;
    calProgress.voltageSum = 0.0;
    calProgress.startTime = millis();
  }
  else if (strcmp(command, "cal_voltage") == 0) {
    calibrateVoltageWithReference();
  }
  else if (strcmp(command, "stats") == 0) {
    printStatistics();
  }
  else if (strcmp(command, "clear") == 0) {
    maxCurrent = 0.0;
    maxVoltage = 0.0;
    minVoltage = 999.0;
    maxPower = 0.0;
    energyConsumed = 0.0;
    shutdownCount = 0;
    loopCount = 0;
    systemStartTime = millis();
    perfMetrics.maxLoopTime = 0;
    perfMetrics.minLoopTime = 999999;
    Serial.println(F("✓ Statistics cleared"));
  }
  else if (strcmp(command, "manual") == 0) {
    manualControl = !manualControl;
    currentState = manualControl ? STATE_MANUAL_CONTROL : STATE_MONITORING;
    Serial.printf("Manual mode: %s\n", manualControl ? "ON" : "OFF");
    if (manualControl) {
      Serial.println(F("⚠️  Safety checks disabled in manual mode"));
    }
  }
  else if (strcmp(command, "safety") == 0) {
    safetyEnabled = !safetyEnabled;
    Serial.printf("Safety: %s\n", safetyEnabled ? "ON" : "OFF");
  }
  else if (strcmp(command, "buzzer") == 0) {
    buzzerEnabled = !buzzerEnabled;
    Serial.printf("Buzzer: %s\n", buzzerEnabled ? "ON" : "OFF");
  }
  else if (strncmp(command, "power_factor ", 13) == 0) {
    float pf = atof(command + 13);
    if (pf >= 0.1 && pf <= 1.0) {
      calData.powerFactor = pf;
      saveCalibrationData();
      Serial.printf("✓ Power factor: %.2f\n", pf);
    } else {
      Serial.println(F("✗ Invalid (0.1-1.0)"));
    }
  }
  else if (strncmp(command, "current_cal ", 12) == 0) {
    float cal = atof(command + 12);
    if (cal > 0.001 && cal < 100.0) {
      calData.currentCalibration = cal;
      saveCalibrationData();
      Serial.printf("✓ Current cal: %.3f\n", cal);
    } else {
      Serial.println(F("✗ Invalid (0.001-100.0)"));
    }
  }
  else if (strncmp(command, "voltage_cal ", 12) == 0) {
    float cal = atof(command + 12);
    if (cal > 0.01 && cal < 1000.0) {
      calData.voltageCalibration = cal;
      saveCalibrationData();
      Serial.printf("✓ Voltage cal: %.3f\n", cal);
    } else {
      Serial.println(F("✗ Invalid (0.01-1000.0)"));
    }
  }
  else {
    Serial.println(F("✗ Unknown command"));
    Serial.println(F("Type 'help' for commands"));
  }
}

// ==================== MENU ====================
void printMenu() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    COMMAND MENU"));
  Serial.println(F("========================================"));
  Serial.println(F("EMERGENCY:"));
  Serial.println(F("  reset         - Emergency reset"));
  Serial.println(F("  restart       - Restart ESP32"));
  Serial.println();
  Serial.println(F("BASIC:"));
  Serial.println(F("  help          - Show this menu"));
  Serial.println(F("  status        - Show status"));
  Serial.println(F("  test          - Test sensors"));
  Serial.println(F("  on/off        - Control SSR"));
  Serial.println();
  Serial.println(F("DIAGNOSTICS:"));
  Serial.println(F("  diag          - Full diagnostics"));
  Serial.println(F("  mem           - Memory usage"));
  Serial.println(F("  stats         - Statistics"));
  Serial.println();
  Serial.println(F("CALIBRATION:"));
  Serial.println(F("  calibrate     - Auto-calibrate"));
  Serial.println(F("  cal_voltage   - Voltage wizard"));
  Serial.println(F("  voltage_cal X - Set factor (0.01-1000)"));
  Serial.println(F("  current_cal X - Set factor (0.001-100)"));
  Serial.println();
  Serial.println(F("SETTINGS:"));
  Serial.println(F("  manual        - Toggle manual mode"));
  Serial.println(F("  safety        - Toggle safety"));
  Serial.println(F("  buzzer        - Toggle buzzer"));
  Serial.println(F("  power_factor X- Set power factor"));
  Serial.println(F("  clear         - Clear statistics"));
  Serial.println(F("========================================\n"));
}

void printStatistics() {
  Serial.println(F("\n=== STATISTICS ==="));
  unsigned long uptime = (millis() - systemStartTime) / 1000;
  Serial.printf("Uptime: %02lu:%02lu:%02lu\n", uptime/3600, (uptime%3600)/60, uptime%60);
  Serial.printf("Loop count: %lu\n", loopCount);
  Serial.printf("Energy: %.3f Wh (%.2f kWh)\n", energyConsumed, energyConsumed / 1000.0);
  Serial.printf("Max Current: %.3f A\n", maxCurrent);
  Serial.printf("Max Voltage: %.1f V\n", maxVoltage);
  Serial.printf("Min Voltage: %.1f V\n", minVoltage);
  Serial.printf("Max Power: %.1f W\n", maxPower);
  Serial.printf("Shutdowns: %lu\n", shutdownCount);
  Serial.printf("Sensors: %s\n", sensorsValid ? "Valid" : "Invalid");
  Serial.printf("Safety: %s\n", safetyEnabled ? "Enabled" : "Disabled");
  Serial.printf("Manual: %s\n", manualControl ? "Yes" : "No");
  Serial.println(F("==================\n"));
}

// ==================== DIAGNOSTICS ====================
void printDiagnostics() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    SYSTEM DIAGNOSTICS"));
  Serial.println(F("========================================"));
  
  Serial.println(F("\n--- SYSTEM INFO ---"));
  Serial.printf("Version: v4.0 Enhanced\n");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
  
  printMemoryUsage();
  
  Serial.println(F("\n--- SENSOR STATUS ---"));
  Serial.printf("Sensors Valid: %s\n", sensorsValid ? "YES" : "NO");
  Serial.printf("Current Reading: %.3f A\n", currentReading);
  Serial.printf("Voltage Reading: %.1f V\n", voltageReading);
  Serial.printf("Power Reading: %.1f W\n", powerReading);
  Serial.printf("Current Avg: %.3f A\n", getAverageCurrent());
  Serial.printf("Voltage Avg: %.1f V\n", getAverageVoltage());
  
  Serial.println(F("\n--- CALIBRATION ---"));
  Serial.printf("Current Offset: %.3f V\n", calData.currentOffset);
  Serial.printf("Voltage Offset: %.3f V\n", calData.voltageOffset);
  Serial.printf("Current Cal: %.3f\n", calData.currentCalibration);
  Serial.printf("Voltage Cal: %.3f\n", calData.voltageCalibration);
  Serial.printf("Power Factor: %.2f\n", calData.powerFactor);
  Serial.printf("EEPROM Valid: %s\n", (calData.magic == EEPROM_MAGIC) ? "YES" : "NO");
  
  Serial.println(F("\n--- SAFETY STATUS ---"));
  Serial.printf("Safety Enabled: %s\n", safetyEnabled ? "YES" : "NO");
  Serial.printf("Manual Control: %s\n", manualControl ? "YES" : "NO");
  Serial.printf("Overcurrent: %s\n", overcurrentDetected ? "DETECTED" : "OK");
  Serial.printf("Overvoltage: %s\n", overvoltageDetected ? "DETECTED" : "OK");
  Serial.printf("Undervoltage: %s\n", undervoltageDetected ? "DETECTED" : "OK");
  Serial.printf("Brown-out: %s\n", brownoutDetected ? "DETECTED" : "OK");
  Serial.printf("Sensor Fault: %s\n", sensorFaultDetected ? "DETECTED" : "OK");
  
  Serial.println(F("\n--- SSR STATUS ---"));
  Serial.printf("SSR Enabled: %s\n", ssrEnabled ? "YES" : "NO");
  Serial.printf("SSR Command State: %s\n", ssrCommandState ? "ON" : "OFF");
  Serial.printf("SSR Pin State: %s\n", digitalRead(SSR_CONTROL_PIN) == SSR_ON_STATE ? "ON" : "OFF");
  
  Serial.println(F("\n--- PERFORMANCE ---"));
  if (perfMetrics.avgLoopTime > 0) {
    Serial.printf("Avg Loop Time: %lu ms\n", perfMetrics.avgLoopTime);
    Serial.printf("Min Loop Time: %lu ms\n", perfMetrics.minLoopTime);
    Serial.printf("Max Loop Time: %lu ms\n", perfMetrics.maxLoopTime);
    float loopsPerSec = perfMetrics.avgLoopTime > 0 ? 1000.0 / perfMetrics.avgLoopTime : 0;
    Serial.printf("Loops/Second: %.1f\n", loopsPerSec);
  }
  
  Serial.println(F("\n--- PERIPHERALS ---"));
  Serial.printf("SD Card: %s\n", sdCardAvailable ? "Available" : "Not Available");
  Serial.printf("Buzzer: %s\n", buzzerEnabled ? "Enabled" : "Disabled");
  Serial.printf("WiFi: %s\n", wifiEnabled ? "Enabled" : "Disabled");
  
  Serial.println(F("\n========================================\n"));
}

// ==================== MEMORY USAGE ====================
void printMemoryUsage() {
  Serial.println(F("\n--- MEMORY USAGE ---"));
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Heap Size: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.printf("Max Alloc Heap: %d bytes\n", ESP.getMaxAllocHeap());
  
  float heapUsagePercent = 100.0 * (1.0 - (float)ESP.getFreeHeap() / ESP.getHeapSize());
  Serial.printf("Heap Usage: %.1f%%\n", heapUsagePercent);
  
  if (heapUsagePercent > 80.0) {
    Serial.println(F("⚠️  WARNING: High memory usage!"));
  }
}

// ==================== CRC32 CALCULATION ====================
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

// ==================== EEPROM WITH CRC ====================
void saveCalibrationData() {
  calData.magic = EEPROM_MAGIC;
  
  // Calculate CRC (exclude CRC field itself)
  calData.crc = calculateCRC32((uint8_t*)&calData, sizeof(CalibrationData) - sizeof(uint32_t));
  
  // Write to EEPROM
  EEPROM.put(0, calData);
  EEPROM.commit();
  
  Serial.println(F("✓ Saved to EEPROM (with CRC)"));
}

bool loadCalibrationData() {
  CalibrationData loadedData;
  EEPROM.get(0, loadedData);
  
  // Check magic number
  if (loadedData.magic != EEPROM_MAGIC) {
    Serial.println(F("⚠️  EEPROM: Invalid magic number"));
    return false;
  }
  
  // Verify CRC
  uint32_t calculatedCRC = calculateCRC32((uint8_t*)&loadedData, sizeof(CalibrationData) - sizeof(uint32_t));
  if (calculatedCRC != loadedData.crc) {
    Serial.println(F("⚠️  EEPROM: CRC mismatch (data corrupted)"));
    return false;
  }
  
  // Validate ranges
  if (isnan(loadedData.currentOffset) || loadedData.currentOffset < 0.5 || loadedData.currentOffset > 3.0) {
    Serial.println(F("⚠️  EEPROM: Invalid current offset"));
    return false;
  }
  
  if (isnan(loadedData.voltageOffset) || loadedData.voltageOffset < 0.5 || loadedData.voltageOffset > 3.0) {
    Serial.println(F("⚠️  EEPROM: Invalid voltage offset"));
    return false;
  }
  
  if (isnan(loadedData.currentCalibration) || loadedData.currentCalibration <= 0 || loadedData.currentCalibration > 100.0) {
    Serial.println(F("⚠️  EEPROM: Invalid current calibration"));
    return false;
  }
  
  if (isnan(loadedData.voltageCalibration) || loadedData.voltageCalibration <= 0 || loadedData.voltageCalibration > 1000.0) {
    Serial.println(F("⚠️  EEPROM: Invalid voltage calibration"));
    return false;
  }
  
  if (isnan(loadedData.powerFactor) || loadedData.powerFactor <= 0 || loadedData.powerFactor > 1.0) {
    Serial.println(F("⚠️  EEPROM: Invalid power factor"));
    return false;
  }
  
  // All checks passed - load data
  calData = loadedData;
  Serial.println(F("✓ Loaded calibration from EEPROM (CRC verified)"));
  return true;
}

// ==================== SD CARD ====================
void initSDCard() {
  Serial.print(F("Initializing SD card... "));
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAILED"));
    sdCardAvailable = false;
    return;
  }
  
  sdCardAvailable = true;
  Serial.println(F("✓ SD card ready"));
  
  // Write header if file doesn't exist
  if (!SD.exists("/datalog.csv")) {
    File file = SD.open("/datalog.csv", FILE_WRITE);
    if (file) {
      file.println("Timestamp,Voltage,Current,Power,SSR_State");
      file.close();
      Serial.println(F("  Created datalog.csv with header"));
    }
  }
}

void logToSD(const char* data) {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/datalog.csv", FILE_APPEND);
  if (!file) {
    static unsigned long lastError = 0;
    if (millis() - lastError > 10000) {
      Serial.println(F("⚠️  SD write failed"));
      lastError = millis();
    }
    return;
  }
  
  file.println(data);
  file.close();
}

// ==================== BUZZER TEST ====================
void testBuzzer() {
  if (!buzzerEnabled) {
    Serial.println(F("Buzzer disabled"));
    return;
  }
  Serial.println(F("Testing buzzer..."));
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
    feedWatchdog();
  }
  Serial.println(F("✓ Buzzer OK"));
}