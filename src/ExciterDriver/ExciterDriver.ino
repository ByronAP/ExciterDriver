#include <Arduino.h>
#include <EEPROM.h>

/*
 * 2025 ByronAP
 * ExciterDriver v2.9
 * Alternator Regulator Field Control System
 */

// =============================================================================
// == COMPILE-TIME CONFIGURATION
// =============================================================================
#define VERSION "2.9.0-dev"

#define DEBUG

// Macro-based debug printing for conditional compilation.
#ifdef DEBUG
  #define DBG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif


// =============================================================================
// == EEPROM FAULT LOGGING
// =============================================================================
// An enum to represent the types of faults. This is cleaner than using numbers.
enum FaultType {
  FAULT_OVER_VOLTAGE,
  FAULT_UNDER_VOLTAGE,
  FAULT_PROBE_TIMEOUT,
  FAULT_TACH_FAIL
};

// A struct to hold the entire fault log. This will be written to/read from EEPROM as a single block.
struct FaultLog {
  byte overVoltage;
  byte underVoltage;
  byte probeTimeout;
  byte tachFail;
};

// Value written into a fault log member to indicate the fault has occurred.
#define FAULT_LOGGED 1


// =============================================================================
// == PIN DEFINITIONS
// =============================================================================
const int vSensePin = A0;             // Analog input for battery voltage sensing
const int fieldControlPin = 3;        // PWM output for alternator field coil MOSFET
const int statusLedPin = 13;          // Status LED (built-in)
const int tachPin = 6;                // Tachometer signal input
const int fieldLedPin = 10;           // PWM output for field duty cycle indicator LED
const int errorLedPin = 11;           // Digital output for error indicator LED
const int diagButtonPin = A3;         // Input for diagnostic mode button (connect to GND)


// =============================================================================
// == VOLTAGE CALIBRATION & REGULATION
// =============================================================================
const float CAL_LOW_VOLTS = 12.001;
const int   CAL_LOW_ADC   = 742;
const float CAL_HIGH_VOLTS = 14.499;
const int   CAL_HIGH_ADC   = 854;
const float CAL_VOLT_RANGE = CAL_HIGH_VOLTS - CAL_LOW_VOLTS;
const float CAL_ADC_RANGE = (float)(CAL_HIGH_ADC - CAL_LOW_ADC);
const float targetVoltage = 14.2;
const float deadband = 0.1;
const float maxVoltage = 16.0;
const float overVoltageThreshold = 15.5;
const float underVoltageErrorThreshold = 12.8;


// =============================================================================
// == ENGINE STATE & CONTROL PARAMETERS
// =============================================================================
const float engineOffThreshold = 12.5;
const float runningThreshold = 13.0;
const float stopRunningThreshold = 12.0;
const float probeDutyCyclePercentage = 5.0;
const unsigned long softStartDuration = 1500;
const unsigned long lowVoltageShutdownDelay = 3000;
const int softStartEndDutyCycle = 40;
const int pulsesPerRevolution = 2;
const unsigned long tachTimeoutMicros = 1000000;
const unsigned long tachDebounceMicros = 500;
const int TACH_PULSES_TO_CONFIRM = 5;
const unsigned long TACH_QUALIFICATION_TIMEOUT = 2000;
const unsigned long MAX_PULSE_INTERVAL_MICROS = 300000;


// =============================================================================
// == TIMING & PID CONSTANTS
// =============================================================================
const int loopInterval = 10;
const int voltageWindowSize = 50;
const unsigned long diagnosticInterval = 1000;
const unsigned long stateTransitionDebounce = 100;
const int adcSamplesToAverage = 4;
const int adcSampleDelayMicros = 250;
const float pGain = 10.0;
const float iGain = 0.05;
const float dGain = 2.0;
const float maxIntegralError = 25.0;
const int maxDutyCycleChangePerLoop = 5;


// =============================================================================
// == GLOBAL VARIABLES
// =============================================================================
enum EngineState { OFF, CRANKING, PROBING, SOFT_START, RUNNING };
EngineState engineState = OFF;
EngineState lastEngineState = OFF;
unsigned long stateEntryTime = 0;
float voltageReadings[voltageWindowSize];
int readingIndex = 0;
float voltageTotal = 0.0;
float lastVoltage = 0.0;
float voltageRate = 0.0;
int dutyCycle = 0;
float lastError = 0.0;
float integralError = 0.0;
bool pidResetRequest = false;
bool initialized = false;
unsigned long probingStartTime = 0;
unsigned long lowVoltageTimerStart = 0;
volatile unsigned long lastTachPulseTime = 0;
volatile unsigned long tachPulseInterval = 0;
volatile int consecutiveTachPulses = 0;
bool engineIsSpinning = false;
bool tachSignalConfirmed = false;
unsigned long qualificationStartTime = 0;
bool probingTimedOut = false;
bool tachSignalLostFault = false;
const int probeDutyCycle = (probeDutyCyclePercentage * 255) / 100;


// =============================================================================
// == FUNCTION PROTOTYPES
// =============================================================================
void enterDiagnosticMode();
void blinkCode(int code);
void logFault(FaultType fault);
float readVoltage();
void updateMovingAverage(float voltage);
float getAverageVoltage();
void transitionToState(EngineState newState);
void updateEngineState(float averageVoltage, unsigned long currentTime);
void updatePwmDutyCycle(float averageVoltage);
void checkParameters();
void blinkStatusLed(int times);
void printDiagnostics(float voltage, float averageVoltage);
void tachIsr();
void updateTachStatus();
void updateHardwareOutputs(float averageVoltage);


// =============================================================================
// == SETUP
// =============================================================================
void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  DBG_PRINTLN(F("\n\n---"));
  DBG_PRINT(F("ExciterDriver v"));
  DBG_PRINTLN(VERSION);

  pinMode(fieldControlPin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);
  pinMode(fieldLedPin, OUTPUT);
  pinMode(errorLedPin, OUTPUT);
  pinMode(tachPin, INPUT_PULLUP);
  pinMode(diagButtonPin, INPUT_PULLUP);

  delay(50);
  if (digitalRead(diagButtonPin) == LOW) {
    enterDiagnosticMode();
  }

  DBG_PRINTLN(F("Normal startup. Clearing fault logs..."));
  FaultLog emptyLog = {0};
  EEPROM.put(0, emptyLog);

  analogWrite(fieldControlPin, 0);
  digitalWrite(statusLedPin, LOW);
  analogWrite(fieldLedPin, 0);
  digitalWrite(errorLedPin, LOW);

  checkParameters();

  float initialVoltage = 0.0;
  for (int i = 0; i < 10; i++) {
    initialVoltage += readVoltage();
    delay(10);
  }
  initialVoltage /= 10;

  for (int i = 0; i < voltageWindowSize; i++) {
    voltageReadings[i] = initialVoltage;
  }
  voltageTotal = initialVoltage * voltageWindowSize;
  lastVoltage = initialVoltage;

  attachInterrupt(digitalPinToInterrupt(tachPin), tachIsr, RISING);

  if (initialVoltage >= runningThreshold) {
    engineState = SOFT_START;
    DBG_PRINTLN(F("Detected running engine at startup"));
  } else if (initialVoltage >= engineOffThreshold) {
    engineState = PROBING;
    DBG_PRINTLN(F("Detected voltage above threshold at startup"));
  }
  
  stateEntryTime = millis();
  blinkStatusLed(engineState + 1);
  initialized = true;
  // System is now fully operational and entering the main loop.
  DBG_PRINTLN(F("Initialization complete."));
}


// =============================================================================
// == MAIN LOOP
// =============================================================================
void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  updateTachStatus();
  if (tachSignalConfirmed) {
      unsigned long currentMicros = micros();
      noInterrupts();
      unsigned long lastPulseTime = lastTachPulseTime;
      interrupts();
      engineIsSpinning = ((uint32_t)(currentMicros - lastPulseTime) < tachTimeoutMicros);
  }

  if ((unsigned long)(currentTime - lastLoopTime) >= loopInterval) {
    lastLoopTime = currentTime;
    
    if (initialized) {
      float voltage = readVoltage();
      updateMovingAverage(voltage);
      float averageVoltage = getAverageVoltage();
      
      updateEngineState(averageVoltage, currentTime);
      updatePwmDutyCycle(averageVoltage);
      updateHardwareOutputs(averageVoltage);
      
      lastVoltage = voltage;
      
      static unsigned long lastDebugTime = 0;
      if ((unsigned long)(currentTime - lastDebugTime) >= diagnosticInterval) {
        printDiagnostics(voltage, averageVoltage);
        lastDebugTime = currentTime;
      }
    }
  }
}


// =============================================================================
// == DIAGNOSTIC & FAULT LOGGING FUNCTIONS
// =============================================================================

/**
 * Enters a permanent loop to read and blink fault codes from EEPROM.
 */
void enterDiagnosticMode() {
  DBG_PRINTLN(F("Entering Diagnostic Mode..."));
  while (true) {
    FaultLog storedLog;
    EEPROM.get(0, storedLog);
    bool faultsFound = false;

    DBG_PRINTLN(F("Reading fault codes from previous run:"));

    if (storedLog.overVoltage == FAULT_LOGGED) {
      faultsFound = true;
      DBG_PRINTLN(F("  - Fault found: Over-Voltage (Code 1)"));
      blinkCode(1);
      delay(2000);
    }
    if (storedLog.underVoltage == FAULT_LOGGED) {
      faultsFound = true;
      DBG_PRINTLN(F("  - Fault found: Under-Voltage (Code 2)"));
      blinkCode(2);
      delay(2000);
    }
    if (storedLog.probeTimeout == FAULT_LOGGED) {
      faultsFound = true;
      DBG_PRINTLN(F("  - Fault found: Probing Timeout (Code 3)"));
      blinkCode(3);
      delay(2000);
    }
    if (storedLog.tachFail == FAULT_LOGGED) {
      faultsFound = true;
      DBG_PRINTLN(F("  - Fault found: Tachometer Fail (Code 4)"));
      blinkCode(4);
      delay(2000);
    }

    if (!faultsFound) {
      DBG_PRINTLN(F("  - No faults found."));
      digitalWrite(errorLedPin, HIGH);
      delay(2000);
      digitalWrite(errorLedPin, LOW);
    }
    delay(5000);
  }
}

/**
 * Blinks the error LED a specific number of times to indicate a code.
 */
void blinkCode(int code) {
  for (int i = 0; i < code; i++) {
    digitalWrite(errorLedPin, HIGH);
    delay(250);
    digitalWrite(errorLedPin, LOW);
    delay(400);
  }
}

/**
 * Logs a specific fault type to the EEPROM struct.
 */
void logFault(FaultType fault) {
  FaultLog currentLog;
  EEPROM.get(0, currentLog);
  bool needsUpdate = false;

  switch (fault) {
    case FAULT_OVER_VOLTAGE:
      if (currentLog.overVoltage != FAULT_LOGGED) {
        currentLog.overVoltage = FAULT_LOGGED;
        needsUpdate = true;
      }
      break;
    case FAULT_UNDER_VOLTAGE:
      if (currentLog.underVoltage != FAULT_LOGGED) {
        currentLog.underVoltage = FAULT_LOGGED;
        needsUpdate = true;
      }
      break;
    case FAULT_PROBE_TIMEOUT:
      if (currentLog.probeTimeout != FAULT_LOGGED) {
        currentLog.probeTimeout = FAULT_LOGGED;
        needsUpdate = true;
      }
      break;
    case FAULT_TACH_FAIL:
      if (currentLog.tachFail != FAULT_LOGGED) {
        currentLog.tachFail = FAULT_LOGGED;
        needsUpdate = true;
      }
      break;
  }

  if (needsUpdate) {
    EEPROM.put(0, currentLog);
    DBG_PRINT(F("!!! FAULT LOGGED: "));
    DBG_PRINTLN(fault);
  }
}


// =============================================================================
// == CORE FUNCTIONS
// =============================================================================

float readVoltage() {
  int adcTotal = 0;
  for (int i = 0; i < adcSamplesToAverage; i++) {
    adcTotal += analogRead(vSensePin);
    delayMicroseconds(adcSampleDelayMicros);
  }
  int adcValue = adcTotal / adcSamplesToAverage;
  float voltage = CAL_LOW_VOLTS + (((float)adcValue - CAL_LOW_ADC) * CAL_VOLT_RANGE / CAL_ADC_RANGE);
  if (voltage < 1.0 || voltage > maxVoltage + 2.0) {
    voltage = lastVoltage;
  }
  voltageRate = (voltage - lastVoltage);
  return voltage;
}

void updateEngineState(float averageVoltage, unsigned long currentTime) {
  switch (engineState) {
    case OFF:
      if (tachSignalConfirmed && engineIsSpinning) transitionToState(SOFT_START);
      else if (averageVoltage > engineOffThreshold) transitionToState(PROBING);
      break;
    case CRANKING: break;
    case PROBING:
      if ((tachSignalConfirmed && engineIsSpinning) || (averageVoltage > runningThreshold)) {
        transitionToState(SOFT_START);
      } else if (averageVoltage < engineOffThreshold) {
        transitionToState(OFF);
      } else if ((unsigned long)(currentTime - probingStartTime) > 5000) {
        probingTimedOut = true;
        logFault(FAULT_PROBE_TIMEOUT);
        transitionToState(OFF);
      }
      break;
    case SOFT_START:
      if ((tachSignalConfirmed && !engineIsSpinning) || (averageVoltage < stopRunningThreshold)) {
          transitionToState(OFF);
          break;
      }
      if ((unsigned long)(currentTime - stateEntryTime) >= softStartDuration) {
          transitionToState(RUNNING);
      }
      break;
    case RUNNING:
      if (tachSignalConfirmed && !engineIsSpinning && averageVoltage >= runningThreshold) {
        DBG_PRINTLN(F("! TACH FAIL: Reverting to voltage-only mode."));
        tachSignalLostFault = true;
        logFault(FAULT_TACH_FAIL);
        tachSignalConfirmed = false;
        noInterrupts();
        consecutiveTachPulses = 0;
        interrupts();
        qualificationStartTime = 0;
        break;
      }
      bool potentialShutdown = (tachSignalConfirmed && !engineIsSpinning) || (averageVoltage < stopRunningThreshold);
      if (potentialShutdown) {
        if (lowVoltageTimerStart == 0) lowVoltageTimerStart = currentTime;
        if ((unsigned long)(currentTime - lowVoltageTimerStart) >= lowVoltageShutdownDelay) {
          DBG_PRINTLN(F("Engine stopped (condition persisted)."));
          transitionToState(OFF);
        }
      } else {
        lowVoltageTimerStart = 0;
      }
      break;
  }
}

void updatePwmDutyCycle(float averageVoltage) {
  if (engineState == OFF || engineState == CRANKING) {
    dutyCycle = 0;
    integralError = 0;
  } else if (engineState == PROBING) {
    dutyCycle = probeDutyCycle;
    integralError = 0;
  } else if (engineState == SOFT_START) {
    unsigned long timeInState = millis() - stateEntryTime;
    float rampProgress = (float)timeInState / softStartDuration;
    int targetDuty = (softStartEndDutyCycle / 100.0) * 255;
    dutyCycle = constrain(rampProgress * targetDuty, 0, 255);
    integralError = 0;
  } else if (engineState == RUNNING) {
    float error = targetVoltage - averageVoltage;
    if (pidResetRequest) {
      lastError = error;
      integralError = 0;
      pidResetRequest = false;
      DBG_PRINTLN(F("PID history reset for smooth engagement."));
    }
    if (abs(error) >= deadband / 2) {
      integralError += error * (loopInterval / 1000.0);
      integralError = constrain(integralError, -maxIntegralError, maxIntegralError);
      float derivative = (error - lastError) / (loopInterval / 1000.0);
      int adjustment = round(pGain * error + iGain * integralError + dGain * derivative);
      dutyCycle += constrain(adjustment, -maxDutyCycleChangePerLoop, maxDutyCycleChangePerLoop);
      dutyCycle = constrain(dutyCycle, 0, 255);
    }
    lastError = error;
  }
}


// =============================================================================
// == UTILITY FUNCTIONS
// =============================================================================

void updateHardwareOutputs(float averageVoltage) {
  analogWrite(fieldControlPin, dutyCycle);
  analogWrite(fieldLedPin, dutyCycle);
  bool errorState = false;
  if (averageVoltage > overVoltageThreshold) {
    errorState = true;
    logFault(FAULT_OVER_VOLTAGE);
  }
  if (engineState == RUNNING && averageVoltage < underVoltageErrorThreshold) {
    errorState = true;
    logFault(FAULT_UNDER_VOLTAGE);
  }
  if (probingTimedOut || tachSignalLostFault) {
    errorState = true;
  }
  digitalWrite(errorLedPin, errorState);
}

void transitionToState(EngineState newState) {
  if (newState != engineState) {
    static unsigned long lastTransitionTime = 0;
    unsigned long currentTime = millis();
    if ((unsigned long)(currentTime - lastTransitionTime) > stateTransitionDebounce) {
      lastEngineState = engineState;
      engineState = newState;
      stateEntryTime = currentTime;
      lastTransitionTime = currentTime;
      if (newState == PROBING) {
        probingStartTime = currentTime;
        probingTimedOut = false;
      }
      if (newState == RUNNING && lastEngineState == SOFT_START) {
        pidResetRequest = true;
      }
      if (newState == OFF) {
        if (tachSignalLostFault) tachSignalLostFault = false;
      }
      DBG_PRINT(F("State transition: "));
      DBG_PRINT(lastEngineState);
      DBG_PRINT(F(" -> "));
      DBG_PRINTLN(engineState);
      blinkStatusLed(engineState + 1);
    }
  }
}

// =============================================================================
// == INTERRUPT SERVICE ROUTINE & TACHOMETER LOGIC
// =============================================================================

void tachIsr() {
  unsigned long now = micros();
  if ((unsigned long)(now - lastTachPulseTime) < tachDebounceMicros) return;
  unsigned long interval = now - lastTachPulseTime;
  lastTachPulseTime = now;
  if (tachSignalConfirmed) {
    tachPulseInterval = interval;
    return;
  }
  if (interval > MAX_PULSE_INTERVAL_MICROS || consecutiveTachPulses == 0) {
    consecutiveTachPulses = 1;
  } else {
    consecutiveTachPulses++;
  }
}

void updateTachStatus() {
  if (tachSignalConfirmed) return;
  noInterrupts();
  int pulseCount = consecutiveTachPulses;
  interrupts();
  if (pulseCount > 0 && qualificationStartTime == 0) {
    qualificationStartTime = millis();
  }
  if (pulseCount >= TACH_PULSES_TO_CONFIRM) {
    tachSignalConfirmed = true;
    engineIsSpinning = true;
    DBG_PRINTLN(F(">>> Tach signal CONFIRMED <<<"));
    if (tachSignalLostFault) {
      DBG_PRINTLN(F("Tach fault condition cleared."));
      tachSignalLostFault = false;
    }
    qualificationStartTime = 0;
  }
  if (qualificationStartTime > 0 && (unsigned long)(millis() - qualificationStartTime) > TACH_QUALIFICATION_TIMEOUT) {
    DBG_PRINTLN(F("Tach qualification timed out, resetting."));
    noInterrupts();
    consecutiveTachPulses = 0;
    interrupts();
    qualificationStartTime = 0;
  }
}


// =============================================================================
// == OTHER HELPER FUNCTIONS
// =============================================================================
void updateMovingAverage(float voltage) {
  voltageTotal -= voltageReadings[readingIndex];
  voltageReadings[readingIndex] = voltage;
  voltageTotal += voltage;
  readingIndex = (readingIndex + 1) % voltageWindowSize;
}

float getAverageVoltage() {
  return voltageTotal / voltageWindowSize;
}

void blinkStatusLed(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(statusLedPin, HIGH);
    delay(100);
    digitalWrite(statusLedPin, LOW);
    delay(100);
  }
}

void checkParameters() {
  bool paramsValid = true;
  if (stopRunningThreshold >= engineOffThreshold || engineOffThreshold >= runningThreshold) {
    DBG_PRINTLN(F("Warning: Voltage thresholds not in order"));
    paramsValid = false;
  }
  if (!paramsValid) {
    DBG_PRINTLN(F("Some configuration parameters are invalid!"));
    blinkStatusLed(5);
  }
}

void printDiagnostics(float voltage, float averageVoltage) {
#ifdef DEBUG
  Serial.print(F("State: "));
  switch(engineState) {
    case OFF: Serial.print(F("OFF")); break;
    case CRANKING: Serial.print(F("CRANKING")); break;
    case PROBING: Serial.print(F("PROBING")); break;
    case SOFT_START: Serial.print(F("SOFT_START")); break;
    case RUNNING: Serial.print(F("RUNNING")); break;
  }
  
  Serial.print(F(", Tach: "));
  if (tachSignalConfirmed) {
    if (engineIsSpinning) {
      noInterrupts();
      unsigned long interval = tachPulseInterval;
      interrupts();
      unsigned long rpm = (interval > 0) ? (60000000UL / (interval * pulsesPerRevolution)) : 0;
      Serial.print(rpm);
      Serial.print(F(" RPM"));
    } else {
      Serial.print(F("Stopped"));
    }
  } else {
    noInterrupts();
    int pulseCount = consecutiveTachPulses;
    interrupts();
    Serial.print(F("Qualifying("));
    Serial.print(pulseCount);
    Serial.print(F(")"));
  }

  Serial.print(F(", V: "));
  Serial.print(averageVoltage, 2);
  Serial.print(F("V, T: "));
  Serial.print(targetVoltage, 2);
  Serial.print(F("V, Duty: "));
  Serial.print((dutyCycle * 100) / 255);
  Serial.print(F("%, I-Term: "));
  Serial.println(integralError, 2);
#endif
}