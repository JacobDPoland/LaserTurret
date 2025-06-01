#include <Servo.h>
#include <IRremote.h>

// Define PI constant for sine wave calculations
#ifndef PI
#define PI 3.14159265359
#endif

// Pin definitions
const int SERVOX_PIN = 9;   // Servo-X on pin 9
const int SERVOY_PIN = 10;  // Servo-Y on pin 10
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   // Servo x left
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  // Servo x right
const uint32_t IR_VOL_UP = 0xB946FF00;      // Servo y up (left)
const uint32_t IR_VOL_DOWN = 0xEA15FF00;    // Servo y down (right)
const uint32_t IR_POWER = 0xBA45FF00;       // Toggle servo enable/disable
const uint32_t IR_ONE = 0xF30CFF00;         // Triggers circle pathh
const uint32_t IR_TWO = 0xE718FF00;         // Triggers randomized sine frequency mode
const uint32_t IR_THREE = 0xA15EFF00;
const uint32_t IR_FOUR = 0xF708FF00;
const uint32_t IR_FIVE = 0xE31CFF00;
const uint32_t IR_SIX = 0xA55AFF00;
const uint32_t IR_SEVEN = 0xBD42FF00;
const uint32_t IR_EIGHT = 0xAD52FF00;
const uint32_t IR_NINE = 0xB54AFF00;
const uint32_t IR_ZERO = 0xE916FF00;


const int SERVOX_MIN_ANGLE = 30;  // 30
const int SERVOX_MAX_ANGLE = 105;  // 105
const int SERVOY_MIN_ANGLE = 0;  
const int SERVOY_MAX_ANGLE = 15; 

const int SERVOX_CENTER_ANGLE = (SERVOX_MIN_ANGLE + SERVOX_MAX_ANGLE) / 2;  
const int SERVOY_CENTER_ANGLE = (SERVOY_MIN_ANGLE + SERVOY_MAX_ANGLE) / 2; 
const int SERVO_STEP_SIZE = 5;

// Sine wave oscillation constants
const unsigned long SINE_UPDATE_INTERVAL = 20;
const int SINEX_AMPLITUDE = (SERVOX_MAX_ANGLE - SERVOX_MIN_ANGLE) / 2;       
const int SINEY_AMPLITUDE = (SERVOY_MAX_ANGLE - SERVOY_MIN_ANGLE) / 2;  

const unsigned long POSITION_PRINT_INTERVAL = 1000;  // in miliseconds

// Core servo control variables
Servo servoX;
Servo servoY;
int currentServoXAngle = SERVOX_CENTER_ANGLE;
int currentServoYAngle = SERVOY_CENTER_ANGLE;
bool servosEnabled = false;
bool sineOscillatingMode = false;
float sineFrequency = 0.25;
unsigned long sineStartTime = 0;
unsigned long lastSineUpdate = 0;

unsigned long lastPositionPrint = 0;


void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  servoX.attach(SERVOX_PIN);
  servoY.attach(SERVOY_PIN);
  servosEnabled = true;

  servoX.write(SERVOX_CENTER_ANGLE);
  servoY.write(SERVOY_CENTER_ANGLE);
  currentServoXAngle = SERVOX_CENTER_ANGLE;
  currentServoYAngle = SERVOY_CENTER_ANGLE;

  Serial.println(F("IR Servo Controller Ready"));  // F function stores the string in flash memory

  lastPositionPrint = millis();
}

void loop() {
  handleIRRemote();

  if (sineOscillatingMode && servosEnabled) {
    updateSineOscillation();
  }

  enforceServoSafety();
  printServoPositions();
}

void handleIRRemote() {
  if (IrReceiver.decode()) {
    uint32_t receivedCode = IrReceiver.decodedIRData.decodedRawData;
    Serial.print("IR code: ");
    Serial.println(receivedCode, HEX);
    
    switch (receivedCode) {
      case IR_SKIP_LEFT:
        adjustServoX(SERVO_STEP_SIZE);
        break;
        
      case IR_SKIP_RIGHT:
        adjustServoX(-SERVO_STEP_SIZE);
        break;
        
      case IR_VOL_UP:
        adjustServoY(SERVO_STEP_SIZE);
        break;
        
      case IR_VOL_DOWN:
        adjustServoY(-SERVO_STEP_SIZE);
        break;
        
      case IR_POWER:
        toggleServos();
        break;
        
      case IR_ONE:
        toggleSineOscillation();
        break;
    }
    
    IrReceiver.resume();
  }
}

void enforceServoSafety() {
  if (!servosEnabled) return;
  
  currentServoXAngle = constrain(currentServoXAngle, SERVOX_MIN_ANGLE, SERVOX_MAX_ANGLE);
  currentServoYAngle = constrain(currentServoYAngle, SERVOY_MIN_ANGLE, SERVOY_MAX_ANGLE);
}

void updateSineOscillation() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSineUpdate >= SINE_UPDATE_INTERVAL) {
    float timeElapsed = (currentTime - sineStartTime) / 1000.0;
    float sineValue = sin(2 * PI * sineFrequency * timeElapsed);
    float cosineValue = cos(2 * PI * sineFrequency * timeElapsed);
    
    // Use separate amplitudes and centers for each servo
    int targetServoXAngle = SERVOX_CENTER_ANGLE + (int)(sineValue * SINEX_AMPLITUDE);
    int targetServoYAngle = SERVOY_CENTER_ANGLE + (int)(cosineValue * SINEY_AMPLITUDE);
    
    targetServoXAngle = constrain(targetServoXAngle, SERVOX_MIN_ANGLE, SERVOX_MAX_ANGLE);
    targetServoYAngle = constrain(targetServoYAngle, SERVOY_MIN_ANGLE, SERVOY_MAX_ANGLE);
    servoX.write(targetServoXAngle);
    servoY.write(targetServoYAngle);
    
    lastSineUpdate = currentTime;
  }
}

void enableServos() {
  if (!servosEnabled) {
    servosEnabled = true;
    
    sineOscillatingMode = false; // Stop oscillation when enabling
    currentServoXAngle = SERVOX_CENTER_ANGLE;
    currentServoYAngle = SERVOY_CENTER_ANGLE;
    servoX.write(SERVOX_CENTER_ANGLE);
    servoY.write(SERVOY_CENTER_ANGLE);

  }
}

void disableServos() {
  if (servosEnabled) {
    servosEnabled = false;
    sineOscillatingMode = false;
  }
}

void toggleServos() {
  if (servosEnabled) {
    disableServos();
  } else {
    enableServos();
  }
}

void printServoPositions() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPositionPrint >= POSITION_PRINT_INTERVAL) {
    Serial.print(F("ServoX: "));
    Serial.print(currentServoXAngle);
    Serial.print(F("°, ServoY: "));
    Serial.print(currentServoYAngle);
    Serial.print(F("° ["));
    
    if (!servosEnabled) {
      Serial.print(F("DISABLED"));
    } else if (sineOscillatingMode) {
      Serial.print(F("SINE OSC"));
    } else {
      Serial.print(F("MANUAL"));
    }
    
    Serial.println(F("]"));
    
    lastPositionPrint = currentTime;
  }
}

void adjustServoX(int degrees) {
  if (!servosEnabled) return;
  
  sineOscillatingMode = false; // Stop sine oscillation when manually adjusting
  
  int newAngle = currentServoXAngle + degrees;
  newAngle = constrain(newAngle, SERVOX_MIN_ANGLE, SERVOX_MAX_ANGLE);
  
  if (newAngle != currentServoXAngle) {
    currentServoXAngle = newAngle;
    servoX.write(currentServoXAngle);
  }
}

void adjustServoY(int degrees) {
  if (!servosEnabled) return;
  
  sineOscillatingMode = false; // Stop sine oscillation when manually adjusting
  
  int newAngle = currentServoYAngle + degrees;
  newAngle = constrain(newAngle, SERVOY_MIN_ANGLE, SERVOY_MAX_ANGLE);
  
  if (newAngle != currentServoYAngle) {
    currentServoYAngle = newAngle;
    servoY.write(currentServoYAngle);
  }
}


void toggleSineOscillation() {
  if (!servosEnabled) return;
  
  if (sineOscillatingMode) {
    sineOscillatingMode = false;
  } else {
    sineOscillatingMode = true;
    sineStartTime = millis();
    lastSineUpdate = millis();
  }
}
