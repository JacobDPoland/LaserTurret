// Arduino IR Remote Servo Controller - Modified Version
// IR Controls Only: Power, Vol+/-, Skip Left/Right, Button 1 (sine wave)
// Now prints servo positions once per second

#include <Servo.h>
#include <IRremote.h>

// Define PI constant for sine wave calculations
#ifndef PI
#define PI 3.14159265359
#endif

// Pin definitions
const int SERVO1_PIN = 9;   // Servo-1 on pin 9
const int SERVO2_PIN = 10;  // Servo-2 on pin 10
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   // Servo 1 left by 5°
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  // Servo 1 right by 5°
const uint32_t IR_VOL_UP = 0xB946FF00;      // Servo 2 left by 5°
const uint32_t IR_VOL_DOWN = 0xEA15FF00;    // Servo 2 right by 5°
const uint32_t IR_POWER = 0xBA45FF00;       // Toggle servo enable/disable
const uint32_t IR_ONE = 0xF30CFF00;         // Triggers sine wave oscillation (was button 3)

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 92;
const int SERVO2_MIN_ANGLE = 0;  // Servo-2 minimum angle limit
const int SERVO2_MAX_ANGLE = 14; // Servo-2 maximum angle limit
// Calculate actual center angles for each servo
const int SERVO1_CENTER_ANGLE = (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2;  // 90° for 60-120 range
const int SERVO2_CENTER_ANGLE = (SERVO2_MIN_ANGLE + SERVO2_MAX_ANGLE) / 2; 
const int SERVO_CENTER_ANGLE = 90;  // Keep for compatibility
const int SERVO_STEP_SIZE = 5;

// Sine wave oscillation constants
const unsigned long SINE_UPDATE_INTERVAL = 20;
const float SINE_FREQUENCY = 0.25;
// Calculate amplitudes based on actual servo ranges
const int SINE1_AMPLITUDE = (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 2;        // 30° for 60-120 range
const int SINE2_AMPLITUDE = (SERVO2_MAX_ANGLE - SERVO2_MIN_ANGLE) / 2;      
const int SINE1_CENTER = SERVO1_CENTER_ANGLE;                               // 90°
const int SINE2_CENTER = SERVO2_CENTER_ANGLE;                              

// Position printing constants
const unsigned long POSITION_PRINT_INTERVAL = 1000; // 1 second

// Core servo control variables
Servo servo1;
Servo servo2;
int currentServo1Angle = SERVO1_CENTER_ANGLE;
int currentServo2Angle = SERVO2_CENTER_ANGLE;
bool servo1Attached = false;
bool servo2Attached = false;
bool servoEnabled = true;
bool sineOscillatingMode = false;

// Position printing variable
unsigned long lastPositionPrint = 0;

// Sine wave oscillation variables
unsigned long sineStartTime = 0;
unsigned long lastSineUpdate = 0;
float sineFrequency = SINE_FREQUENCY;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // Initialize both servos at their respective center positions
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1Attached = true;
  servo2Attached = true;
  servoEnabled = true;
  servo1.write(SERVO1_CENTER_ANGLE);
  servo2.write(SERVO2_CENTER_ANGLE);
  currentServo1Angle = SERVO1_CENTER_ANGLE;
  currentServo2Angle = SERVO2_CENTER_ANGLE;
  
  Serial.println(F("IR Servo Controller Ready"));
  startupLEDSequence();
  
  // Initialize position printing timer
  lastPositionPrint = millis();
}

void loop() {
  handleIRRemote();
  
  if (sineOscillatingMode && servoEnabled) {
    updateSineOscillation();
    handleIRRemote(); // Extra IR check during oscillation
  }
  
  enforceServoSafety();
  printServoPositions(); // Print servo positions once per second
}

void printServoPositions() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPositionPrint >= POSITION_PRINT_INTERVAL) {
    Serial.print(F("Servo1: "));
    Serial.print(currentServo1Angle);
    Serial.print(F("°, Servo2: "));
    Serial.print(currentServo2Angle);
    Serial.print(F("° ["));
    
    if (!servoEnabled) {
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

void updateSineOscillation() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSineUpdate >= SINE_UPDATE_INTERVAL) {
    float timeElapsed = (currentTime - sineStartTime) / 1000.0;
    float sineValue = sin(2 * PI * sineFrequency * timeElapsed);
    float cosineValue = cos(2 * PI * sineFrequency * timeElapsed);
    
    // Use separate amplitudes and centers for each servo
    int targetServo1Angle = SINE1_CENTER + (int)(sineValue * SINE1_AMPLITUDE);
    int targetServo2Angle = SINE2_CENTER + (int)(cosineValue * SINE2_AMPLITUDE);
    
    targetServo1Angle = constrain(targetServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    targetServo2Angle = constrain(targetServo2Angle, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
    
    if (abs(targetServo1Angle - currentServo1Angle) >= 1 || abs(targetServo2Angle - currentServo2Angle) >= 1) {
      currentServo1Angle = targetServo1Angle;
      currentServo2Angle = targetServo2Angle;
      
      if (servo1Attached && servo2Attached && servoEnabled) {
        servo1.write(currentServo1Angle);
        servo2.write(currentServo2Angle);
      }
    }
    
    lastSineUpdate = currentTime;
  }
}

void handleIRRemote() {
  if (IrReceiver.decode()) {
    uint32_t receivedCode = IrReceiver.decodedIRData.decodedRawData;
    
    // Improved repeat handling
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      static unsigned long lastRepeatTime = 0;
      if (millis() - lastRepeatTime < 150) {
        IrReceiver.resume();
        return;
      }
      lastRepeatTime = millis();
    }
    
    switch (receivedCode) {
      case IR_SKIP_LEFT:
        adjustServo1(SERVO_STEP_SIZE);
        break;
        
      case IR_SKIP_RIGHT:
        adjustServo1(-SERVO_STEP_SIZE);
        break;
        
      case IR_VOL_UP:
        adjustServo2(SERVO_STEP_SIZE);
        break;
        
      case IR_VOL_DOWN:
        adjustServo2(-SERVO_STEP_SIZE);
        break;
        
      case IR_POWER:
        toggleServo();
        break;
        
      case IR_ONE:
        toggleSineOscillation();
        break;
    }
    
    IrReceiver.resume();
  }
}

void adjustServo1(int degrees) {
  if (!servoEnabled) return;
  
  sineOscillatingMode = false; // Stop sine oscillation when manually adjusting
  
  int newAngle = currentServo1Angle + degrees;
  newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  if (newAngle != currentServo1Angle) {
    currentServo1Angle = newAngle;
    
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
    }
    
    servo1.write(currentServo1Angle);
    flashLED(1, 50);
  }
}

void adjustServo2(int degrees) {
  if (!servoEnabled) return;
  
  sineOscillatingMode = false; // Stop sine oscillation when manually adjusting
  
  int newAngle = currentServo2Angle + degrees;
  newAngle = constrain(newAngle, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
  if (newAngle != currentServo2Angle) {
    currentServo2Angle = newAngle;
    
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
    }
    
    servo2.write(currentServo2Angle);
    flashLED(1, 50);
  }
}

void toggleSineOscillation() {
  if (!servoEnabled) return;
  
  if (sineOscillatingMode) {
    sineOscillatingMode = false;
    flashLED(3, 300);
  } else {
    sineOscillatingMode = true;
    sineStartTime = millis();
    lastSineUpdate = millis();
    flashLED(4, 150);
  }
}

void toggleServo() {
  if (servoEnabled) {
    disableServo();
  } else {
    enableServo();
  }
}

void enableServo() {
  if (!servoEnabled) {
    servoEnabled = true;
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
    }
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
    }
    
    sineOscillatingMode = false; // Stop oscillation when enabling
    currentServo1Angle = SERVO1_CENTER_ANGLE;
    currentServo2Angle = SERVO2_CENTER_ANGLE;
    servo1.write(SERVO1_CENTER_ANGLE);
    servo2.write(SERVO2_CENTER_ANGLE);
    
    flashLED(3, 100);
  }
}

void disableServo() {
  if (servoEnabled) {
    servoEnabled = false;
    sineOscillatingMode = false; // Stop oscillation when disabling
    
    if (servo1Attached) {
      servo1.detach();
      servo1Attached = false;
    }
    if (servo2Attached) {
      servo2.detach();
      servo2Attached = false;
    }
    
    flashLED(5, 200);
  }
}

void enforceServoSafety() {
  if (!servoEnabled) return;
  
  currentServo1Angle = constrain(currentServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  currentServo2Angle = constrain(currentServo2Angle, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
  if ((!servo1Attached || !servo2Attached) && servoEnabled) {
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
    }
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
    }
    flashLED(3, 50);
  }
}

void startupLEDSequence() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void flashLED(int count, int delayTime) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayTime);
  }
}