// Arduino IR Remote Servo Controller - Cleaned Version
// IR Controls Only: Power, Vol+/-, Skip Left/Right, Buttons 1-3

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
const uint32_t IR_ONE = 0xF30CFF00;         // Triggers snap oscillation 0° <-> 180°
const uint32_t IR_TWO = 0xE718FF00;         // Triggers slow oscillation at half speed
const uint32_t IR_THREE = 0xA15EFF00;       // Triggers sine wave oscillation

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO2_MAX_ANGLE = 125; // Servo-2 maximum angle limit
const int SERVO_CENTER_ANGLE = 90;
const int SERVO_STEP_SIZE = 5;

// Oscillation timing constants
const unsigned long SERVO_SETTLE_TIME = 450;
const unsigned long SERVO_SLOW_SETTLE_TIME = 927;

// Slow oscillation step control
const int SLOW_STEP_SIZE = 2;
const int SLOW_STEP_DELAY = 10;

// Sine wave oscillation constants
const unsigned long SINE_UPDATE_INTERVAL = 20;
const float SINE_FREQUENCY = 0.5;
const int SINE_AMPLITUDE = 90;
const int SINE_CENTER = 90;

// Core servo control variables
Servo servo1;
Servo servo2;
int currentServo1Angle = SERVO_CENTER_ANGLE;
int currentServo2Angle = SERVO_CENTER_ANGLE;
unsigned long lastServoCommand = 0;
bool servoMoving = false;
bool servo1Attached = false;
bool servo2Attached = false;
bool servoEnabled = true;
bool snapOscillatingMode = false;
bool slowOscillatingMode = false;
bool sineOscillatingMode = false;
bool snapPosition = true;

// Slow oscillation variables
int slowTargetServo1Angle = SERVO_CENTER_ANGLE;
int slowTargetServo2Angle = SERVO_CENTER_ANGLE;
unsigned long lastSlowStepTime = 0;
unsigned long lastDirChange = 0;

// Sine wave oscillation variables
unsigned long sineStartTime = 0;
unsigned long lastSineUpdate = 0;
float sineFrequency = SINE_FREQUENCY;
int sineAmplitude = SINE_AMPLITUDE;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // Initialize both servos at center position
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1Attached = true;
  servo2Attached = true;
  servoEnabled = true;
  servo1.write(SERVO_CENTER_ANGLE);
  servo2.write(SERVO_CENTER_ANGLE);
  currentServo1Angle = SERVO_CENTER_ANGLE;
  currentServo2Angle = SERVO_CENTER_ANGLE;
  
  Serial.println(F("IR Servo Controller Ready"));
  startupLEDSequence();
}

void loop() {
  handleIRRemote();
  
  if ((snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) && servoEnabled) {
    if (snapOscillatingMode) {
      updateSnapOscillation();
    } else if (slowOscillatingMode) {
      updateSlowOscillation();
    } else if (sineOscillatingMode) {
      updateSineOscillation();
    }
    handleIRRemote(); // Extra IR check during oscillation
  }
  
  enforceServoSafety();
}

void updateSineOscillation() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSineUpdate >= SINE_UPDATE_INTERVAL) {
    float timeElapsed = (currentTime - sineStartTime) / 1000.0;
    float sineValue = sin(2 * PI * sineFrequency * timeElapsed);
    
    int targetServo1Angle = SINE_CENTER + (int)(sineValue * sineAmplitude);
    int targetServo2Angle = SINE_CENTER + (int)(sineValue * sineAmplitude);
    
    targetServo1Angle = constrain(targetServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    targetServo2Angle = constrain(targetServo2Angle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
    
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

void updateSnapOscillation() {
  if (!servoMoving || (millis() - lastServoCommand >= SERVO_SETTLE_TIME)) {
    if (snapPosition) {
      currentServo1Angle = SERVO_MIN_ANGLE;
      currentServo2Angle = SERVO_MIN_ANGLE;
      snapPosition = false;
    } else {
      currentServo1Angle = SERVO_MAX_ANGLE;
      currentServo2Angle = SERVO2_MAX_ANGLE;
      snapPosition = true;
    }
    
    if (servo1Attached && servo2Attached && servoEnabled) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      servoMoving = true;
      lastServoCommand = millis();
      flashLED(1, 25);
    }
  }
}

void updateSlowOscillation() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSlowStepTime >= SLOW_STEP_DELAY) {
    if (currentServo1Angle == slowTargetServo1Angle && currentServo2Angle == slowTargetServo2Angle) {
      lastDirChange = currentTime;
      
      if (slowTargetServo1Angle == SERVO_MAX_ANGLE) {
        slowTargetServo1Angle = SERVO_MIN_ANGLE;
        slowTargetServo2Angle = SERVO_MIN_ANGLE;
        snapPosition = false;
      } else {
        slowTargetServo1Angle = SERVO_MAX_ANGLE;
        slowTargetServo2Angle = SERVO2_MAX_ANGLE;
        snapPosition = true;
      }
    }
    
    // Move one step closer to targets
    if (currentServo1Angle < slowTargetServo1Angle) {
      currentServo1Angle += SLOW_STEP_SIZE;
      if (currentServo1Angle > slowTargetServo1Angle) {
        currentServo1Angle = slowTargetServo1Angle;
      }
    } else if (currentServo1Angle > slowTargetServo1Angle) {
      currentServo1Angle -= SLOW_STEP_SIZE;
      if (currentServo1Angle < slowTargetServo1Angle) {
        currentServo1Angle = slowTargetServo1Angle;
      }
    }
    
    if (currentServo2Angle < slowTargetServo2Angle) {
      currentServo2Angle += SLOW_STEP_SIZE;
      if (currentServo2Angle > slowTargetServo2Angle) {
        currentServo2Angle = slowTargetServo2Angle;
      }
    } else if (currentServo2Angle > slowTargetServo2Angle) {
      currentServo2Angle -= SLOW_STEP_SIZE;
      if (currentServo2Angle < slowTargetServo2Angle) {
        currentServo2Angle = slowTargetServo2Angle;
      }
    }
    
    if (servo1Attached && servo2Attached && servoEnabled) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      lastSlowStepTime = millis();
    }
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
        toggleSnapOscillation();
        break;
        
      case IR_TWO:
        toggleSlowOscillation();
        break;
        
      case IR_THREE:
        toggleSineOscillation();
        break;
    }
    
    IrReceiver.resume();
  }
}

void adjustServo1(int degrees) {
  if (!servoEnabled) return;
  
  stopAllOscillations();
  
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
  
  stopAllOscillations();
  
  int newAngle = currentServo2Angle + degrees;
  newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
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

void toggleSnapOscillation() {
  if (!servoEnabled) return;
  
  stopOtherOscillations(1);
  
  if (snapOscillatingMode) {
    snapOscillatingMode = false;
    flashLED(2, 200);
  } else {
    snapOscillatingMode = true;
    
    int avgAngle = (currentServo1Angle + currentServo2Angle) / 2;
    if (avgAngle >= SERVO_CENTER_ANGLE) {
      snapPosition = true;
      currentServo1Angle = SERVO_MAX_ANGLE;
      currentServo2Angle = SERVO2_MAX_ANGLE;
    } else {
      snapPosition = false;
      currentServo1Angle = SERVO_MIN_ANGLE;
      currentServo2Angle = SERVO_MIN_ANGLE;
    }
    
    if (servo1Attached && servo2Attached) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      servoMoving = true;
      lastServoCommand = millis();
    }
    
    flashLED(4, 100);
  }
}

void toggleSlowOscillation() {
  if (!servoEnabled) return;
  
  stopOtherOscillations(2);
  
  if (slowOscillatingMode) {
    slowOscillatingMode = false;
    flashLED(2, 400);
  } else {
    slowOscillatingMode = true;
    
    int avgAngle = (currentServo1Angle + currentServo2Angle) / 2;
    if (avgAngle >= SERVO_CENTER_ANGLE) {
      slowTargetServo1Angle = SERVO_MIN_ANGLE;
      slowTargetServo2Angle = SERVO_MIN_ANGLE;
      snapPosition = false;
    } else {
      slowTargetServo1Angle = SERVO_MAX_ANGLE;
      slowTargetServo2Angle = SERVO2_MAX_ANGLE;
      snapPosition = true;
    }
    
    lastSlowStepTime = millis();
    flashLED(6, 100);
  }
}

void toggleSineOscillation() {
  if (!servoEnabled) return;
  
  stopOtherOscillations(3);
  
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
    
    stopAllOscillations();
    servoMoving = false;
    currentServo1Angle = SERVO_CENTER_ANGLE;
    currentServo2Angle = SERVO_CENTER_ANGLE;
    servo1.write(SERVO_CENTER_ANGLE);
    servo2.write(SERVO_CENTER_ANGLE);
    
    flashLED(3, 100);
  }
}

void disableServo() {
  if (servoEnabled) {
    servoEnabled = false;
    stopAllOscillations();
    servoMoving = false;
    
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

void stopAllOscillations() {
  snapOscillatingMode = false;
  slowOscillatingMode = false;
  sineOscillatingMode = false;
}

void stopOtherOscillations(int keepMode) {
  if (keepMode != 1) snapOscillatingMode = false;
  if (keepMode != 2) slowOscillatingMode = false;
  if (keepMode != 3) sineOscillatingMode = false;
}

void enforceServoSafety() {
  if (!servoEnabled) return;
  
  currentServo1Angle = constrain(currentServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  currentServo2Angle = constrain(currentServo2Angle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
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