// Arduino IR Remote Servo Controller - Sine Wave Oscillation Mode
// Press "1" button to toggle snap oscillation between 0% (0°) and 100% (180°)
// Press "2" button to toggle slow oscillation at half speed (900ms per rotation)
// Press "3" button to toggle sine wave oscillation with smooth motion

#include <Servo.h>
#include <IRremote.h>

// Define PI constant for sine wave calculations
#ifndef PI
#define PI 3.14159265359
#endif

// Pin definitions
const int SERVO1_PIN = 9;   // Servo-1 on pin 9
const int SERVO2_PIN = 10;  // Servo-2 on pin 10
const int BATTERY_PIN = A7;
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   // Servo 1 left by 5°
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  // Servo 1 right by 5°
const uint32_t IR_VOL_UP = 0xB946FF00;      // Servo 2 left by 5°
const uint32_t IR_VOL_DOWN = 0xEA15FF00;    // Servo 2 right by 5°
const uint32_t IR_PLAY_PAUSE = 0xBF40FF00;  
const uint32_t IR_POWER = 0xBA45FF00;       
const uint32_t IR_ONE = 0xF30CFF00;         // Triggers snap oscillation 0° <-> 180°
const uint32_t IR_TWO = 0xE718FF00;         // Triggers slow oscillation at half speed
const uint32_t IR_THREE = 0xA15EFF00;       // Triggers sine wave oscillation

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;    // 0% rotation
const int SERVO_MAX_ANGLE = 180;  // 100% rotation
const int SERVO2_MAX_ANGLE = 125; // Servo-2 maximum angle limit
const int SERVO_CENTER_ANGLE = 90;
const int SERVO_STEP_SIZE = 5;    // 5 degree increments for IR remote control

// Battery monitoring constants
const float LOW_BATTERY_THRESHOLD = 6.5;
const unsigned long BATTERY_CHECK_INTERVAL = 5000;
const unsigned long PRINT_INTERVAL = 1000;

// Oscillation timing constants
const unsigned long SERVO_SETTLE_TIME = 450; // Time to wait for servo to reach position (ms) - full speed
const unsigned long SERVO_SLOW_SETTLE_TIME = 927; // Time to wait for servo at half speed (ms)

// Slow oscillation step control
const int SLOW_STEP_SIZE = 2; // Degrees per step for smooth slow movement
const int SLOW_STEP_DELAY = 10; // Milliseconds between each step (10ms * 90 steps = 900ms total)

// Sine wave oscillation constants
const unsigned long SINE_UPDATE_INTERVAL = 20; // Update servo position every 20ms for smooth motion
const float SINE_FREQUENCY = 0.5; // Oscillations per second (adjustable)
const int SINE_AMPLITUDE = 90; // Maximum deviation from center (90° = full range 0-180°)
const int SINE_CENTER = 90; // Center position for sine wave

// Core servo control variables
Servo servo1;  // Servo-1 on pin 9
Servo servo2;  // Servo-2 on pin 10
int currentServo1Angle = SERVO_CENTER_ANGLE;  // Individual servo 1 angle
int currentServo2Angle = SERVO_CENTER_ANGLE;  // Individual servo 2 angle
unsigned long lastBatteryCheck = 0;
unsigned long lastPrintTime = 0;
unsigned long lastIRCommand = 0;
unsigned long lastServoCommand = 0;
bool servoMoving = false;
bool servo1Attached = false;
bool servo2Attached = false;
bool lowBatteryWarning = false;
bool servoEnabled = true;
bool snapOscillatingMode = false;
bool slowOscillatingMode = false;
bool sineOscillatingMode = false;
bool snapPosition = true; // true = at max position, false = at 0°

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

// Function declarations
void printStartupInfo();
void startupLEDSequence();
void handleIRRemote();
void handleSerialCommands();
void updateSineOscillation();
void updateSnapOscillation();
void updateSlowOscillation();
void printStatusInfo();
void checkBatteryVoltage();
void enforceServoSafety();
void setServoAngle(int angle);
void setServo1Angle(int angle);
void setServo2Angle(int angle);
void moveServoLeft();
void moveServoRight();
void moveServoCenter();
void moveServo1Left();
void moveServo1Right();
void moveServo2Left();
void moveServo2Right();
void adjustServo1(int degrees);
void adjustServo2(int degrees);
void testServo();
void emergencyStop();
void printIRCodes();
void enableServo();
void disableServo();
void toggleSnapOscillation();
void toggleSlowOscillation();
void toggleSineOscillation();
void toggleServo();
void setSineFrequency(float frequency);
void setSineAmplitude(int amplitude);
void flashLED(int count, int delayTime);

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("IR Remote receiver initialized on pin "));
  Serial.println(IR_RECEIVE_PIN);
  
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
  
  Serial.println(F("SERVO-1 (PIN 9) AND SERVO-2 (PIN 10) ATTACHED AND POSITIONED TO 90°"));
  delay(500);
  
  printStartupInfo();
  startupLEDSequence();
}

void loop() {
  // Handle IR with highest priority
  handleIRRemote();
  
  // Handle oscillation modes with highest priority during oscillation
  if ((snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) && servoEnabled) {
    if (snapOscillatingMode) {
      updateSnapOscillation();
    } else if (slowOscillatingMode) {
      updateSlowOscillation();
    } else if (sineOscillatingMode) {
      updateSineOscillation();
    }
    // Extra IR check during oscillation for responsiveness
    handleIRRemote(); 
  }
  
  handleSerialCommands();
  
  // Reduce status printing frequency to minimize interference
  if (millis() - lastPrintTime >= PRINT_INTERVAL && !snapOscillatingMode && !slowOscillatingMode && !sineOscillatingMode) {
    printStatusInfo();
    lastPrintTime = millis();
  }
  
  if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    checkBatteryVoltage();
    lastBatteryCheck = millis();
  }
  
  enforceServoSafety();
}

void updateSineOscillation() {
  unsigned long currentTime = millis();
  
  // Check if it's time to update the sine position
  if (currentTime - lastSineUpdate >= SINE_UPDATE_INTERVAL) {
    // Calculate time elapsed since sine oscillation started
    float timeElapsed = (currentTime - sineStartTime) / 1000.0; // Convert to seconds
    
    // Calculate sine wave value (-1 to 1)
    float sineValue = sin(2 * PI * sineFrequency * timeElapsed);
    
    // Map sine value to servo angles
    // sineValue ranges from -1 to 1
    // We want to map this to SINE_CENTER ± SINE_AMPLITUDE
    int targetServo1Angle = SINE_CENTER + (int)(sineValue * sineAmplitude);
    int targetServo2Angle = SINE_CENTER + (int)(sineValue * sineAmplitude);
    
    // Constrain to valid servo ranges
    targetServo1Angle = constrain(targetServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    targetServo2Angle = constrain(targetServo2Angle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
    
    // Only update servos if position has changed significantly (reduces jitter)
    if (abs(targetServo1Angle - currentServo1Angle) >= 1 || abs(targetServo2Angle - currentServo2Angle) >= 1) {
      currentServo1Angle = targetServo1Angle;
      currentServo2Angle = targetServo2Angle;
      
      if (servo1Attached && servo2Attached && servoEnabled) {
        servo1.write(currentServo1Angle);
        servo2.write(currentServo2Angle);
        
        // Optional: Brief LED flash every full cycle
        static float lastSineValue = 0;
        if ((lastSineValue < 0 && sineValue >= 0) || (lastSineValue > 0 && sineValue <= 0)) {
          digitalWrite(LED_BUILTIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(LED_BUILTIN, LOW);
        }
        lastSineValue = sineValue;
      }
    }
    
    lastSineUpdate = currentTime;
    
    // Debug output every second
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 1000) {
      Serial.print(F("SINE: t="));
      Serial.print(timeElapsed, 2);
      Serial.print(F("s, sin="));
      Serial.print(sineValue, 3);
      Serial.print(F(", S1="));
      Serial.print(currentServo1Angle);
      Serial.print(F("°, S2="));
      Serial.print(currentServo2Angle);
      Serial.println(F("°"));
      lastDebugPrint = currentTime;
    }
  }
}

void updateSnapOscillation() {
  // Check if servo has reached its target position and is ready for next command
  if (!servoMoving || (millis() - lastServoCommand >= SERVO_SETTLE_TIME)) {
    
    // Servo has settled, time to snap to the other position
    if (snapPosition) {
      // Currently at max position, snap to 0°
      currentServo1Angle = SERVO_MIN_ANGLE;
      currentServo2Angle = SERVO_MIN_ANGLE;
      snapPosition = false;
      Serial.println(F("SNAP: Max -> 0°"));
    } else {
      // Currently at 0°, snap to max positions
      currentServo1Angle = SERVO_MAX_ANGLE;
      currentServo2Angle = SERVO2_MAX_ANGLE;  // Servo-2 limited to 125°
      snapPosition = true;
      Serial.print(F("SNAP: 0° -> Max (S1="));
      Serial.print(SERVO_MAX_ANGLE);
      Serial.print(F("°, S2="));
      Serial.print(SERVO2_MAX_ANGLE);
      Serial.println(F("°)"));
    }
    
    // Send command to both servos and mark as moving
    if (servo1Attached && servo2Attached && servoEnabled) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      servoMoving = true;
      lastServoCommand = millis();
      
      // Quick LED flash without blocking delay
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(50);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void updateSlowOscillation() {
  // Check if it's time to take the next step
  unsigned long currentTime = millis(); 
  unsigned long deltaTime = currentTime - lastDirChange;
  if (currentTime - lastSlowStepTime >= SLOW_STEP_DELAY) {
    
    // Check if we've reached the target positions
    if (currentServo1Angle == slowTargetServo1Angle && currentServo2Angle == slowTargetServo2Angle) {
      Serial.print("SLOW: Time between direction change is ");
      Serial.println(deltaTime);
      lastDirChange = currentTime;

      // We've reached the targets, time to set new targets in the opposite direction
      if (slowTargetServo1Angle == SERVO_MAX_ANGLE) {
        // Currently at max positions, next targets are 0°
        slowTargetServo1Angle = SERVO_MIN_ANGLE;
        slowTargetServo2Angle = SERVO_MIN_ANGLE;
        snapPosition = false;
        Serial.println(F("SLOW: Starting move Max -> 0° (900ms)"));
      } else {
        // Currently at 0°, next targets are max positions
        slowTargetServo1Angle = SERVO_MAX_ANGLE;
        slowTargetServo2Angle = SERVO2_MAX_ANGLE;  // Servo-2 limited to 125°
        snapPosition = true;
        Serial.print(F("SLOW: Starting move 0° -> Max (S1="));
        Serial.print(SERVO_MAX_ANGLE);
        Serial.print(F("°, S2="));
        Serial.print(SERVO2_MAX_ANGLE);
        Serial.println(F("°) (900ms)"));
      }
    }
    
    // Move one step closer to the targets for each servo
    // Servo 1 movement
    if (currentServo1Angle < slowTargetServo1Angle) {
      currentServo1Angle += SLOW_STEP_SIZE;
      if (currentServo1Angle > slowTargetServo1Angle) {
        currentServo1Angle = slowTargetServo1Angle; // Don't overshoot
      }
    } else if (currentServo1Angle > slowTargetServo1Angle) {
      currentServo1Angle -= SLOW_STEP_SIZE;
      if (currentServo1Angle < slowTargetServo1Angle) {
        currentServo1Angle = slowTargetServo1Angle; // Don't overshoot
      }
    }
    
    // Servo 2 movement
    if (currentServo2Angle < slowTargetServo2Angle) {
      currentServo2Angle += SLOW_STEP_SIZE;
      if (currentServo2Angle > slowTargetServo2Angle) {
        currentServo2Angle = slowTargetServo2Angle; // Don't overshoot
      }
    } else if (currentServo2Angle > slowTargetServo2Angle) {
      currentServo2Angle -= SLOW_STEP_SIZE;
      if (currentServo2Angle < slowTargetServo2Angle) {
        currentServo2Angle = slowTargetServo2Angle; // Don't overshoot
      }
    }
    
    // Send the new positions to both servos
    if (servo1Attached && servo2Attached && servoEnabled) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      lastSlowStepTime = millis();
      
      // Brief LED flash every 10 steps to show activity without being annoying
      static int stepCounter = 0;
      stepCounter++;
      if (stepCounter >= 10) {
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(25);
        digitalWrite(LED_BUILTIN, LOW);
        stepCounter = 0;
      }
    }
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command.startsWith("A")) {
      int angle = command.substring(1).toInt();
      setServoAngle(angle);
    } else if (command.startsWith("FREQ")) {
      float freq = command.substring(4).toFloat();
      setSineFrequency(freq);
    } else if (command.startsWith("AMP")) {
      int amp = command.substring(3).toInt();
      setSineAmplitude(amp);
    } else if (command == "LEFT") {
      moveServoLeft();
    } else if (command == "RIGHT") {
      moveServoRight();
    } else if (command == "CENTER") {
      moveServoCenter();
    } else if (command == "TEST") {
      testServo();
    } else if (command == "STOP") {
      emergencyStop();
    } else if (command == "IRCODES") {
      printIRCodes();
    } else if (command == "ENABLE") {
      enableServo();
    } else if (command == "DISABLE") {
      disableServo();
    } else if (command == "SNAP") {
      toggleSnapOscillation();
    } else if (command == "SLOW") {
      toggleSlowOscillation();
    } else if (command == "SINE") {
      toggleSineOscillation();
    } else if (command.length() > 0) {
      Serial.println(F("Commands: A## (angle), FREQ# (Hz), AMP# (degrees), LEFT, RIGHT, CENTER"));
      Serial.println(F("          TEST, STOP, IRCODES, ENABLE, DISABLE, SNAP, SLOW, SINE"));
    }
  }
}

void setSineFrequency(float frequency) {
  if (frequency > 0 && frequency <= 5.0) {
    sineFrequency = frequency;
    Serial.print(F("Sine frequency set to "));
    Serial.print(sineFrequency, 2);
    Serial.println(F(" Hz"));
    
    // Reset sine wave timing if currently oscillating
    if (sineOscillatingMode) {
      sineStartTime = millis();
    }
  } else {
    Serial.println(F("Invalid frequency! Use 0.1 to 5.0 Hz"));
  }
}

void setSineAmplitude(int amplitude) {
  if (amplitude > 0 && amplitude <= 90) {
    sineAmplitude = amplitude;
    Serial.print(F("Sine amplitude set to ±"));
    Serial.print(sineAmplitude);
    Serial.println(F("° from center"));
  } else {
    Serial.println(F("Invalid amplitude! Use 1 to 90 degrees"));
  }
}

void toggleSineOscillation() {
  if (!servoEnabled) {
    Serial.println(F("Servos are disabled - cannot oscillate"));
    return;
  }
  
  // Stop other oscillation modes if they're running
  if (snapOscillatingMode) {
    snapOscillatingMode = false;
    Serial.println(F("Snap oscillation stopped"));
  }
  if (slowOscillatingMode) {
    slowOscillatingMode = false;
    Serial.println(F("Slow oscillation stopped"));
  }
  
  if (sineOscillatingMode) {
    sineOscillatingMode = false;
    Serial.println(F("SINE OSCILLATION MODE OFF - Both servos will hold current position"));
    Serial.print(F("Current positions: S1="));
    Serial.print(currentServo1Angle);
    Serial.print(F("°, S2="));
    Serial.print(currentServo2Angle);
    Serial.println(F("°"));
    flashLED(3, 300);
  } else {
    sineOscillatingMode = true;
    sineStartTime = millis();
    lastSineUpdate = millis();
    
    Serial.println(F("SINE OSCILLATION MODE ON - Both servos"));
    Serial.print(F("Frequency: "));
    Serial.print(sineFrequency, 2);
    Serial.print(F(" Hz, Amplitude: ±"));
    Serial.print(sineAmplitude);
    Serial.print(F("°, Center: "));
    Serial.print(SINE_CENTER);
    Serial.println(F("°"));
    Serial.print(F("Period: "));
    Serial.print(1.0 / sineFrequency, 2);
    Serial.println(F(" seconds"));
    Serial.println(F("Note: Servo-2 limited to 125° maximum"));
    
    flashLED(4, 150);
  }
}

void setServoAngle(int angle) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
    // Stop any oscillation when manually setting position
    if (snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) {
      snapOscillatingMode = false;
      slowOscillatingMode = false;
      sineOscillatingMode = false;
      Serial.println(F("Oscillation mode stopped"));
    }
    
    currentServo1Angle = angle;
    // Constrain Servo-2 to its maximum limit
    currentServo2Angle = constrain(angle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
    
    // Ensure both servos are attached and write position
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
      Serial.println(F("Servo-1 reattached"));
    }
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
      Serial.println(F("Servo-2 reattached"));
    }
    
    servo1.write(currentServo1Angle);
    servo2.write(currentServo2Angle);
    
    if (currentServo1Angle == currentServo2Angle) {
      Serial.print(F("Both servos moved to "));
      Serial.print(currentServo1Angle);
      Serial.println(F("°"));
    } else {
      Serial.print(F("Servos moved to S1="));
      Serial.print(currentServo1Angle);
      Serial.print(F("°, S2="));
      Serial.print(currentServo2Angle);
      Serial.println(F("° (S2 limited to 125°)"));
    }
    flashLED(1, 100);
  } else {
    Serial.println(F("Invalid angle! Use 0-180"));
  }
}

void setServo1Angle(int angle) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
    // Stop any oscillation when manually setting position
    if (snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) {
      snapOscillatingMode = false;
      slowOscillatingMode = false;
      sineOscillatingMode = false;
      Serial.println(F("Oscillation mode stopped"));
    }
    
    currentServo1Angle = angle;
    
    // Ensure servo is attached and write position
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
      Serial.println(F("Servo-1 reattached"));
    }
    
    servo1.write(currentServo1Angle);
    Serial.print(F("Servo-1 moved to "));
    Serial.print(currentServo1Angle);
    Serial.println(F("°"));
    flashLED(1, 100);
  } else {
    Serial.println(F("Invalid angle! Use 0-180"));
  }
}

void setServo2Angle(int angle) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  if (angle >= SERVO_MIN_ANGLE && angle <= SERVO2_MAX_ANGLE) {
    // Stop any oscillation when manually setting position
    if (snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) {
      snapOscillatingMode = false;
      slowOscillatingMode = false;
      sineOscillatingMode = false;
      Serial.println(F("Oscillation mode stopped"));
    }
    
    currentServo2Angle = angle;
    
    // Ensure servo is attached and write position
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
      Serial.println(F("Servo-2 reattached"));
    }
    
    servo2.write(currentServo2Angle);
    Serial.print(F("Servo-2 moved to "));
    Serial.print(currentServo2Angle);
    Serial.println(F("°"));
    flashLED(1, 100);
  } else {
    if (angle > SERVO2_MAX_ANGLE) {
      Serial.print(F("Invalid angle! Servo-2 limited to 0-"));
      Serial.print(SERVO2_MAX_ANGLE);
      Serial.println(F("°"));
    } else {
      Serial.println(F("Invalid angle! Use 0-125"));
    }
  }
}

void adjustServo1(int degrees) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  // Stop any oscillation when manually adjusting position
  if (snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) {
    snapOscillatingMode = false;
    slowOscillatingMode = false;
    sineOscillatingMode = false;
    Serial.println(F("Oscillation mode stopped"));
  }
  
  int newAngle = currentServo1Angle + degrees;
  newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  if (newAngle != currentServo1Angle) {
    currentServo1Angle = newAngle;
    
    // Ensure servo is attached and write position  
    if (!servo1Attached) {
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
      Serial.println(F("Servo-1 reattached"));
    }
    
    servo1.write(currentServo1Angle);
    Serial.print(F("Servo-1 adjusted by "));
    Serial.print(degrees);
    Serial.print(F("° to "));
    Serial.print(currentServo1Angle);
    Serial.println(F("°"));
    flashLED(1, 50);
  } else {
    Serial.print(F("Servo-1 at limit ("));
    Serial.print(currentServo1Angle);
    Serial.println(F("°) - cannot adjust further"));
  }
}

void adjustServo2(int degrees) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  // Stop any oscillation when manually adjusting position
  if (snapOscillatingMode || slowOscillatingMode || sineOscillatingMode) {
    snapOscillatingMode = false;
    slowOscillatingMode = false;
    sineOscillatingMode = false;
    Serial.println(F("Oscillation mode stopped"));
  }
  
  int newAngle = currentServo2Angle + degrees;
  newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
  if (newAngle != currentServo2Angle) {
    currentServo2Angle = newAngle;
    
    // Ensure servo is attached and write position
    if (!servo2Attached) {
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
      Serial.println(F("Servo-2 reattached"));
    }
    
    servo2.write(currentServo2Angle);
    Serial.print(F("Servo-2 adjusted by "));
    Serial.print(degrees);
    Serial.print(F("° to "));
    Serial.print(currentServo2Angle);
    Serial.println(F("°"));
    flashLED(1, 50);
  } else {
    Serial.print(F("Servo-2 at limit ("));
    Serial.print(currentServo2Angle);
    if (currentServo2Angle >= SERVO2_MAX_ANGLE) {
      Serial.print(F("° - maximum "));
      Serial.print(SERVO2_MAX_ANGLE);
      Serial.println(F("°) - cannot adjust further"));
    } else {
      Serial.println(F("° - minimum 0°) - cannot adjust further"));
    }
  }
}

void moveServoLeft() {
  Serial.println(F("Moving both servos to LEFT (180°) - 100%"));
  setServoAngle(SERVO_MAX_ANGLE);
}

void moveServoRight() {
  Serial.println(F("Moving both servos to RIGHT (0°) - 0%"));
  setServoAngle(SERVO_MIN_ANGLE);
}

void moveServoCenter() {
  Serial.println(F("Moving both servos to CENTER (90°) - 50%"));
  setServoAngle(SERVO_CENTER_ANGLE);
}

void moveServo1Left() {
  Serial.println(F("Moving Servo-1 to LEFT (180°) - 100%"));
  setServo1Angle(SERVO_MAX_ANGLE);
}

void moveServo1Right() {
  Serial.println(F("Moving Servo-1 to RIGHT (0°) - 0%"));
  setServo1Angle(SERVO_MIN_ANGLE);
}

void moveServo2Left() {
  Serial.println(F("Moving Servo-2 to LEFT (125°) - Maximum"));
  setServo2Angle(SERVO2_MAX_ANGLE);
}

void moveServo2Right() {
  Serial.println(F("Moving Servo-2 to RIGHT (0°) - 0%"));
  setServo2Angle(SERVO_MIN_ANGLE);
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
    
    // Show IR codes only when not oscillating to reduce serial spam
    if (receivedCode != 0x0 && !snapOscillatingMode && !slowOscillatingMode && !sineOscillatingMode) {
      Serial.print(F("IR Code: 0x"));
      Serial.println(receivedCode, HEX);
    }
    
    switch (receivedCode) {
      case IR_SKIP_LEFT:
        Serial.println(F("IR: Skip Left - Adjusting Servo-1 LEFT by 5°"));
        adjustServo1(SERVO_STEP_SIZE);
        lastIRCommand = millis();
        break;
        
      case IR_SKIP_RIGHT:
        Serial.println(F("IR: Skip Right - Adjusting Servo-1 RIGHT by 5°"));
        adjustServo1(-SERVO_STEP_SIZE);
        lastIRCommand = millis();
        break;
        
      case IR_VOL_UP:
        Serial.println(F("IR: Vol Up - Adjusting Servo-2 LEFT by 5°"));
        adjustServo2(SERVO_STEP_SIZE);
        lastIRCommand = millis();
        break;
        
      case IR_VOL_DOWN:
        Serial.println(F("IR: Vol Down - Adjusting Servo-2 RIGHT by 5°"));
        adjustServo2(-SERVO_STEP_SIZE);
        lastIRCommand = millis();
        break;
        
      case IR_PLAY_PAUSE:
        Serial.println(F("IR: Play/Pause - Moving both servos to CENTER (90°) - 50%"));
        moveServoCenter();
        lastIRCommand = millis();
        break;
        
      case IR_POWER:
        Serial.println(F("IR: Power - Toggling servos enable/disable"));
        toggleServo();
        lastIRCommand = millis();
        break;
        
      case IR_ONE:
        Serial.println(F("IR: 1 - Toggling SNAP oscillation (0° <-> 180°)"));
        toggleSnapOscillation();
        lastIRCommand = millis();
        break;
        
      case IR_TWO:
        Serial.println(F("IR: 2 - Toggling SLOW oscillation (0° <-> 180° at half speed)"));
        toggleSlowOscillation();
        lastIRCommand = millis();
        break;
        
      case IR_THREE:
        Serial.println(F("IR: 3 - Toggling SINE oscillation (smooth sine wave)"));
        toggleSineOscillation();
        lastIRCommand = millis();
        break;
        
      default:
        if (receivedCode != 0x0 && !snapOscillatingMode && !slowOscillatingMode && !sineOscillatingMode) {
          Serial.print(F("IR: Unknown code 0x"));
          Serial.println(receivedCode, HEX);
        }
        break;
    }
    
    IrReceiver.resume();
  }
}

void toggleSnapOscillation() {
  if (!servoEnabled) {
    Serial.println(F("Servos are disabled - cannot oscillate"));
    return;
  }
  
  // Stop other oscillation modes if they're running
  if (slowOscillatingMode) {
    slowOscillatingMode = false;
    Serial.println(F("Slow oscillation stopped"));
  }
  if (sineOscillatingMode) {
    sineOscillatingMode = false;
    Serial.println(F("Sine oscillation stopped"));
  }
  
  if (snapOscillatingMode) {
    snapOscillatingMode = false;
    Serial.println(F("SNAP OSCILLATION MODE OFF - Both servos will hold current position"));
    Serial.print(F("Current positions: S1="));
    Serial.print(currentServo1Angle);
    Serial.print(F("°, S2="));
    Serial.print(currentServo2Angle);
    Serial.println(F("°"));
    flashLED(2, 200);
  } else {
    snapOscillatingMode = true;
    
    // Determine starting position based on current angles
    int avgAngle = (currentServo1Angle + currentServo2Angle) / 2;
    if (avgAngle >= SERVO_CENTER_ANGLE) {
      // Start at max positions
      snapPosition = true;
      currentServo1Angle = SERVO_MAX_ANGLE;
      currentServo2Angle = SERVO2_MAX_ANGLE;  // Servo-2 limited to 125°
      Serial.print(F("SNAP OSCILLATION MODE ON - Starting at Max (S1="));
      Serial.print(SERVO_MAX_ANGLE);
      Serial.print(F("°, S2="));
      Serial.print(SERVO2_MAX_ANGLE);
      Serial.println(F("°)"));
    } else {
      // Start at 0° position
      snapPosition = false;
      currentServo1Angle = SERVO_MIN_ANGLE;
      currentServo2Angle = SERVO_MIN_ANGLE;
      Serial.println(F("SNAP OSCILLATION MODE ON - Starting both servos at 0°"));
    }
    
    // Move both servos to starting position immediately
    if (servo1Attached && servo2Attached) {
      servo1.write(currentServo1Angle);
      servo2.write(currentServo2Angle);
      servoMoving = true;
      lastServoCommand = millis();
    }
    
    Serial.println(F("Fast oscillation - 450ms per rotation"));
    flashLED(4, 100);
  }
}

void toggleSlowOscillation() {
  if (!servoEnabled) {
    Serial.println(F("Servos are disabled - cannot oscillate"));
    return;
  }
  
  // Stop other oscillation modes if they're running
  if (snapOscillatingMode) {
    snapOscillatingMode = false;
    Serial.println(F("Snap oscillation stopped"));
  }
  if (sineOscillatingMode) {
    sineOscillatingMode = false;
    Serial.println(F("Sine oscillation stopped"));
  }
  
  if (slowOscillatingMode) {
    slowOscillatingMode = false;
    Serial.println(F("SLOW OSCILLATION MODE OFF - Both servos will hold current position"));
    Serial.print(F("Current positions: S1="));
    Serial.print(currentServo1Angle);
    Serial.print(F("°, S2="));
    Serial.print(currentServo2Angle);
    Serial.println(F("°"));
    flashLED(2, 400); // Longer flash for slow mode
  } else {
    slowOscillatingMode = true;
    
    // Determine starting position and targets based on current angles
    int avgAngle = (currentServo1Angle + currentServo2Angle) / 2;
    if (avgAngle >= SERVO_CENTER_ANGLE) {
      // Start moving toward 0° from current positions
      slowTargetServo1Angle = SERVO_MIN_ANGLE;
      slowTargetServo2Angle = SERVO_MIN_ANGLE;
      snapPosition = false;
      Serial.println(F("SLOW OSCILLATION MODE ON - Moving both servos toward 0°"));
    } else {
      // Start moving toward max positions from current positions
      slowTargetServo1Angle = SERVO_MAX_ANGLE;
      slowTargetServo2Angle = SERVO2_MAX_ANGLE;  // Servo-2 limited to 125°
      snapPosition = true;
      Serial.print(F("SLOW OSCILLATION MODE ON - Moving toward Max (S1="));
      Serial.print(SERVO_MAX_ANGLE);
      Serial.print(F("°, S2="));
      Serial.print(SERVO2_MAX_ANGLE);
      Serial.println(F("°)"));
    }
    
    // Initialize timing
    lastSlowStepTime = millis();
    
    Serial.print(F("Smooth slow oscillation - 2° steps every 10ms ("));
    Serial.print((180 / SLOW_STEP_SIZE) * SLOW_STEP_DELAY);
    Serial.println(F("ms per full rotation)"));
    flashLED(6, 100); // More flashes for slow mode
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
    
    snapOscillatingMode = false;
    slowOscillatingMode = false;
    sineOscillatingMode = false;
    servoMoving = false;
    currentServo1Angle = SERVO_CENTER_ANGLE;
    currentServo2Angle = SERVO_CENTER_ANGLE;
    servo1.write(SERVO_CENTER_ANGLE);
    servo2.write(SERVO_CENTER_ANGLE);
    
    Serial.println(F("BOTH SERVOS ENABLED - Position: 90° (50%)"));
    flashLED(3, 100);
  } else {
    Serial.println(F("Servos already enabled"));
  }
}

void disableServo() {
  if (servoEnabled) {
    servoEnabled = false;
    snapOscillatingMode = false;
    slowOscillatingMode = false;
    sineOscillatingMode = false;
    servoMoving = false;
    
    if (servo1Attached) {
      servo1.detach();
      servo1Attached = false;
    }
    if (servo2Attached) {
      servo2.detach();
      servo2Attached = false;
    }
    
    Serial.println(F("BOTH SERVOS DISABLED"));
    flashLED(5, 200);
  } else {
    Serial.println(F("Servos already disabled"));
  }
}

void printIRCodes() {
  Serial.println(F("=== IR REMOTE CODES ==="));
  Serial.println(F("Skip Left:   0xBB44FF00 (Servo-1 +5°)"));
  Serial.println(F("Skip Right:  0xBC43FF00 (Servo-1 -5°)"));
  Serial.println(F("Vol Up:      0xB946FF00 (Servo-2 +5°)"));
  Serial.println(F("Vol Down:    0xEA15FF00 (Servo-2 -5°)"));
  Serial.println(F("Play/Pause:  0xBF40FF00 (Both servos to 90°)"));
  Serial.println(F("Power:       0xBA45FF00 (Toggle Enable/Disable)"));
  Serial.println(F("1:           0xF30CFF00 (Toggle SNAP Oscillation 0°<->180°)"));
  Serial.println(F("2:           0xE718FF00 (Toggle SLOW Oscillation 0°<->180° - Half Speed)"));
  Serial.println(F("3:           0xA15EFF00 (Toggle SINE Oscillation - Smooth Sine Wave)"));
  Serial.println(F("========================"));
}

void enforceServoSafety() {
  if (!servoEnabled) {
    return;
  }
  
  currentServo1Angle = constrain(currentServo1Angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  currentServo2Angle = constrain(currentServo2Angle, SERVO_MIN_ANGLE, SERVO2_MAX_ANGLE);
  
  if ((!servo1Attached || !servo2Attached) && servoEnabled) {
    if (!servo1Attached) {
      Serial.println(F("WARNING: Servo-1 not attached! Reattaching..."));
      servo1.attach(SERVO1_PIN);
      servo1Attached = true;
    }
    if (!servo2Attached) {
      Serial.println(F("WARNING: Servo-2 not attached! Reattaching..."));
      servo2.attach(SERVO2_PIN);
      servo2Attached = true;
    }
    flashLED(3, 50);
  }
}

void printStatusInfo() {
  Serial.print(F("Servo-1: "));
  Serial.print(currentServo1Angle);
  Serial.print(F("° | Servo-2: "));
  Serial.print(currentServo2Angle);
  Serial.print(F("° ["));
  
  if (!servoEnabled) {
    Serial.print(F("DISABLED"));
  } else if (snapOscillatingMode) {
    Serial.print(F("SNAP OSCILLATING "));
    Serial.print(snapPosition ? "180°" : "0°");
  } else if (slowOscillatingMode) {
    Serial.print(F("SLOW OSCILLATING "));
    Serial.print(snapPosition ? "180°" : "0°");
  } else if (sineOscillatingMode) {
    Serial.print(F("SINE OSCILLATING "));
    Serial.print(sineFrequency, 1);
    Serial.print(F("Hz"));
  } else {
    Serial.print(F("HOLDING"));
  }
  
  Serial.println(F("]"));
}

void checkBatteryVoltage() {
  int rawReading = analogRead(BATTERY_PIN);
  float voltage = rawReading * (5.0 / 1023.0) * 2.0;
  
  if (voltage < LOW_BATTERY_THRESHOLD && !lowBatteryWarning) {
    Serial.print(F("LOW BATTERY: "));
    Serial.print(voltage, 1);
    Serial.println(F("V"));
    flashLED(6, 50);
    lowBatteryWarning = true;
  } else if (voltage >= LOW_BATTERY_THRESHOLD + 0.5 && lowBatteryWarning) {
    lowBatteryWarning = false;
    Serial.print(F("Battery OK: "));
    Serial.print(voltage, 1);
    Serial.println(F("V"));
  }
}

void emergencyStop() {
  Serial.println(F("EMERGENCY STOP!"));
  if (servo1Attached) {
    servo1.detach();
    servo1Attached = false;
  }
  if (servo2Attached) {
    servo2.detach();
    servo2Attached = false;
  }
  
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    Serial.println(F("System halted. Reset to resume."));
  }
}

void testServo() {
  Serial.println(F("=== SERVO POSITION TEST ==="));
  
  bool wasSnapping = snapOscillatingMode;
  bool wasSlow = slowOscillatingMode;
  bool wasSine = sineOscillatingMode;
  snapOscillatingMode = false;
  slowOscillatingMode = false;
  sineOscillatingMode = false;
  
  flashLED(3, 100);
  
  Serial.println(F("Testing both servos RIGHT (0°)..."));
  moveServoRight();
  delay(1000);
  
  Serial.println(F("Testing both servos LEFT (Servo-1: 180°, Servo-2: 125°)..."));
  moveServoLeft();
  delay(1000);
  
  Serial.println(F("Testing both servos CENTER (90°)..."));
  moveServoCenter();
  delay(1000);
  
  Serial.println(F("Testing Servo-1 individual control..."));
  moveServo1Left();
  delay(1000);
  moveServo1Right();
  delay(1000);
  
  Serial.println(F("Testing Servo-2 individual control (0° to 125°)..."));
  moveServo2Left();
  delay(1000);
  moveServo2Right();
  delay(1000);
  
  Serial.println(F("Returning both to center..."));
  moveServoCenter();
  delay(1000);
  
  if (wasSnapping) {
    snapOscillatingMode = true;
    Serial.println(F("Snap oscillation restored"));
  } else if (wasSlow) {
    slowOscillatingMode = true;
    Serial.println(F("Slow oscillation restored"));
  } else if (wasSine) {
    sineOscillatingMode = true;
    sineStartTime = millis(); // Reset sine wave timing
    Serial.println(F("Sine oscillation restored"));
  }
  
  Serial.println(F("=== TEST COMPLETE ==="));
  flashLED(2, 500);
}

void printStartupInfo() {
  Serial.println(F("====================================="));
  Serial.println(F("Dual IR Remote Servo Controller"));
  Serial.println(F("Servo-1: Pin 9 (0-180°) | Servo-2: Pin 10 (0-125°)"));
  Serial.println(F("IR Remote Controls (5° increments):"));
  Serial.println(F("Skip Left/Right - Servo-1 ±5°"));
  Serial.println(F("Vol Up/Down - Servo-2 ±5° (max 125°)"));
  Serial.println(F("Press '1' for Fast Oscillation (450ms)"));
  Serial.println(F("Press '2' for Slow Oscillation (900ms)"));
  Serial.println(F("Press '3' for Sine Wave Oscillation"));
  Serial.println(F("====================================="));
  Serial.println(F("Serial Commands:"));
  Serial.println(F("SINE - Toggle sine oscillation"));
  Serial.println(F("FREQ# - Set frequency (0.1-5.0 Hz)"));
  Serial.println(F("AMP# - Set amplitude (1-90 degrees)"));
  Serial.println(F("LEFT, RIGHT, CENTER"));
  Serial.println(F("====================================="));
  Serial.println(F("Default: 90° (CENTER position)"));
  Serial.print(F("Sine defaults: "));
  Serial.print(SINE_FREQUENCY, 1);
  Serial.print(F("Hz, ±"));
  Serial.print(SINE_AMPLITUDE);
  Serial.println(F("°"));
  Serial.println(F("Note: Servo-2 limited to 125° maximum"));
  Serial.println(F("====================================="));
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