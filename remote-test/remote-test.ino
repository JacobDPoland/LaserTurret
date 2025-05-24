// Arduino IR Remote Servo Controller - Snap Oscillation Mode
// Press "1" button to toggle snap oscillation between 0% (0°) and 100% (180°)

#include <Servo.h>
#include <IRremote.h>

// Pin definitions
const int SERVO_PIN = 9;
const int BATTERY_PIN = A7;
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  
const uint32_t IR_PLAY_PAUSE = 0xBF40FF00;  
const uint32_t IR_POWER = 0xBA45FF00;       
const uint32_t IR_ONE = 0xF30CFF00;         // Triggers snap oscillation 0° <-> 180°

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;    // 0% rotation
const int SERVO_MAX_ANGLE = 180;  // 100% rotation
const int SERVO_CENTER_ANGLE = 90;

// Battery monitoring constants
const float LOW_BATTERY_THRESHOLD = 6.5;
const unsigned long BATTERY_CHECK_INTERVAL = 5000;
const unsigned long PRINT_INTERVAL = 1000;

// Snap oscillation timing - position-based instead of time-based
const unsigned long SERVO_SETTLE_TIME = 450; // Time to wait for servo to reach position (ms)

// Core servo control variables
Servo myServo;
int currentServoAngle = SERVO_CENTER_ANGLE;
unsigned long lastBatteryCheck = 0;
unsigned long lastPrintTime = 0;
unsigned long lastIRCommand = 0;
unsigned long lastServoCommand = 0;
bool servoMoving = false;
bool servoAttached = false;
bool lowBatteryWarning = false;
bool servoEnabled = true;
bool snapOscillatingMode = false;
bool snapPosition = true; // true = at 180°, false = at 0°

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("IR Remote receiver initialized on pin "));
  Serial.println(IR_RECEIVE_PIN);
  
  // Initialize servo at center position
  myServo.attach(SERVO_PIN);
  servoAttached = true;
  servoEnabled = true;
  myServo.write(SERVO_CENTER_ANGLE);
  currentServoAngle = SERVO_CENTER_ANGLE;
  
  Serial.println(F("SERVO ATTACHED AND POSITIONED TO 90°"));
  delay(500);
  
  printStartupInfo();
  startupLEDSequence();
}

void loop() {
  // Handle IR with highest priority
  handleIRRemote();
  
  // Handle snap oscillation with highest priority during oscillation
  if (snapOscillatingMode && servoEnabled) {
    updateSnapOscillation();
    // Extra IR check during oscillation for responsiveness
    handleIRRemote(); 
  }
  
  handleSerialCommands();
  
  // Reduce status printing frequency to minimize interference
  if (millis() - lastPrintTime >= PRINT_INTERVAL && !snapOscillatingMode) {
    printStatusInfo();
    lastPrintTime = millis();
  }
  
  if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    checkBatteryVoltage();
    lastBatteryCheck = millis();
  }
  
  enforceServoSafety();
}

void updateSnapOscillation() {
  // Check if servo has reached its target position and is ready for next command
  if (!servoMoving || (millis() - lastServoCommand >= SERVO_SETTLE_TIME)) {
    
    // Servo has settled, time to snap to the other position
    if (snapPosition) {
      // Currently at 180°, snap to 0°
      currentServoAngle = SERVO_MIN_ANGLE;
      snapPosition = false;
      Serial.println(F("SNAP: 180° -> 0%"));
    } else {
      // Currently at 0°, snap to 180°
      currentServoAngle = SERVO_MAX_ANGLE;
      snapPosition = true;
      Serial.println(F("SNAP: 0° -> 100%"));
    }
    
    // Send command to servo and mark as moving
    if (servoAttached && servoEnabled) {
      myServo.write(currentServoAngle);
      servoMoving = true;
      lastServoCommand = millis();
      
      // Quick LED flash without blocking delay
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(50);
      digitalWrite(LED_BUILTIN, LOW);
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
    } else if (command == "DEBUG") {
      debugServo();
    } else if (command.length() > 0) {
      Serial.println(F("Commands: A## (angle 0-180), LEFT, RIGHT, CENTER, TEST, STOP, IRCODES, ENABLE, DISABLE, SNAP, DEBUG"));
    }
  }
}

void debugServo() {
  Serial.println(F("=== SERVO DEBUG INFO ==="));
  Serial.print(F("Servo Attached: "));
  Serial.println(servoAttached ? "YES" : "NO");
  Serial.print(F("Servo Enabled: "));
  Serial.println(servoEnabled ? "YES" : "NO");
  Serial.print(F("Current Angle: "));
  Serial.println(currentServoAngle);
  Serial.print(F("Snap Oscillating: "));
  Serial.println(snapOscillatingMode ? "YES" : "NO");
  if (snapOscillatingMode) {
    Serial.print(F("Current Position: "));
    Serial.println(snapPosition ? "180° (100%)" : "0° (0%)");
    Serial.print(F("Next Snap In: "));
    unsigned long timeLeft = SERVO_SETTLE_TIME - (millis() - lastServoCommand);
    if (servoMoving && timeLeft > 0) {
      Serial.print(timeLeft);
      Serial.println(F(" ms (waiting for servo)"));
    } else {
      Serial.println(F("Ready to snap"));
    }
  }
  Serial.println(F("Forcing servo write..."));
  
  if (servoAttached) {
    myServo.write(currentServoAngle);
    Serial.println(F("Servo write command sent"));
  } else {
    Serial.println(F("ERROR: Servo not attached!"));
  }
  Serial.println(F("========================"));
}

void setServoAngle(int angle) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
    // Stop snap oscillation when manually setting position
    if (snapOscillatingMode) {
      snapOscillatingMode = false;
      Serial.println(F("Snap oscillation mode stopped"));
    }
    
    currentServoAngle = angle;
    
    // Ensure servo is attached and write position
    if (!servoAttached) {
      myServo.attach(SERVO_PIN);
      servoAttached = true;
      Serial.println(F("Servo reattached"));
    }
    
    myServo.write(currentServoAngle);
    Serial.print(F("Servo moved to "));
    Serial.print(currentServoAngle);
    Serial.println(F("°"));
    flashLED(1, 100);
  } else {
    Serial.println(F("Invalid angle! Use 0-180"));
  }
}

void moveServoLeft() {
  Serial.println(F("Moving servo to LEFT (180°) - 100%"));
  setServoAngle(SERVO_MAX_ANGLE);
}

void moveServoRight() {
  Serial.println(F("Moving servo to RIGHT (0°) - 0%"));
  setServoAngle(SERVO_MIN_ANGLE);
}

void moveServoCenter() {
  Serial.println(F("Moving servo to CENTER (90°) - 50%"));
  setServoAngle(SERVO_CENTER_ANGLE);
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
    if (receivedCode != 0x0 && !snapOscillatingMode) {
      Serial.print(F("IR Code: 0x"));
      Serial.println(receivedCode, HEX);
    }
    
    switch (receivedCode) {
      case IR_SKIP_LEFT:
        Serial.println(F("IR: Skip Left - Moving servo to LEFT (180°) - 100%"));
        moveServoLeft();
        lastIRCommand = millis();
        break;
        
      case IR_SKIP_RIGHT:
        Serial.println(F("IR: Skip Right - Moving servo to RIGHT (0°) - 0%"));
        moveServoRight();
        lastIRCommand = millis();
        break;
        
      case IR_PLAY_PAUSE:
        Serial.println(F("IR: Play/Pause - Moving servo to CENTER (90°) - 50%"));
        moveServoCenter();
        lastIRCommand = millis();
        break;
        
      case IR_POWER:
        Serial.println(F("IR: Power - Toggling servo enable/disable"));
        toggleServo();
        lastIRCommand = millis();
        break;
        
      case IR_ONE:
        Serial.println(F("IR: 1 - Toggling SNAP oscillation (0° <-> 180°)"));
        toggleSnapOscillation();
        lastIRCommand = millis();
        break;
        
      default:
        if (receivedCode != 0x0 && !snapOscillatingMode) {
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
    Serial.println(F("Servo is disabled - cannot oscillate"));
    return;
  }
  
  if (snapOscillatingMode) {
    snapOscillatingMode = false;
    Serial.println(F("SNAP OSCILLATION MODE OFF - Servo will hold current position"));
    Serial.print(F("Current position: "));
    Serial.print(currentServoAngle);
    Serial.println(F("°"));
    flashLED(2, 200);
  } else {
    snapOscillatingMode = true;
    
    // Determine starting position based on current angle
    if (currentServoAngle >= SERVO_CENTER_ANGLE) {
      // Start at 180° position
      snapPosition = true;
      currentServoAngle = SERVO_MAX_ANGLE;
      Serial.println(F("SNAP OSCILLATION MODE ON - Starting at 180° (100%)"));
    } else {
      // Start at 0° position
      snapPosition = false;
      currentServoAngle = SERVO_MIN_ANGLE;
      Serial.println(F("SNAP OSCILLATION MODE ON - Starting at 0° (0%)"));
    }
    
    // Move to starting position immediately
    if (servoAttached) {
      myServo.write(currentServoAngle);
      servoMoving = true;
      lastServoCommand = millis();
    }
    
    Serial.println(F("Position-based snapping - servo snaps when it reaches target"));
    flashLED(4, 100);
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
    if (!servoAttached) {
      myServo.attach(SERVO_PIN);
      servoAttached = true;
    }
    
    snapOscillatingMode = false;
    servoMoving = false;
    currentServoAngle = SERVO_CENTER_ANGLE;
    myServo.write(SERVO_CENTER_ANGLE);
    
    Serial.println(F("SERVO ENABLED - Position: 90° (50%)"));
    flashLED(3, 100);
  } else {
    Serial.println(F("Servo already enabled"));
  }
}

void disableServo() {
  if (servoEnabled) {
    servoEnabled = false;
    snapOscillatingMode = false;
    servoMoving = false;
    
    if (servoAttached) {
      myServo.detach();
      servoAttached = false;
    }
    
    Serial.println(F("SERVO DISABLED"));
    flashLED(5, 200);
  } else {
    Serial.println(F("Servo already disabled"));
  }
}

void printIRCodes() {
  Serial.println(F("=== IR REMOTE CODES ==="));
  Serial.println(F("Skip Left:   0xBB44FF00 (Servo to 180°)"));
  Serial.println(F("Skip Right:  0xBC43FF00 (Servo to 0°)"));
  Serial.println(F("Play/Pause:  0xBF40FF00 (Servo to 90°)"));
  Serial.println(F("Power:       0xBA45FF00 (Toggle Enable/Disable)"));
  Serial.println(F("1:           0xF30CFF00 (Toggle SNAP Oscillation 0°<->180°)"));
  Serial.println(F("========================"));
}

void enforceServoSafety() {
  if (!servoEnabled) {
    return;
  }
  
  currentServoAngle = constrain(currentServoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  if (!servoAttached && servoEnabled) {
    Serial.println(F("WARNING: Servo not attached! Reattaching..."));
    myServo.attach(SERVO_PIN);
    servoAttached = true;
    flashLED(3, 50);
  }
}

void printStatusInfo() {
  Serial.print(F("Servo: "));
  Serial.print(currentServoAngle);
  Serial.print(F("° ["));
  
  if (!servoEnabled) {
    Serial.print(F("DISABLED"));
  } else if (snapOscillatingMode) {
    Serial.print(F("SNAP OSCILLATING "));
    Serial.print(snapPosition ? "180°" : "0°");
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
  if (servoAttached) {
    myServo.detach();
    servoAttached = false;
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
  snapOscillatingMode = false;
  
  flashLED(3, 100);
  
  Serial.println(F("Testing RIGHT (0°)..."));
  moveServoRight();
  delay(1000);
  
  Serial.println(F("Testing LEFT (180°)..."));
  moveServoLeft();
  delay(1000);
  
  Serial.println(F("Testing CENTER (90°)..."));
  moveServoCenter();
  delay(1000);
  
  if (wasSnapping) {
    snapOscillatingMode = true;
    Serial.println(F("Snap oscillation restored"));
  }
  
  Serial.println(F("=== TEST COMPLETE ==="));
  flashLED(2, 500);
}

void printStartupInfo() {
  Serial.println(F("====================================="));
  Serial.println(F("IR Remote Servo Controller - SNAP MODE"));
  Serial.println(F("Press '1' for 0°<->180° Snap Oscillation"));
  Serial.println(F("====================================="));
  Serial.println(F("Commands: LEFT, RIGHT, CENTER, SNAP, DEBUG"));
  Serial.println(F("Default: 90° (CENTER position)"));
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