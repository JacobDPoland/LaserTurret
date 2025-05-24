// Arduino IR Remote Servo Controller - Position Hold Version
// Controls a servo motor using IR remote only
// Servo holds position until new command is received
// Default position is 50% rotation (90°)

#include <Servo.h>
#include <IRremote.h>

// Pin definitions (using const to store in flash memory)
const int SERVO_PIN = 9;
const int BATTERY_PIN = A7;
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   // Skip Left button (move to 180°)
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  // Skip Right button (move to 0°)
const uint32_t IR_PLAY_PAUSE = 0xBF40FF00;  // Play/Pause (center to 90°)
const uint32_t IR_POWER = 0xBA45FF00;       // Power button (toggle enable/disable)

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_CENTER_ANGLE = 90;  // 50% position (default)

// Battery monitoring constants
const float LOW_BATTERY_THRESHOLD = 6.5;
const unsigned long BATTERY_CHECK_INTERVAL = 5000;
const unsigned long PRINT_INTERVAL = 500; // Reduced frequency to not slow down servo

// Core servo control variables
Servo myServo;
int currentServoAngle = SERVO_CENTER_ANGLE;  // Start at 50% position
unsigned long lastBatteryCheck = 0;
unsigned long lastPrintTime = 0;
unsigned long lastIRCommand = 0;
bool servoAttached = false;
bool lowBatteryWarning = false;
bool servoEnabled = true;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("IR Remote receiver initialized on pin "));
  Serial.println(IR_RECEIVE_PIN);
  
  // Initialize servo at center position (50%)
  myServo.attach(SERVO_PIN);
  servoAttached = true;
  servoEnabled = true;
  myServo.write(SERVO_CENTER_ANGLE);
  currentServoAngle = SERVO_CENTER_ANGLE;
  delay(500);
  
  // Startup sequence
  printStartupInfo();
  startupLEDSequence();
}

void loop() {
  handleSerialCommands();
  handleIRRemote();
  
  // Periodic tasks
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    printStatusInfo();
    lastPrintTime = millis();
  }
  
  if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    checkBatteryVoltage();
    lastBatteryCheck = millis();
  }
  
  enforceServoSafety();
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
    } else if (command.length() > 0) {
      Serial.println(F("Commands: A## (angle 0-180), LEFT, RIGHT, CENTER, TEST, STOP, IRCODES, ENABLE, DISABLE"));
    }
  }
}

void setServoAngle(int angle) {
  if (!servoEnabled) {
    Serial.println(F("Servo is disabled"));
    return;
  }
  
  if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
    currentServoAngle = angle;
    if (servoAttached) {
      myServo.write(currentServoAngle);
    }
    Serial.print(F("Servo moved to "));
    Serial.print(currentServoAngle);
    Serial.println(F("°"));
    flashLED(1, 100);
  } else {
    Serial.println(F("Invalid angle! Use 0-180"));
  }
}

void moveServoLeft() {
  Serial.println(F("Moving servo to LEFT (180°)"));
  setServoAngle(SERVO_MAX_ANGLE);
}

void moveServoRight() {
  Serial.println(F("Moving servo to RIGHT (0°)"));
  setServoAngle(SERVO_MIN_ANGLE);
}

void moveServoCenter() {
  Serial.println(F("Moving servo to CENTER (90°)"));
  setServoAngle(SERVO_CENTER_ANGLE);
}

void handleIRRemote() {
  if (IrReceiver.decode()) {
    uint32_t receivedCode = IrReceiver.decodedIRData.decodedRawData;
    
    // Handle repeat codes
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      // Ignore repeat for now - could be used for continuous movement
      IrReceiver.resume();
      return;
    }
    
    Serial.print(F("IR Code: 0x"));
    Serial.println(receivedCode, HEX);
    
    switch (receivedCode) {
      case IR_SKIP_LEFT:
        Serial.println(F("IR: Skip Left - Moving servo to LEFT (180°)"));
        moveServoLeft();
        lastIRCommand = millis();
        break;
        
      case IR_SKIP_RIGHT:
        Serial.println(F("IR: Skip Right - Moving servo to RIGHT (0°)"));
        moveServoRight();
        lastIRCommand = millis();
        break;
        
      case IR_PLAY_PAUSE:
        Serial.println(F("IR: Play/Pause - Moving servo to CENTER (90°)"));
        moveServoCenter();
        lastIRCommand = millis();
        break;
        
      case IR_POWER:
        Serial.println(F("IR: Power - Toggling servo enable/disable"));
        toggleServo();
        lastIRCommand = millis();
        break;
        
      default:
        Serial.print(F("IR: Unknown code 0x"));
        Serial.println(receivedCode, HEX);
        break;
    }
    
    IrReceiver.resume(); // Enable receiving of the next value
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
    // Return to center position (50%) when re-enabling
    currentServoAngle = SERVO_CENTER_ANGLE;
    myServo.write(SERVO_CENTER_ANGLE);
    
    Serial.println(F("SERVO ENABLED - Position: 90° (50%)"));
    flashLED(3, 100); // 3 quick flashes for enable
  } else {
    Serial.println(F("Servo already enabled"));
  }
}

void disableServo() {
  if (servoEnabled) {
    servoEnabled = false;
    // Detach servo to save power and prevent jitter
    if (servoAttached) {
      myServo.detach();
      servoAttached = false;
    }
    
    Serial.println(F("SERVO DISABLED"));
    flashLED(5, 200); // 5 slower flashes for disable
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
  Serial.println(F("========================"));
  Serial.println(F("Point remote at IR sensor on pin 12"));
  Serial.println(F("If codes don't match, use IRCODES command"));
  Serial.println(F("and press buttons to see actual codes"));
}

void enforceServoSafety() {
  // Only enforce safety if servo is enabled
  if (!servoEnabled) {
    return;
  }
  
  currentServoAngle = constrain(currentServoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  if (!servoAttached && servoEnabled) {
    Serial.println(F("WARNING: Servo not attached! Reattaching..."));
    flashLED(10, 25); // Faster LED flash
    myServo.attach(SERVO_PIN);
    servoAttached = true;
  }
}

void printStatusInfo() {
  Serial.print(F("Servo Position: "));
  Serial.print(currentServoAngle);
  Serial.print(F("° ("));
  
  float servoPercent = ((float)(currentServoAngle - SERVO_MIN_ANGLE) / 
                       (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) * 100;
  Serial.print(servoPercent, 1);
  Serial.print(F("%) ["));
  
  if (!servoEnabled) {
    Serial.print(F("DISABLED"));
  } else {
    Serial.print(F("ENABLED - HOLDING"));
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
    flashLED(6, 50); // Faster flash
    lowBatteryWarning = true;
  } else if (voltage >= LOW_BATTERY_THRESHOLD + 0.5 && lowBatteryWarning) {
    lowBatteryWarning = false;
    Serial.print(F("Battery OK: "));
    Serial.print(voltage, 1);
    Serial.println(F("V"));
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200); // Reduced delay
    digitalWrite(LED_BUILTIN, LOW);
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
    delay(250); // Faster emergency blink
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    Serial.println(F("System halted. Reset to resume."));
  }
}

void testServo() {
  Serial.println(F("=== SERVO POSITION TEST ==="));
  flashLED(3, 100);
  
  unsigned long startTime = millis();
  
  // Test all three positions
  Serial.println(F("Testing RIGHT (0°)..."));
  moveServoRight();
  delay(1000);
  
  Serial.println(F("Testing LEFT (180°)..."));
  moveServoLeft();
  delay(1000);
  
  Serial.println(F("Testing CENTER (90°)..."));
  moveServoCenter();
  delay(1000);
  
  unsigned long endTime = millis();
  
  Serial.println(F("=== TEST COMPLETE ==="));
  Serial.print(F("Test duration: "));
  Serial.print(endTime - startTime);
  Serial.println(F("ms"));
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void printStartupInfo() {
  Serial.println(F("====================================="));
  Serial.println(F("IR Remote Servo Controller"));
  Serial.println(F("Position Hold Mode"));
  Serial.println(F("====================================="));
  Serial.println(F("IR REMOTE (Pin 12):"));
  Serial.println(F("  Skip Left:  Move to 180° (LEFT)"));
  Serial.println(F("  Skip Right: Move to 0° (RIGHT)"));
  Serial.println(F("  Play/Pause: Move to 90° (CENTER)"));
  Serial.println(F("  Power:      Toggle Enable/Disable"));
  Serial.println(F(""));
  Serial.println(F("SERIAL COMMANDS:"));
  Serial.println(F("  A## - Move to angle (0-180°)"));
  Serial.println(F("  LEFT, RIGHT, CENTER - Quick positions"));
  Serial.println(F("  ENABLE, DISABLE - Toggle servo"));
  Serial.println(F("Default: 90° (50% rotation)"));
  Serial.println(F("Mode: Position Hold - Stays until moved"));
  Serial.println(F("====================================="));
}

void startupLEDSequence() {
  // Faster startup sequence
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void flashLED(int count, int delayTime) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayTime);
  }
}