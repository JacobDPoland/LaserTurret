// Arduino Joystick Servo Controller - Maximum Speed Version with IR Remote
// Controls a servo motor at maximum speed using joystick left/right movement OR IR remote
// Up, down, and button inputs are ignored for safety
// Optimized for fastest possible servo response
// IR Remote: Skip Left/Right buttons control servo

#include <Servo.h>
#include <IRremote.h>

// Pin definitions (using const to store in flash memory)
const int VRX_PIN = A0;
const int VRY_PIN = A1;
const int SW_PIN = 2;
const int SERVO_PIN = 9;
const int BATTERY_PIN = A7;
const int IR_RECEIVE_PIN = 12;

// IR Remote button codes for Elegoo small remote
const uint32_t IR_SKIP_LEFT = 0xBB44FF00;   // Skip Left button
const uint32_t IR_SKIP_RIGHT = 0xBC43FF00;  // Skip Right button
const uint32_t IR_PLAY_PAUSE = 0xBF40FF00;  // Play/Pause (for center position)
const uint32_t IR_POWER = 0xBA45FF00;       // Power button (emergency stop)

// Joystick configuration constants
const int CENTER_X = 512;
const int CENTER_Y = 512;
const int DEADZONE = 50;
const int THRESHOLD = 200;

// Servo configuration constants
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_CENTER_ANGLE = 90;

// Battery monitoring constants
const float LOW_BATTERY_THRESHOLD = 6.5;
const unsigned long BATTERY_CHECK_INTERVAL = 5000;
const unsigned long PRINT_INTERVAL = 500; // Reduced frequency to not slow down servo

// Core servo control variables (optimized for speed)
Servo myServo;
int currentServoAngle = SERVO_CENTER_ANGLE;
int targetServoAngle = SERVO_CENTER_ANGLE;
int servoSpeed = 100;        // Always at maximum
int servoStepSize = 180;     // Maximum step size for instant movement
unsigned long lastServoUpdate = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastPrintTime = 0;
unsigned long lastIRCommand = 0;
bool servoAttached = false;
bool lowBatteryWarning = false;
bool irControlActive = false;

void setup() {
  Serial.begin(9600); // Restored original baud rate for stable communication
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize IR receiver (new library syntax)
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("IR Remote receiver initialized on pin "));
  Serial.println(IR_RECEIVE_PIN);
  
  // Initialize servo
  myServo.attach(SERVO_PIN);
  servoAttached = true;
  myServo.write(SERVO_CENTER_ANGLE);
  delay(500); // Reduced startup delay
  
  // Startup sequence
  printStartupInfo();
  startupLEDSequence();
}

void loop() {
  handleSerialCommands();
  handleIRRemote();
  
  // Read joystick and process movement (only if IR not recently used)
  if (!irControlActive || (millis() - lastIRCommand > 2000)) {
    int xValue = analogRead(VRX_PIN);
    int yValue = analogRead(VRY_PIN);
    bool buttonPressed = !digitalRead(SW_PIN);
    
    processJoystickMovement(xValue, yValue, buttonPressed);
  }
  
  updateServoPosition();
  
  // Periodic tasks (less frequent to not slow down servo)
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    int xValue = analogRead(VRX_PIN);
    printStatusInfo(xValue);
    lastPrintTime = millis();
  }
  
  if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    checkBatteryVoltage();
    lastBatteryCheck = millis();
  }
  
  enforceServoSafety();
  // Removed delay(10) for maximum speed - no artificial delays
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command.startsWith("S")) {
      int speed = command.substring(1).toInt();
      setServoSpeed(speed);
    } else if (command.startsWith("T")) {
      int stepSize = command.substring(1).toInt();
      setServoStepSize(stepSize);
    } else if (command == "CAL") {
      calibrateJoystick();
    } else if (command == "TEST") {
      testServo();
    } else if (command == "STOP") {
      emergencyStop();
    } else if (command == "MAXSPEED") {
      setMaximumSpeed();
    } else if (command == "IRCODES") {
      printIRCodes();
    } else if (command.length() > 0) {
      Serial.println(F("Commands: S## (speed 1-100), T## (step 1-180), CAL, TEST, STOP, MAXSPEED, IRCODES"));
    }
  }
}

void processJoystickMovement(int xValue, int yValue, bool buttonPressed) {
  int xRelative = xValue - CENTER_X;
  int yRelative = yValue - CENTER_Y;
  
  bool moveLeft = false;
  bool moveRight = false;
  bool moveUp = false;
  bool moveDown = false;
  
  // Process X-axis (active for servo control) - Direct position mapping for speed
  if (abs(xRelative) > DEADZONE) {
    if (xRelative > THRESHOLD) {
      moveRight = true;
      // For maximum speed, jump directly to target position
      targetServoAngle = SERVO_MIN_ANGLE; // Full right
    } else if (xRelative < -THRESHOLD) {
      moveLeft = true;
      // For maximum speed, jump directly to target position  
      targetServoAngle = SERVO_MAX_ANGLE; // Full left
    }
  } else {
    // Return to center when joystick is centered
    targetServoAngle = SERVO_CENTER_ANGLE;
  }
  
  // Check Y-axis (disabled for safety)
  if (abs(yRelative) > DEADZONE) {
    if (yRelative > THRESHOLD) moveDown = true;
    else if (yRelative < -THRESHOLD) moveUp = true;
  }
  
  // Log disabled inputs (less frequently to not slow down)
  if (moveUp || moveDown || buttonPressed) {
    logDisabledInputs(moveUp, moveDown, buttonPressed);
  }
}

void updateServoPosition() {
  // Remove all delays and update servo immediately for maximum speed
  if (currentServoAngle != targetServoAngle) {
    // For maximum speed, jump directly to target instead of gradual movement
    currentServoAngle = targetServoAngle;
    
    if (servoAttached) {
      myServo.write(currentServoAngle);
    }
    lastServoUpdate = millis();
  }
}

void setServoSpeed(int speed) {
  if (speed >= 1 && speed <= 100) {
    servoSpeed = speed;
    Serial.print(F("Speed set to: "));
    Serial.print(servoSpeed);
    Serial.println(F("/100 (Note: Max speed mode active)"));
  } else {
    Serial.println(F("Invalid speed! Use 1-100"));
  }
}

void setServoStepSize(int stepSize) {
  if (stepSize >= 1 && stepSize <= 180) {
    servoStepSize = stepSize;
    Serial.print(F("Step size set to: "));
    Serial.print(servoStepSize);
    Serial.println(F("° (Note: Direct positioning mode active)"));
  } else {
    Serial.println(F("Invalid step size! Use 1-180"));
  }
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
        Serial.println(F("IR: Skip Left - Moving servo to MAX (180°)"));
        targetServoAngle = SERVO_MAX_ANGLE;
        irControlActive = true;
        lastIRCommand = millis();
        flashLED(1, 50);
        break;
        
      case IR_SKIP_RIGHT:
        Serial.println(F("IR: Skip Right - Moving servo to MIN (0°)"));
        targetServoAngle = SERVO_MIN_ANGLE;
        irControlActive = true;
        lastIRCommand = millis();
        flashLED(1, 50);
        break;
        
      case IR_PLAY_PAUSE:
        Serial.println(F("IR: Play/Pause - Centering servo (90°)"));
        targetServoAngle = SERVO_CENTER_ANGLE;
        irControlActive = true;
        lastIRCommand = millis();
        flashLED(2, 100);
        break;
        
      case IR_POWER:
        Serial.println(F("IR: Power - Emergency Stop!"));
        emergencyStop();
        break;
        
      default:
        Serial.print(F("IR: Unknown code 0x"));
        Serial.println(receivedCode, HEX);
        break;
    }
    
    IrReceiver.resume(); // Enable receiving of the next value
  }
}

void setMaximumSpeed() {
  servoSpeed = 100;
  servoStepSize = 180;
  Serial.println(F("MAXIMUM SPEED MODE ACTIVATED!"));
  Serial.println(F("- Speed: 100/100"));
  Serial.println(F("- Step: 180° (full range)"));
  Serial.println(F("- Direct positioning enabled"));
  Serial.println(F("- All delays removed"));
  flashLED(5, 50);
}

void printIRCodes() {
  Serial.println(F("=== IR REMOTE CODES ==="));
  Serial.println(F("Skip Left:   0xFF22DD (Servo to 180°)"));
  Serial.println(F("Skip Right:  0xFFC23D (Servo to 0°)"));
  Serial.println(F("Play/Pause:  0xFF02FD (Servo to 90°)"));
  Serial.println(F("Power:       0xFFA25D (Emergency Stop)"));
  Serial.println(F("========================"));
  Serial.println(F("Point remote at IR sensor on pin 12"));
  Serial.println(F("If codes don't match, use IRCODES command"));
  Serial.println(F("and press buttons to see actual codes"));
}

void enforceServoSafety() {
  currentServoAngle = constrain(currentServoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  targetServoAngle = constrain(targetServoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  if (!servoAttached) {
    Serial.println(F("WARNING: Servo not attached! Reattaching..."));
    flashLED(10, 25); // Faster LED flash
    myServo.attach(SERVO_PIN);
    servoAttached = true;
  }
}

void logDisabledInputs(bool moveUp, bool moveDown, bool buttonPressed) {
  static unsigned long lastWarningTime = 0;
  unsigned long currentTime = millis();
  
  // Reduced warning frequency to not slow down main loop
  if (currentTime - lastWarningTime >= 3000) {
    if (moveUp) Serial.println(F("INFO: UP disabled"));
    if (moveDown) Serial.println(F("INFO: DOWN disabled"));
    if (buttonPressed) Serial.println(F("INFO: Button disabled"));
    
    // Quick LED flash without delay
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(10000); // 10ms in microseconds for precision
    digitalWrite(LED_BUILTIN, LOW);
    lastWarningTime = currentTime;
  }
}

void printStatusInfo(int xValue) {
  Serial.print(F("X:"));
  Serial.print(xValue);
  Serial.print(F(" Servo:"));
  Serial.print(currentServoAngle);
  Serial.print(F("° Target:"));
  Serial.print(targetServoAngle);
  Serial.print(F("° [MAX SPEED"));
  if (irControlActive && (millis() - lastIRCommand < 2000)) {
    Serial.print(F(" + IR"));
  }
  Serial.print(F("] Range:"));
  
  float servoPercent = ((float)(currentServoAngle - SERVO_MIN_ANGLE) / 
                       (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) * 100;
  Serial.print(servoPercent, 1);
  Serial.println(F("%"));
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

void calibrateJoystick() {
  Serial.println(F("=== JOYSTICK CALIBRATION ==="));
  Serial.println(F("Center joystick for 2 seconds...")); // Reduced time
  
  flashLED(3, 100); // Fewer flashes
  
  for (int i = 2; i > 0; i--) { // Reduced countdown
    Serial.print(i);
    Serial.println(F("..."));
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400); // Faster countdown
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  long xSum = 0, ySum = 0;
  const int samples = 50; // Fewer samples for speed
  
  Serial.println(F("Sampling..."));
  for (int i = 0; i < samples; i++) {
    xSum += analogRead(VRX_PIN);
    ySum += analogRead(VRY_PIN);
    if (i % 5 == 0) { // Faster LED update
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    delay(5); // Faster sampling
  }
  
  int centerX = xSum / samples;
  int centerY = ySum / samples;
  
  Serial.println(F("=== RESULTS ==="));
  Serial.print(F("Measured X: "));
  Serial.println(centerX);
  Serial.print(F("Measured Y: "));
  Serial.println(centerY);
  Serial.print(F("Current X: "));
  Serial.println(CENTER_X);
  Serial.print(F("Current Y: "));
  Serial.println(CENTER_Y);
  
  if (abs(centerX - CENTER_X) > 20 || abs(centerY - CENTER_Y) > 20) {
    Serial.println(F("Update CENTER_X and CENTER_Y constants"));
  } else {
    Serial.println(F("Calibration is good!"));
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void testServo() {
  Serial.println(F("=== SERVO SPEED TEST ==="));
  flashLED(3, 100);
  
  unsigned long startTime = millis();
  
  // Quick sweep to max (larger steps for speed)
  for (int angle = SERVO_CENTER_ANGLE; angle <= SERVO_MAX_ANGLE; angle += 10) {
    myServo.write(angle);
    currentServoAngle = angle;
    delay(20); // Minimal delay
  }
  
  // Quick sweep to min
  for (int angle = SERVO_MAX_ANGLE; angle >= SERVO_MIN_ANGLE; angle -= 10) {
    myServo.write(angle);
    currentServoAngle = angle;
    delay(20); // Minimal delay
  }
  
  // Quick return to center
  for (int angle = SERVO_MIN_ANGLE; angle <= SERVO_CENTER_ANGLE; angle += 10) {
    myServo.write(angle);
    currentServoAngle = angle;
    delay(20); // Minimal delay
  }
  
  unsigned long endTime = millis();
  targetServoAngle = SERVO_CENTER_ANGLE;
  
  Serial.println(F("=== SPEED TEST COMPLETE ==="));
  Serial.print(F("Full sweep time: "));
  Serial.print(endTime - startTime);
  Serial.println(F("ms"));
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void printStartupInfo() {
  Serial.println(F("====================================="));
  Serial.println(F("Servo Controller - MAX SPEED + IR!"));
  Serial.println(F("====================================="));
  Serial.println(F("JOYSTICK:"));
  Serial.println(F("  LEFT/RIGHT: Move servo (INSTANT)"));
  Serial.println(F("  UP/DOWN/BUTTON: Disabled"));
  Serial.println(F("IR REMOTE (Pin 12):"));
  Serial.println(F("  Skip Left:  Servo to 180°"));
  Serial.println(F("  Skip Right: Servo to 0°"));
  Serial.println(F("  Play/Pause: Servo to 90°"));
  Serial.println(F("  Power:      Emergency Stop"));
  Serial.println(F("Mode: MAXIMUM SPEED"));
  Serial.println(F("Commands: S##, T##, CAL, TEST, STOP, MAXSPEED, IRCODES"));
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