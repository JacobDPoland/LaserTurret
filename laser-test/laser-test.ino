# This file tests wether the digital pin is sufficient to turn the laser on/off
const int LASER_PIN = 7;
void setup() {
  // Set pin 13 as an output
  pinMode(LASER_PIN, OUTPUT);
  
  // Set the pin HIGH (5V)
  digitalWrite(LASER_PIN, HIGH);
  delay(5000);
  digitalWrite(LASER_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
