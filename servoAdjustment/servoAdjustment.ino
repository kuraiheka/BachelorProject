int SERVO_ADJUSTMENT_UP = //pin
int SERVO_ADJUSTMENT_DOWN = //pin
int current_adjustment_servo = 0


void setup() {
  pinMode(SERVO_ADJUSTMENT_UP, INPUT);
  pinMode(SERVO_ADJUSTMENT_DOWN, INPUT);

}

void loop() {
  // Adjust servo position if desired
  servos[current_adjustment_servo].adjustServoPosition()
}





// Checks if adjustment buttons are pressed and moves the servo accordingly, ignoring positioning
void adjustServoPosition(){
  if(digitalRead(SERVO_ADJUSTMENT_DOWN) == HIGH) {
    _servo.writeMicroseconds(_config.ServoValueDown;);
    PrintDebug("Motor %i: adjusting down", _config.ServoPin);
  } else if(digitalRead(SERVO_ADJUSTMENT_UP) == HIGH) {
    _servo.writeMicroseconds(_config.ServoValueUp);
    PrintDebug("Motor %i: adjusting up", _config.ServoPin);
  }
}