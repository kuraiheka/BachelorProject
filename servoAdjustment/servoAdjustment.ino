int SERVO_ADJUSTMENT_UP = //select pin;
int SERVO_ADJUSTMENT_DOWN = //select pin;
int SERVO_ADJUSTMENT_NEXT = //select pin;
int next_switch_prior_state = LOW;
int current_adjustment_servo = 0;

void servoAdjustment(){
  int next_switch_current_state = digitalRead(SERVO_ADJUSTMENT_NEXT); // Read next switch state
  if (next_switch_current_state != next_switch_prior_state){ // Check if state has changed
    next_switch_prior_state = next_switch_current_state; // Update state
    if (next_switch_current_state == 1){ // Check if next switch is pressed
      
      // Cycle to next servo
      if (current_adjustment_servo < NUM_SERVOS-1){
        current_adjustment_servo ++;
      } else {
        current_adjustment_servo = 0;
      }

      //PrintDebug("Now adjusting motor %i.", current_adjustment_servo);
    }
  }

  if(digitalRead(SERVO_ADJUSTMENT_DOWN) == HIGH) { // Check if down button is pressed
    servos[current_adjustment_servo].writeMicroseconds(_config.ServoValueDown;); // Adjust servo down
   PrintDebug("Motor %i: adjusting down", _config.ServoPin);
  } else if(digitalRead(SERVO_ADJUSTMENT_UP) == HIGH) { // Check if up button is pressed
    servos[current_adjustment_servo].writeMicroseconds(_config.ServoValueUp); // Adjust servo up
    PrintDebug("Motor %i: adjusting up", _config.ServoPin);
}


void setup() {
  pinMode(SERVO_ADJUSTMENT_UP, INPUT);
  pinMode(SERVO_ADJUSTMENT_DOWN, INPUT);
  pinMode(SERVO_ADJUSTMENT_NEXT, INPUT);
}

void loop() {
  // Checks switches and adjusts servos if desired
  servoAdjustment();
}
