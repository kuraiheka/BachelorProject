const int pulsePin = 10;        // Define the pin you want to generate pulses on
const float CWdirection = 1;    // Pulse duration in milliseconds (1 second)  - Clockwise direction
const float CCWdirection = 2;   // Pulse duration in milliseconds (1 second) - Counter clockwise direction
const int CWsteps = 2;         // Number of steps in clockwise direction
const int CCWsteps = 2;        // Number of steps in counter clockwise direction
const int flexPin = A0;           // Activate the analog input A0
const int frequency = 100;      // Set the frequency of the pulses
//const int flex = 50;            // Resistance in flex. Under variable is stretched fingers
//const int bent = 80;            // Resistance in flex. Over variable is bent finger 

const int flexLowerBound = 50;
const int flexUpperBound = 1024;
const int flexThreshold = 5;
int flexPos = 0;

int flexCheck(){
  float read = analogRead(flexPin);
  if(read < flexLowerBound){
    read = flexLowerBound;
  }
  if(read > flexPos+flexThreshold){
    flexPos += flexThreshold;
    return(1);
  }
  if(read < flexPos-flexThreshold){
    flexPos -= flexThreshold;
    return(-1);
  }
  return(0);
}

// Generate a pulse by setting the pin to HIGH, setting duration of the pulse, and setting the pin LOW again.
// Here Clockwise direction 
void counterclockwise(){
  for (int step = 1; step <= CWsteps; step++) {
      digitalWrite(pulsePin, HIGH);
      delay(CWdirection);
      digitalWrite(pulsePin, LOW);
      delay(frequency);
}}

// Generate a pulse by setting the pin to HIGH, setting duration of the puls, and setting the pin LOW again.
// Here Counter clockwise direction 
void clockwise(){
  for (int step = 1; step <= CCWsteps; step++) {
      digitalWrite(pulsePin, HIGH);
      delay(CCWdirection);
      digitalWrite(pulsePin, LOW);
      delay(frequency);

}}

void setup() {
  pinMode(pulsePin, OUTPUT);  // Set the pulsePin as an output 
  pinMode(flexPin, INPUT);      // Set input
  Serial.begin(9600);         // Lagringsbuffer, grei å ha
  flexPos = analogRead(flexPin);  //Set initial value of flexPos
}

void loop() {
      float read = analogRead(flexPin); // declear and read flex 
      Serial.println(read);           // Prints varable from flex.

      int flexDirection = flexCheck();
      Serial.println(flexDirection);

      if(flexDirection < 0){               // Calles function to turn servo clockwise if finger is flexed
      clockwise();
      }
      
      if(flexDirection > 0){                // Calles function to turn servi counterclockwise if finger is bent.
      counterclockwise();
      }

      //delay(500);                     // Thinkingbreak
}




