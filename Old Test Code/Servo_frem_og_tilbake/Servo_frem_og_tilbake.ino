const int pulsePin = 10;        // Define the pin you want to generate pulses on
const float CWdirection = 1;    // Pulse duration in milliseconds (1 second)  - Clockwise direction
const float CCWdirection = 2;   // Pulse duration in milliseconds (1 second) - Counter clockwise direction
const int CWsteps = 50;         // Number of steps in clockwise direction
const int CCWsteps = 50;        // Number of steps in counter clockwise direction
const int InPin = A0;           // Activate the analog input A0
const int frequency = 100;      // Set the frequency of the pulses
const int flex = 50;            // Resistance in flex. Under variable is stretched fingers
const int bent = 80;            // Resistance in flex. Over variable is bent finger 


// Generate a pulse by setting the pin to HIGH, setting duration of the puls, and setting the pin LOW again.
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
  pinMode(InPin, INPUT);      // Set input
  Serial.begin(9600);         // Lagringsbuffer, grei å ha

}

void loop() {
      float read = analogRead(InPin); // declear and read flex 
      Serial.println(read);           // Prints varable from flex.
      delay(500);                     // Thinkingbreak

      if(read < flex){                // Calles function to turn servo clockwise if finger is flexed
      clockwise();
      }
      
      if(read > 80){                  // Calles function to turn servi counterclockwise if finger is bent.
      counterclockwise();
      }



}




