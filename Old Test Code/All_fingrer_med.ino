const int pulsePin[] = {6,7,8,9,10};           // Define the pin you want to generate pulses on
const float CWdirection = 1;                   // Pulse duration in milliseconds (1 second)  - Clockwise direction
const float CCWdirection = 2;                  // Pulse duration in milliseconds (1 second) - Counter clockwise direction
const int CWsteps = 50;                        // Number of steps in clockwise direction
const int CCWsteps = 50;                       // Number of steps in counter clockwise direction
const int InPin[] = {A0, A1, A2, A3, A4};      // Activate the analog input A0
const int frequency = 100;                     // Set the frequency of the pulses
const int flexr = 150;                         // Resistance in flex. Under variable is stretched fingers
const int bentr = 280;                         // Resistance in flex. Over variable is bent finger 


//Testversjon av counterclockwise alle motorer på en gang
void counterclockwise2(){
for (int i = 0; i < 5; i++){
  for (int step = 1; step <= CWsteps; step++) {
      digitalWrite(pulsePin[i], HIGH);}}
      delay(CWdirection);
for (int i = 0; i < 5; i++){
  for (int step = 1; step <= CWsteps; step++) {
      digitalWrite(pulsePin[i], LOW); }}   
      Serial.print("3");

delay(frequency);
}

// Generate a pulse by setting the pin to HIGH, setting duration of the puls, and setting the pin LOW again.
// Here Clockwise direction 
void counterclockwise(){
for (int i = 0; i < 5; i++){
  for (int step = 1; step <= CWsteps; step++) {
      digitalWrite(pulsePin[i], HIGH);
      delay(CWdirection);
      digitalWrite(pulsePin[i], LOW); 
      delay(10);    
      Serial.print("0");
}}
delay(frequency);
}

// Generate a pulse by setting the pins to HIGH, setting duration of the puls, and setting the pin LOW again.
// Here Counter clockwise direction 
void clockwise(){
for (int i = 0; i < 5; i++){
  for (int step = 1; step <= CCWsteps; step++) {
      digitalWrite(pulsePin[i], HIGH);
      delay(CCWdirection);
      digitalWrite(pulsePin[i], LOW);
      delay(10);
      Serial.print("1");
}}
delay(frequency);  // Setting frequenzy of puls signal
}

void setup() {
for (int i = 0; i < 5; i++){
  pinMode(pulsePin[i], OUTPUT);  // Set the pulsePin as an output 
  pinMode(InPin[i], INPUT);      // Set input
  Serial.begin(9600);            // Lagringsbuffer, grei å ha

}}

void loop() {
  unsigned long startMillis = millis();  // Lagre starttiden
      for (int i = 0; i < 5; i++){
      float read = analogRead(InPin[i]);   // declear and read flexes 
      Serial.println(read);               // Prints varable from flexes.
      
                            

      if(read < flexr){                 // Calles function to turn servo clockwise if finger is flexed
      clockwise();
      }
      
      if(read > bentr){                 // Calles function to turn servo counterclockwise if finger is bent.
      counterclockwise2();}
      delay(0);}                      // Thinkingbreak



  // Din kode her...

  unsigned long endMillis = millis();  // Lagre sluttiden
  unsigned long duration = endMillis - startMillis;  // Beregn varigheten

  Serial.print("Koden tok ");
  Serial.print(duration);
  Serial.println(" millisekunder.");

}




