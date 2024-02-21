const int flexPin = A0;

int flexLowerBound = 50;
int flexUpperBound = 1024;
int flexThreshold = 10;
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

void setup() {
  pinMode(flexPin, INPUT);
  Serial.begin(9600);
  Serial.println("Test");

  flexPos = analogRead(flexPin);
}

void loop() {
  Serial.println(flexCheck());
  delay(500);
}