// VERSION WITH DIRECT POSITIONING
// RELATIVE TO VOLTAGE INPUT
// RESOLUTION IS 10 STEPS
// TIME INCREMENT IS 25*5 = 125ms  (otherwise motor slip will occur)


// what is the maximum motor position. Decrease to 10 to have fewer steps, and make it less sensitive
// but then also adjust the LENGTH_ONE_INTERVAL
#define MAX_MOTOR_POSITION 20

// How long is one loop() interval
#define LENGTH_ONE_INTERVAL 25000*5


#define DEFAULT_THRESHOLD_MOVE_DOWN_0 400
#define DEFAULT_THRESHOLD_MOVE_UP_0 750

#define DEFAULT_THRESHOLD_MOVE_DOWN_1 400
#define DEFAULT_THRESHOLD_MOVE_UP_1 650

#define DEFAULT_THRESHOLD_MOVE_DOWN_2 330
#define DEFAULT_THRESHOLD_MOVE_UP_2 640

#define DEFAULT_THRESHOLD_MOVE_DOWN_3 370
#define DEFAULT_THRESHOLD_MOVE_UP_3 670

#define DEFAULT_THRESHOLD_MOVE_DOWN_4 450
#define DEFAULT_THRESHOLD_MOVE_UP_4 600

// set this ABOVE the amount the value jitters around if it is not moves
#define FILTER_THRESHOLD    20

// comment the following line out to remove debug output
#define PRINT_DEBUG_OUTPUT  1

// comment the following line out to remove LED MATRIX support
#define ENABLE_UNO_R4_LED_MATRIX 1




#ifdef ENABLE_UNO_R4_LED_MATRIX == 1
#include "Arduino_LED_Matrix.h"
#endif

#include <Servo.h>

// (de)activate DEBUG output if requested
#ifdef PRINT_DEBUG_OUTPUT == 1
#define PrintDebugRaw(...) PrintInfoRaw(__VA_ARGS__)
#define PrintDebug(...) { PrintInfoRaw(__VA_ARGS__); Serial.println(); }
#else
#define PrintDebugRaw(...) {}
#define PrintDebug(...) {}
#endif


// reimplements the printf function over serial, without newline
void PrintInfoRaw(const  char* format, ...) {
  char* buffer = NULL;
  va_list args;
  va_start(args, format);
  size_t buf_size = vsnprintf(NULL, 0, format, args) + 1;
  buffer = (char*)alloca(buf_size);
  vsnprintf(buffer, buf_size, format, args);
  Serial.print(buffer);
  va_end(args);
}

// reimplements the printf function over serial, out newline
#define PrintInfo(...)  { PrintInfoRaw(__VA_ARGS__); Serial.println(); }

// configuration for a flex<>motor binding
struct InputConfig {
  int AnalogPin;
  int ThresholdDown;
  int ThresholdUp;
};

struct ServoConfig {
  int ServoPin;
  int ServoValueDown;
  int ServoValueStop;
  int ServoValueUp;
};

struct BindingConfig {
  InputConfig Input;
  ServoConfig Servo;
};

enum MotorMovement { FULL_STOP, GO_TO };

struct MotorCommand {
  MotorMovement Command;
  int Value;
};

// Class that encapsulates motor control
class ManagedServo {
  private:
    Servo _servo;
    ServoConfig _config;

    int _positionIs = 0;
    int _positionWant = 0;

  public:
    void configure(ServoConfig config) {
      _config = config;
      _servo.attach(_config.ServoPin);

      PrintInfo("Motor %i attached. Config values: %i/%i/%i", 
           _config.ServoPin, _config.ServoValueDown, _config.ServoValueStop, _config.ServoValueUp);
    }

    // set by control loop. Specifies where the motor should head to
    void setMotorCommand(MotorCommand command) {
      if(command.Command == GO_TO) {
        _positionWant = command.Value;
      } else if(command.Command == FULL_STOP ){
        _positionWant = _positionIs;
      } else {
        PrintInfo("Unknown command received: %i/%i", (int)command.Command, command.Value);
      }       
    }
       
    // in case you need to read the position
    int getPosition() {
      return _positionIs;
    }
    

    // perform one "step"
    int performOneStep() {
      int valueToWrite;

      if(_positionIs == _positionWant) {
        valueToWrite = _config.ServoValueStop;
      } else if (_positionIs > _positionWant) {
        valueToWrite = _config.ServoValueDown;
        _positionIs--;
      } else {
        valueToWrite = _config.ServoValueUp;
        _positionIs++;
      }

      PrintDebug("Motor %i: wanted: %i, is: %i. Writing: %i", _config.ServoPin, _positionWant, _positionIs, valueToWrite);

      _servo.writeMicroseconds(valueToWrite);

      return _positionIs;
    }
};

// same as built in map(), just correct.
long mapValue(long x, long in_min, long in_max, long out_min, long out_max) {
  if(x <= in_min) return out_min;
  if(x >= in_max) return out_max;

  return (long)((float)x - (float)in_min) * ((float)out_max - (float)out_min) / ((float)in_max - (float)in_min) + (float)out_min;
}

// class that encapsulates reading from an analog input and transforming this into a motor command
class ManagedAnalogReader {
  private:
    InputConfig _config;
    // memory for filtering
    int _lastValueThatCausedChange = -1;
    int _lastPosition = -1;

  public: 
    void configure(InputConfig config) {
      _config = config;

      PrintInfo("Input pin %i attached. (A0 is not '0' but pin %i)", _config.AnalogPin, A0);
    }

    MotorCommand determineMotorCommand() {
      // read the value from the analog input, and map it to the motor position
      int value = analogRead(_config.AnalogPin);
      auto newPos = mapValue(value, _config.ThresholdDown, _config.ThresholdUp, 0, MAX_MOTOR_POSITION);
  
      // only perform any action, if the difference is big enough

      if(_lastValueThatCausedChange >= 0) { // check if we have already formed some memory
        auto difference = abs(value-_lastValueThatCausedChange);
        if(  difference < FILTER_THRESHOLD ) {
          PrintDebug("Input %i: Value %i mapped to pos: %i. Threshold too low. filtered.", _config.AnalogPin, value, newPos);
          // return old values
          return (MotorCommand)  { GO_TO, _lastPosition };
        }
      }

      // remember the value that caused the change
      _lastValueThatCausedChange = value;
      _lastPosition = newPos;

      PrintDebug("Input %i: Value %i mapped to pos: %i", _config.AnalogPin, value, newPos);
      return (MotorCommand)  { GO_TO, newPos };
    }
};

// Helper class that makes sure each loop takes EXACTLY xxx microseconds
// by waiting at the top of the loop for an appropriate amount
// that amount is calculated
class ConstantIntervalTimer {
  private:
    unsigned long _lastMicros = 0;
    unsigned int _constantIntervalMicros = 0;

  public: 
    // the constructor takes the amount of microseconds each loop should exactly take
    ConstantIntervalTimer(int constantIntervalMicros) {
        _constantIntervalMicros = constantIntervalMicros;
    }

    // this is called to wait for an appropriate amount of time
    // we first determine how long ago the last call happened
    // then we determine how much longer we have to wait to reach the EXACT time
    // if for whatever reason we are already too late, we don't wait
    void waitForNextTick() {
      if(_lastMicros != 0) {
          unsigned long currentMicros = micros();
          unsigned long diff = currentMicros - _lastMicros; // note: if micros() runs over after 70 minutes and starts again at 0, this code still works, thanks to binary math
          long timeToWait = _constantIntervalMicros - diff;
          if( timeToWait > 0) {
            delayMicroseconds(timeToWait);
          }
        }
        _lastMicros = micros();
    }
};


// abstraction class to make frame buffer access to UNO R4 Wifi matrix easer
#ifdef ENABLE_UNO_R4_LED_MATRIX == 1
class MatrixMotorDisplay {
  private: 
    ArduinoLEDMatrix _matrix;
    uint32_t _frameBuffer[3] = {0,0,0};

  public: 
    void begin(){
      _matrix.begin();
      _matrix.loadFrame(_frameBuffer);
    }

    void clear() {
      _frameBuffer[0] = 0; 
      _frameBuffer[1] = 0; 
      _frameBuffer[2] = 0;
    }

    void setLane(int lane, int position) {
      if(position < 0 || position > 11 || lane < 0 || lane > 7) {
        PrintInfo("Invalid position for matrix call");
        return;
      }

      int bitIndex = position + (lane * 12);
      int idx = bitIndex / 32;
      int shift = bitIndex % 32;
      _frameBuffer[2-idx] |= 1 << shift;
    }

    void update() {
      _matrix.loadFrame(_frameBuffer);
    }
};
#endif


// helper function to get the length of an array
template<std::size_t N, class T> constexpr std::size_t countof(T(&)[N]) { return N; }


////
//// MAIN CONFIG per Analog<>Servo pair
//// 

BindingConfig configuration[] = {
  { {A0, DEFAULT_THRESHOLD_MOVE_DOWN_0, DEFAULT_THRESHOLD_MOVE_UP_0 }, { 5,  2000, 1500, 1000} }, 
  { {A1, DEFAULT_THRESHOLD_MOVE_DOWN_1, DEFAULT_THRESHOLD_MOVE_UP_1 }, { 6,  2000, 1500, 1000} }, 
  { {A2, DEFAULT_THRESHOLD_MOVE_DOWN_2, DEFAULT_THRESHOLD_MOVE_UP_2 }, { 9,  2000, 1500, 1000} }, 
  { {A3, DEFAULT_THRESHOLD_MOVE_DOWN_3, DEFAULT_THRESHOLD_MOVE_UP_3 }, { 10, 2000, 1500, 1000} }, 
  { {A4, DEFAULT_THRESHOLD_MOVE_DOWN_4, DEFAULT_THRESHOLD_MOVE_UP_4 }, { 11, 2000, 1500, 1000} }  
};

// used many times to determine the amount of servos
const int NUM_SERVOS = countof(configuration);

// instantiate the servos
ManagedServo servos[NUM_SERVOS];
ManagedAnalogReader inputs[NUM_SERVOS];

ConstantIntervalTimer loopTimer(LENGTH_ONE_INTERVAL); // 25000 microseconds unless changed at the top

MatrixMotorDisplay matrix;


void setup() {
  // WATCH OUT: The serial console should be at least 115200, or 1000000 if available
  // otherwise the Serial.println statements will slow down the program A LOT
  // make sure to set the serial monitor to the SAME value. 115200 is supported by all programs, 1000000 by arduino studio
  Serial.begin(1000000);

  PrintInfo("Startup");

  // attach the pins according to config
  for(int i =0; i < NUM_SERVOS; i++) {
    auto config = configuration[i];
    servos[i].configure(config.Servo);
    inputs[i].configure(config.Input);

    // Print information for A0 and A1 analog pins
    if (config.Input.AnalogPin == A0 || config.Input.AnalogPin == A1) {
      Serial.print("Configured motor for analog pin ");
      Serial.println(config.Input.AnalogPin);
    }
  }

#ifdef ENABLE_UNO_R4_LED_MATRIX == 1
  matrix.begin();
#endif

  PrintInfo("Done");
}



void loop() {
  

  // wait until it is the next turn. You can adjust the interval length up in the definitions of LENGHT_ONE_INTERVAL
  loopTimer.waitForNextTick(); 

  // first determine the motor commands and then pre-notify the motor of our intent
  for(int i = 0; i < NUM_SERVOS; i++) {
    auto command = inputs[i].determineMotorCommand();
    servos[i].setMotorCommand(command);
  }

  // execute one round of commands. we seperated the reading of the inputs and the motor commands
  // so that all motors seem to act at the SAME TIME to the outside world, and too keep the code clean
  for(int i = 0; i < NUM_SERVOS; i++) {
    servos[i].performOneStep();
  }

  // optional: indicate motor position on LED matrix found on Arduino UNO R4 WiFi.
#ifdef ENABLE_UNO_R4_LED_MATRIX == 1
  matrix.clear();
  for(int i = 0; i < NUM_SERVOS; i++) {
    int pos = servos[i].getPosition() / (MAX_MOTOR_POSITION/10);
    matrix.setLane(i, pos);  // pos needs to be 0-11, we keep it 0-10
  }
  matrix.update();

  
#endif  

  
}
