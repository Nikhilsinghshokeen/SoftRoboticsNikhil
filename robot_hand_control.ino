#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVO_FREQ 50 
#define SERVOMIN  150 
#define SERVOMAX  600 

// Servo mapping
const int SERVO_PINS[] = {0, 1, 2, 3, 4}; 
const char FINGER_NAMES[] = {'T', 'I', 'M', 'R', 'P'}; 

// Buffer for data
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10);
}

void loop() {
  if (stringComplete) {
    parseAndMoveServos(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseAndMoveServos(String data) {
  // Expected format: "T:angle,I:angle,M:angle,R:angle,P:angle"
  int startPos = 0;
  int endPos = 0;
  
  for (int i = 0; i < 5; i++) {
    // Find the start of the angle value
    startPos = data.indexOf(':', endPos) + 1;
    endPos = data.indexOf(',', startPos);
    if (endPos == -1) endPos = data.length();
    
    // Extract and convert angle
    String angleStr = data.substring(startPos, endPos);
    float angle = angleStr.toFloat();
    
    // Map angle to servo pulse length
    int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    
    // Move servo
    pwm.setPWM(SERVO_PINS[i], 0, pulse);
  }
} 