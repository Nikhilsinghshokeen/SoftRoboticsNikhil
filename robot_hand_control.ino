//Code for Anthropogenic Soft Robotic Hand, visual pipeline to control hand using arduino
//Copyright (C) 2025  Nikhil Shokeen
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU Affero General Public License as published
//by the Free Software Foundation, either version 3 of the License, or
//at your option) any later version.

//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU Affero General Public License for more details.

//You should have received a copy of the GNU Affero General Public License
//along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
