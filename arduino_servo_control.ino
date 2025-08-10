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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servoMin = 150; 
const int servoMax = 600;

// Updated angle ranges to match Python's new mapping (0-180 range)
const int thumbMin = 0, thumbMax = 180;
const int indexMin = 0, indexMax = 180;
const int middleMin = 0, middleMax = 180;
const int ringMin = 0, ringMax = 180;
const int pinkyMin = 0, pinkyMax = 180;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); // Standard servo frequency
  Wire.setClock(400000); // Optional: speed up I2C
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int t = -1, i = -1, m = -1, r = -1, p = -1;

    // Parse the angles from the input string
    int tIdx = input.indexOf("T:");
    int iIdx = input.indexOf("I:");
    int mIdx = input.indexOf("M:");
    int rIdx = input.indexOf("R:");
    int pIdx = input.indexOf("P:");

    if (tIdx != -1) t = input.substring(tIdx + 2, input.indexOf(' ', tIdx + 2)).toInt();
    if (iIdx != -1) i = input.substring(iIdx + 2, input.indexOf(' ', iIdx + 2)).toInt();
    if (mIdx != -1) m = input.substring(mIdx + 2, input.indexOf(' ', mIdx + 2)).toInt();
    if (rIdx != -1) r = input.substring(rIdx + 2, input.indexOf(' ', rIdx + 2)).toInt();
    if (pIdx != -1) p = input.substring(pIdx + 2).toInt();

    // Map and set PWM for each servo if value is valid
    if (t != -1) {
      int pulse = map(t, thumbMin, thumbMax, servoMin, servoMax);
      pulse = constrain(pulse, servoMin, servoMax);
      pwm.setPWM(0, 0, pulse);
    }
    if (i != -1) {
      int pulse = map(i, indexMin, indexMax, servoMin, servoMax);
      pulse = constrain(pulse, servoMin, servoMax);
      pwm.setPWM(1, 0, pulse);
    }
    if (m != -1) {
      int pulse = map(m, middleMin, middleMax, servoMin, servoMax);
      pulse = constrain(pulse, servoMin, servoMax);
      pwm.setPWM(2, 0, pulse);
    }
    if (r != -1) {
      int pulse = map(r, ringMin, ringMax, servoMin, servoMax);
      pulse = constrain(pulse, servoMin, servoMax);
      pwm.setPWM(3, 0, pulse);
    }
    if (p != -1) {
      int pulse = map(p, pinkyMin, pinkyMax, servoMin, servoMax);
      pulse = constrain(pulse, servoMin, servoMax);
      pwm.setPWM(4, 0, pulse);
    }
  }
} 
