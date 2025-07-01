#include <Servo.h>

Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

const int THUMB_PIN = 9;
const int INDEX_PIN = 10;
const int MIDDLE_PIN = 11;
const int RING_PIN = 12;
const int PINKY_PIN = 13;

int thumbAngle = 90;
int indexAngle = 90;
int middleAngle = 90;
int ringAngle = 90;
int pinkyAngle = 90;

String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  
  thumbServo.attach(THUMB_PIN);
  indexServo.attach(INDEX_PIN);
  middleServo.attach(MIDDLE_PIN);
  ringServo.attach(RING_PIN);
  pinkyServo.attach(PINKY_PIN);
  
  thumbServo.write(thumbAngle);
  indexServo.write(indexAngle);
  middleServo.write(middleAngle);
  ringServo.write(ringAngle);
  pinkyServo.write(pinkyAngle);
  
  delay(1000);
  
  Serial.println("Robot hand ready");
}

void loop() {
  if (stringComplete) {
    parseCommand(inputString);
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

void parseCommand(String command) {
  command.trim();
  
  if (command.startsWith("T:") && command.indexOf("I:") > 0) {
    int tStart = command.indexOf("T:") + 2;
    int tEnd = command.indexOf(",I:");
    int iStart = command.indexOf("I:") + 2;
    int iEnd = command.indexOf(",M:");
    int mStart = command.indexOf("M:") + 2;
    int mEnd = command.indexOf(",R:");
    int rStart = command.indexOf("R:") + 2;
    int rEnd = command.indexOf(",P:");
    int pStart = command.indexOf("P:") + 2;
    
    if (tEnd > 0 && iEnd > 0 && mEnd > 0 && rEnd > 0) {
      thumbAngle = command.substring(tStart, tEnd).toInt();
      indexAngle = command.substring(iStart, iEnd).toInt();
      middleAngle = command.substring(mStart, mEnd).toInt();
      ringAngle = command.substring(rStart, rEnd).toInt();
      pinkyAngle = command.substring(pStart).toInt();
      
      thumbAngle = constrain(thumbAngle, 0, 180);
      indexAngle = constrain(indexAngle, 0, 180);
      middleAngle = constrain(middleAngle, 0, 180);
      ringAngle = constrain(ringAngle, 0, 180);
      pinkyAngle = constrain(pinkyAngle, 0, 180);
      
      thumbServo.write(thumbAngle);
      indexServo.write(indexAngle);
      middleServo.write(middleAngle);
      ringServo.write(ringAngle);
      pinkyServo.write(pinkyAngle);
    }
  }
} 