# Hand Tracking Robot Control

A real-time hand tracking system that controls a robot hand using MediaPipe and Arduino.

Made by Nikhil Shokeen
Mentor Neehaal Sharrma

Connect servos to Arduino pins:
- Thumb: Pin 9
- Index: Pin 10
- Middle: Pin 11
- Ring: Pin 12
- Pinky: Pin 13

    SCL is in SCL pin red
    SDA is in SDA pin blue
    Data is in Digital Pin 7 white
    GND is in GND green
    POW 3.3v black

Media Pipe is used to track landmarks on your hand
Movement in your hand converts to movement in cable drive actuators

