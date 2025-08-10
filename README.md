# Visual Gesture Control for Anthropomorphic Hand Gripper

This research uses a visual pipeline with MediaPipe and OpenCV to control a soft hand gripper.

**Made by Nikhil Shokeen @ Worcester Polytechnic Institute (WPI)**  
**Mentor: Neehaal Sharrma**

![Pipeline](Pipeline.png)

## Quick Start

### Prerequisites
- Python 3.8+
- Arduino IDE
- Webcam
- Arduino Uno + Servo Motors + PWM Driver

### Setup
1. Clone repository
2. Run `./setup.sh`
3. Install dependencies: `pip install -r requirements.txt`
4. Upload `arduino_servo_control.ino` to Arduino
5. Run: `python hand_tracking_robot.py`

## Documentation
- [Hardware Setup](wiki/Hardware-Setup.md)
- [Software Installation](wiki/Software-Installation.md)
- [Troubleshooting](wiki/Troubleshooting.md)
- [Technical Specs](wiki/Technical-Specifications.md)

## Project Structure
```
├── hand_tracking_robot.py      # Main application
├── arduino_servo_control.ino   # Arduino code
├── requirements.txt            # Dependencies
├── wiki/                       # Documentation
└── setup.sh                    # Setup script
```

## Contributing
1. Fork repository
2. Create feature branch
3. Make changes
4. Submit pull request

## License
WPI Robotics Engineering Department research project.

