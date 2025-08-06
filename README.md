# Visual Gesture Control for Anthropomorphic Hand Gripper

This research uses a visual pipeline with MediaPipe and OpenCV to control a soft hand gripper. The goal is to create a controlled soft robotics hand that responds to hand movement that can be replicated easily.

**Made by Nikhil Shokeen @ Worcester Polytechnic Institute (WPI)**  
**Mentor: Neehaal Sharrma**

![Pipeline](Pipeline.png)

## Quick Start

### Prerequisites
- Python 3.8+
- Arduino IDE
- Webcam
- Arduino Uno + Servo Motors + PWM Driver

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd MediaPipeWPI
   ```

2. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload Arduino code**
   - Open `arduino_servo_control.ino` in Arduino IDE
   - Upload to your Arduino Uno

4. **Run the application**
   ```bash
   python hand_tracking_robot.py
   ```

## Documentation

For detailed setup instructions, troubleshooting, and technical documentation, visit our **[Wiki](https://github.com/yourusername/MediaPipeWPI/wiki)**:

- [Hardware Setup Guide](https://github.com/yourusername/MediaPipeWPI/wiki/Hardware-Setup)
- [Software Installation](https://github.com/yourusername/MediaPipeWPI/wiki/Software-Installation)
- [Troubleshooting Guide](https://github.com/yourusername/MediaPipeWPI/wiki/Troubleshooting)
- [Technical Specifications](https://github.com/yourusername/MediaPipeWPI/wiki/Technical-Specifications)

## Project Structure
```
MediaPipeWPI/
├── hand_tracking_robot.py      # Main application
├── test_camera.py              # Camera testing utility
├── arduino_servo_control.ino   # Arduino servo control code
├── requirements.txt            # Python dependencies
└── README.md                   # This file
```

## Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License
This project is developed at Worcester Polytechnic Institute (WPI) for research purposes.

---

**For technical support or questions, please contact:**
- **Nikhil Shokeen** - [Your Email]
- **WPI Robotics Engineering Department**

