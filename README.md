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

2. **Initialize submodules and setup**
   ```bash
   ./setup.sh
   ```

3. **Install Python dependencies**
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

For detailed setup instructions, troubleshooting, and technical documentation, check the **wiki/** directory (included as a git submodule):

- [Hardware Setup Guide](wiki/Hardware-Setup.md)
- [Software Installation](wiki/Software-Installation.md)
- [Troubleshooting Guide](wiki/Troubleshooting.md)
- [Technical Specifications](wiki/Technical-Specifications.md)

**Note**: The wiki is included as a git submodule. Run `./setup.sh` to initialize it, or manually run:
```bash
git submodule init
git submodule update
```

## Project Structure
```
MediaPipeWPI/
├── hand_tracking_robot.py      # Main application
├── test_camera.py              # Camera testing utility
├── arduino_servo_control.ino   # Arduino servo control code
├── requirements.txt            # Python dependencies
├── setup.sh                    # Setup script for submodules
├── wiki/                       # Documentation (git submodule)
│   ├── Home.md                 # Wiki index
│   ├── Hardware-Setup.md       # Hardware assembly guide
│   ├── Software-Installation.md # Software setup guide
│   ├── Troubleshooting.md      # Common issues and solutions
│   └── Technical-Specifications.md # Technical details
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

