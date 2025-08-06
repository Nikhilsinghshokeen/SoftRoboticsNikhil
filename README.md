# Visual Gesture Control for Anthropomorphic Hand Gripper

This research uses a visual pipeline with MediaPipe and OpenCV to control a soft hand gripper. The goal is to create a controlled soft robotics hand that responds to hand movement that can be replicated easily. Further implications on the hand include using it as an attachment for a KINOVA robotic hand.

**Made by Nikhil Shokeen @ Worcester Polytechnic Institute Soft Robotics Lab (WPI)**  
**Mentor: Neehal Sharrma**

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation Guide](#installation-guide)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)

## Hardware Requirements

### Core Components
- **Arduino Uno** (or compatible board)
- **DC-DC Buck Converter** (for power regulation)
- **5 A20BHM Servo Motors** (for finger actuation)
- **16-Channel 12-bit PWM/Servo Driver** (PCA9685)

### Additional Components
- **Webcam** (USB camera for hand tracking)
- **Power Supply** (12V, 2A minimum)
- **Jumper Wires** (for connections)
- **Breadboard** (for prototyping)
- **3D Printed Hand Structure** (or equivalent gripper mechanism)

## Software Requirements

### Operating System
- **Ubuntu 18.04+** (recommended)
- **Windows 10/11** (with WSL2 for best compatibility)
- **macOS 10.15+**

### Python Version
- **Python 3.8+** (3.9 or 3.10 recommended)

## Installation Guide

### Step 1: System Preparation

#### Ubuntu/Debian
```bash
# Update package list
sudo apt update

# Install system dependencies
sudo apt install -y python3-pip python3-dev git curl

# Install additional system packages
sudo apt install -y libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1
```

#### Windows (WSL2)
```bash
# In WSL2 Ubuntu terminal
sudo apt update
sudo apt install -y python3-pip python3-dev git
```

#### macOS
```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python dependencies
brew install python3
```

### Step 2: Python Environment Setup

#### Option A: Using Virtual Environment (Recommended)
```bash
# Create virtual environment
python3 -m venv hand_tracking_env

# Activate virtual environment
# On Ubuntu/macOS:
source hand_tracking_env/bin/activate
# On Windows:
# hand_tracking_env\Scripts\activate

# Upgrade pip
pip install --upgrade pip
```

#### Option B: Global Installation
```bash
# Upgrade pip
pip3 install --upgrade pip
```

### Step 3: Install Python Libraries

#### Using requirements.txt (Recommended)
```bash
# Clone or download the project
git clone <repository-url>
cd MediaPipeWPI

# Install all dependencies
pip3 install -r requirements.txt
```

#### Manual Installation
```bash
# Install required Python libraries
pip3 install opencv-python==4.8.1.78
pip3 install mediapipe==0.10.7
pip3 install numpy==1.24.3
pip3 install pyserial==3.5
```

### Step 4: Arduino IDE Setup

#### Install Arduino IDE
```bash
# Ubuntu
sudo apt install arduino

# Or download from https://www.arduino.cc/en/software
```

#### Install Required Libraries
1. Open Arduino IDE
2. Go to **Tools** → **Manage Libraries**
3. Search and install:
   - **Adafruit PWM Servo Driver Library**
   - **Wire** (usually pre-installed)

### Step 5: System Permissions

#### Camera Access
```bash
# Add user to video group
sudo usermod -a -G video $USER
```

#### Serial Port Access (for Arduino communication)
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
```

**Important**: Log out and log back in for permission changes to take effect.

## Hardware Setup

### Servo Motor Connections
Connect the 5 servo motors to the PWM driver:

| Finger | Servo Driver Channel | Arduino Pin | Color Code |
|--------|---------------------|-------------|------------|
| Thumb  | Channel 0           | Pin 9       | Red        |
| Index  | Channel 1           | Pin 10      | Blue       |
| Middle | Channel 2           | Pin 11      | White      |
| Ring   | Channel 3           | Pin 12      | Green      |
| Pinky  | Channel 4           | Pin 13      | Black      |

### PWM Driver Connections
```
SCL (Clock)  → Arduino SCL pin (Red wire)
SDA (Data)   → Arduino SDA pin (Blue wire)
Data         → Arduino Digital Pin 7 (White wire)
GND          → Arduino GND (Green wire)
VCC          → 3.3V Power (Black wire)
```

### Power Supply
- Connect **12V power supply** to the DC-DC buck converter
- Set buck converter output to **5V** for servo motors
- Connect **5V output** to PWM driver VCC

## Software Setup

### Step 1: Arduino Code Upload
1. Connect Arduino Uno to computer via USB
2. Open Arduino IDE
3. Open `arduino_servo_control.ino` (or `robot_hand_arduino.ino`)
4. Select correct board: **Arduino Uno**
5. Select correct port (usually `/dev/ttyACM0` on Linux)
6. Click **Upload**

### Step 2: Camera Configuration
1. Connect webcam to USB port
2. Test camera access:
```bash
python3 test_camera.py
```

### Step 3: Serial Port Configuration
Check available serial ports:
```bash
ls /dev/tty*
```

Common ports:
- **Linux**: `/dev/ttyACM0` or `/dev/ttyUSB0`
- **Windows**: `COM3`, `COM4`, etc.
- **macOS**: `/dev/tty.usbmodem*`

## Usage

### Basic Operation
1. **Start the system**:
```bash
python3 hand_tracking_robot.py
```

2. **Position your hand** in front of the camera
3. **Make gestures** - the robot hand will mimic your movements
4. **Press 'q'** to quit

### Testing Individual Components

#### Test Camera Only
```bash
python3 test_camera.py
```

#### Test Arduino Communication
```bash
# Check if Arduino is detected
ls /dev/ttyACM*
```

### Calibration
The system includes automatic calibration, but you may need to adjust:
- **Camera index** in `hand_tracking_robot.py` (line 10)
- **Servo angle mapping** in the `send_to_arduino` function
- **Detection confidence** in MediaPipe settings

## Troubleshooting

### Common Issues

#### Camera Not Detected
```bash
# Check camera permissions
ls -l /dev/video*

# Test camera access
v4l2-ctl --list-devices

# Reinstall OpenCV if needed
pip3 uninstall opencv-python
pip3 install opencv-python
```

#### Arduino Not Connected
```bash
# Check serial ports
ls /dev/tty*

# Check user permissions
groups $USER

# Test serial communication
python3 -c "import serial; print('PySerial installed successfully')"
```

#### MediaPipe Issues
```bash
# Reinstall MediaPipe
pip3 uninstall mediapipe
pip3 install mediapipe==0.10.7

# Check GPU support (optional)
python3 -c "import mediapipe as mp; print(mp.__version__)"
```

#### Servo Movement Issues
1. Check power supply voltage (should be 5V)
2. Verify servo connections
3. Check PWM driver address (default: 0x40)
4. Test individual servos with Arduino IDE

### Performance Optimization
- **Reduce camera resolution** if experiencing lag
- **Adjust MediaPipe confidence thresholds**
- **Use USB 3.0** for better camera performance
- **Close unnecessary applications** to free up CPU

## Project Structure
```
MediaPipeWPI/
├── hand_tracking_robot.py      # Main application
├── test_camera.py              # Camera testing utility
├── arduino_servo_control.ino   # Arduino servo control code
├── robot_hand_arduino.ino      # Alternative Arduino code
├── robot_hand_control.ino      # Additional Arduino code
├── requirements.txt            # Python dependencies
├── README.md                   # This file
└── .gitignore                  # Git ignore file
```

## Technical Details

### MediaPipe Hand Tracking
- **21 landmarks** per hand
- **Real-time processing** at 30 FPS
- **Single hand detection** (configurable)
- **Joint angle calculation** for each finger

### Servo Control
- **PWM frequency**: 50Hz
- **Angle range**: 0-180 degrees
- **Smooth interpolation** between positions
- **Configurable mapping** for different hand sizes

### Communication Protocol
- **Serial communication** at 115200 baud
- **ASCII protocol**: `T:angle I:angle M:angle R:angle P:angle`
- **Update rate**: 10Hz (configurable)

## Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License
This project is developed at Worcester Polytechnic Institute (WPI) for research purposes.

## Acknowledgments
- **MediaPipe** team for hand tracking technology
- **Adafruit** for PWM servo driver library
- **OpenCV** community for computer vision tools
- **WPI Robotics Engineering Department**

---

**For technical support or questions, please contact:**
- **Nikhil Shokeen** - [shokeen24nikhil@gmail.com]
- **WPI Soft Robotics Lab**


