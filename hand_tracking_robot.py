#Code for Anthropogenic Soft Robotic Hand, visual pipeline to control hand using arduino
#Copyright (C) 2025  Nikhil Shokeen
import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math
import threading
import json
import os
from collections import deque

class HandTrackingRobot:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, camera_index=1):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.camera_index = camera_index
        self.cap = self.initialize_camera()
        
        self.arduino = None
        self.arduino_connected = False
        self.connect_arduino(port, baud_rate)
        
        self.angle_buffer = deque(maxlen=5)
        self.last_send_time = 0
        self.send_interval = 0.1
        
        self.running = False
        self.lock = threading.Lock()
        
        # Remove file-based calibration
        self.calibration_data = {}
        self.needs_calibration = True  # Always start with calibration
        self.calibration_step = 0  # 0: not started, 1: open hand, 2: closed hand, 3: complete
        self.open_hand_angles = None
        self.closed_hand_angles = None

    def initialize_camera(self):
        cap = cv2.VideoCapture(self.camera_index)
        
        if not cap.isOpened():
            print(f"Failed to open camera {self.camera_index}, trying alternative...")
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("Failed to open any camera")
                return None
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"Camera initialized successfully")
        return cap

    def connect_arduino(self, port, baud_rate):
        try:
            self.arduino = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)
            self.arduino_connected = True
            print(f"Arduino connected on {port}")
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.arduino_connected = False

    def calculate_distance(self, p1, p2):
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def calculate_angle(self, a, b, c):
        # a, b, c are (x, y) points
        a = np.array(a)
        b = np.array(b)
        c = np.array(c)
        ba = a - b
        bc = c - b
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
        return np.degrees(angle)

    def get_hand_gesture(self, landmarks, frame_shape):
        h, w, _ = frame_shape
        lm_list = []
        
        for lm in landmarks.landmark:
            cx, cy = int(lm.x * w), int(lm.y * h)
            lm_list.append([cx, cy])
        
        if len(lm_list) < 21:
            return None, lm_list
        
        thumb_tip = lm_list[4]
        index_tip = lm_list[8]
        middle_tip = lm_list[12]
        ring_tip = lm_list[16]
        pinky_tip = lm_list[20]
        
        wrist = lm_list[0]
        thumb_cmc = lm_list[1]
        thumb_mcp = lm_list[2]
        thumb_ip = lm_list[3]
        
        index_mcp = lm_list[5]
        index_pip = lm_list[6]
        index_dip = lm_list[7]
        
        middle_mcp = lm_list[9]
        middle_pip = lm_list[10]
        middle_dip = lm_list[11]
        
        ring_mcp = lm_list[13]
        ring_pip = lm_list[14]
        ring_dip = lm_list[15]
        
        pinky_mcp = lm_list[17]
        pinky_pip = lm_list[18]
        pinky_dip = lm_list[19]
        
        distances = {
            'thumb': self.calculate_distance(thumb_tip, thumb_mcp),
            'index': self.calculate_distance(index_tip, index_mcp),
            'middle': self.calculate_distance(middle_tip, middle_mcp),
            'ring': self.calculate_distance(ring_tip, ring_mcp),
            'pinky': self.calculate_distance(pinky_tip, pinky_mcp)
        }
        
        # Calculate joint angles for each finger
        angles = {
            'thumb': {
                'MCP': self.calculate_angle(thumb_cmc, thumb_mcp, thumb_ip),
                'IP': self.calculate_angle(thumb_mcp, thumb_ip, thumb_tip)
            },
            'index': {
                'MCP': self.calculate_angle(wrist, index_mcp, index_pip),
                'PIP': self.calculate_angle(index_mcp, index_pip, index_dip),
                'DIP': self.calculate_angle(index_pip, index_dip, index_tip)
            },
            'middle': {
                'MCP': self.calculate_angle(wrist, middle_mcp, middle_pip),
                'PIP': self.calculate_angle(middle_mcp, middle_pip, middle_dip),
                'DIP': self.calculate_angle(middle_pip, middle_dip, middle_tip)
            },
            'ring': {
                'MCP': self.calculate_angle(wrist, ring_mcp, ring_pip),
                'PIP': self.calculate_angle(ring_mcp, ring_pip, ring_dip),
                'DIP': self.calculate_angle(ring_pip, ring_dip, ring_tip)
            },
            'pinky': {
                'MCP': self.calculate_angle(wrist, pinky_mcp, pinky_pip),
                'PIP': self.calculate_angle(pinky_mcp, pinky_pip, pinky_dip),
                'DIP': self.calculate_angle(pinky_pip, pinky_dip, pinky_tip)
            }
        }
        # Print angles for debugging/inspection
        print('Joint Angles:', angles)
        
        return distances, lm_list
#changes to the camera to make it more robust for the hand tracking
    def preprocess_frame(self, frame):
        # Convert to YCrCb to reduce lighting effects
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        # Histogram equalization on the Y channel
        ycrcb[:,:,0] = cv2.equalizeHist(ycrcb[:,:,0])
        frame_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
        # Optionally, apply CLAHE for local contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        lab = cv2.cvtColor(frame_eq, cv2.COLOR_BGR2LAB)
        lab[:,:,0] = clahe.apply(lab[:,:,0])
        frame_clahe = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        # Optionally, blur to reduce bloom
        frame_blur = cv2.GaussianBlur(frame_clahe, (3,3), 0)
        return frame_blur

    def map_distance_to_servo(self, distance, min_dist=30, max_dist=150):
        servo_angle = np.interp(distance, [min_dist, max_dist], [0, 180])
        return int(np.clip(servo_angle, 0, 180))

    def map_angle(self, angle, min_angle, max_angle):
        # Clamp and map the angle from [min_angle, max_angle] to [min_angle, max_angle]
        # (identity if in range, but clamps if out of range)
        return int(np.clip(angle, min_angle, max_angle))

    def start_calibration(self):
        """Start the calibration process"""
        self.calibration_step = 1
        self.open_hand_angles = None
        self.closed_hand_angles = None
        print("Calibration started! Show your OPEN hand and press 'o' to record.")
    
    def record_open_hand(self, finger_angles):
        """Record the open hand angles (maximum)"""
        self.open_hand_angles = finger_angles.copy()
        self.calibration_step = 2
        print("Open hand recorded! Now show your CLOSED hand and press 'c' to record.")
    
    def record_closed_hand(self, finger_angles):
        """Record the closed hand angles (minimum)"""
        self.closed_hand_angles = finger_angles.copy()
        self.calibration_step = 3
        self.needs_calibration = False
        print("Calibration complete! Hand tracking is now active.")
    
    def get_calibrated_servo_value(self, finger, current_angle):
        """Map current angle to servo value using calibration data"""
        if not self.open_hand_angles or not self.closed_hand_angles:
            return 90  # Default middle position if not calibrated
        
        open_angle = self.open_hand_angles[finger]
        closed_angle = self.closed_hand_angles[finger]
        
        # Map current angle between open and closed to servo 0-180
        if open_angle != closed_angle:
            servo_value = np.interp(current_angle, [closed_angle, open_angle], [0, 180])
            return int(np.clip(servo_value, 0, 180))
        else:
            return 90  # Default if angles are the same

    def send_to_arduino(self, finger_angles):
        if not self.arduino_connected or not self.arduino:
            return
        current_time = time.time()
        if current_time - self.last_send_time < self.send_interval:
            return
        try:
            # Use calibrated values if available, otherwise use default mapping
            if self.needs_calibration:
                # During calibration, use default mapping
                thumb = int(np.interp(finger_angles['thumb'], [100, 180], [0, 180]))
                index = int(np.interp(finger_angles['index'], [80, 180], [0, 180]))
                middle = int(np.interp(finger_angles['middle'], [80, 180], [0, 180]))
                ring = int(np.interp(finger_angles['ring'], [40, 180], [0, 180]))
                pinky = int(np.interp(finger_angles['pinky'], [30, 180], [0, 180]))
            else:
                # Use calibrated mapping
                thumb = int(self.get_calibrated_servo_value('thumb', finger_angles['thumb']))
                index = int(self.get_calibrated_servo_value('index', finger_angles['index']))
                middle = int(self.get_calibrated_servo_value('middle', finger_angles['middle']))
                ring = int(self.get_calibrated_servo_value('ring', finger_angles['ring']))
                pinky = int(self.get_calibrated_servo_value('pinky', finger_angles['pinky']))
            
            data = f"T:{thumb} I:{index} M:{middle} R:{ring} P:{pinky}\n"
            self.arduino.write(data.encode())
            self.arduino.flush()
            self.last_send_time = current_time
        except Exception as e:
            print(f"Error sending to Arduino: {e}")
            self.arduino_connected = False

    def draw_hand_info(self, frame, distances, servo_values, lm_list, angles=None):
        if not lm_list:
            return
        
        thumb_tip = lm_list[4]
        index_tip = lm_list[8]
        middle_tip = lm_list[12]
        ring_tip = lm_list[16]
        pinky_tip = lm_list[20]
        
        cv2.circle(frame, tuple(thumb_tip), 10, (255, 0, 0), -1)
        cv2.circle(frame, tuple(index_tip), 10, (0, 255, 0), -1)
        cv2.circle(frame, tuple(middle_tip), 10, (0, 0, 255), -1)
        cv2.circle(frame, tuple(ring_tip), 10, (255, 255, 0), -1)
        cv2.circle(frame, tuple(pinky_tip), 10, (255, 0, 255), -1)
        
        y_pos = 30
        if angles:
            for finger, joints in angles.items():
                text = f"{finger.capitalize()}: "
                text += ", ".join([f"{joint}: {int(angle)}" for joint, angle in joints.items() if not np.isnan(angle)])
                cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                y_pos += 25

    def run(self):
        if not self.cap or not self.cap.isOpened():
            print("Error: Camera not available")
            return
        self.running = True
        
        if self.needs_calibration:
            print("Calibration needed! Press 'c' to start calibration, 'q' to quit.")
            self.show_calibration_screen()
        else:
            print("Hand tracking started. Press 'q' to quit, 'r' to recalibrate.")
        
        while self.running:
            success, frame = self.cap.read()
            if not success:
                print("Failed to read frame, retrying...")
                time.sleep(0.1)
                continue
                
            frame = self.preprocess_frame(frame)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    distances, lm_list = self.get_hand_gesture(hand_landmarks, frame.shape)
                    if distances is not None and lm_list:
                        # Calculate joint angles for all fingers
                        angles = {
                            'thumb': {
                                'MCP': self.calculate_angle(lm_list[1], lm_list[2], lm_list[3]),
                                'IP': self.calculate_angle(lm_list[2], lm_list[3], lm_list[4])
                            },
                            'index': {
                                'MCP': self.calculate_angle(lm_list[0], lm_list[5], lm_list[6]),
                                'PIP': self.calculate_angle(lm_list[5], lm_list[6], lm_list[7]),
                                'DIP': self.calculate_angle(lm_list[6], lm_list[7], lm_list[8])
                            },
                            'middle': {
                                'MCP': self.calculate_angle(lm_list[0], lm_list[9], lm_list[10]),
                                'PIP': self.calculate_angle(lm_list[9], lm_list[10], lm_list[11]),
                                'DIP': self.calculate_angle(lm_list[10], lm_list[11], lm_list[12])
                            },
                            'ring': {
                                'MCP': self.calculate_angle(lm_list[0], lm_list[13], lm_list[14]),
                                'PIP': self.calculate_angle(lm_list[13], lm_list[14], lm_list[15]),
                                'DIP': self.calculate_angle(lm_list[14], lm_list[15], lm_list[16])
                            },
                            'pinky': {
                                'MCP': self.calculate_angle(lm_list[0], lm_list[17], lm_list[18]),
                                'PIP': self.calculate_angle(lm_list[17], lm_list[18], lm_list[19]),
                                'DIP': self.calculate_angle(lm_list[18], lm_list[19], lm_list[20])
                            }
                        }
                        
                        # Weighted sum for each finger (more realistic curl)
                        finger_angles = {}
                        for finger in angles:
                            # Weights: MCP=0.5, PIP=0.3, DIP=0.2 (tune as needed)
                            if finger == 'thumb':
                                mcp = angles[finger].get('MCP', 0)
                                ip = angles[finger].get('IP', 0)
                                finger_angles[finger] = 0.6 * mcp + 0.4 * ip
                            else:
                                mcp = angles[finger].get('MCP', 0)
                                pip = angles[finger].get('PIP', 0)
                                dip = angles[finger].get('DIP', 0)
                                finger_angles[finger] = 0.5 * mcp + 0.3 * pip + 0.2 * dip
                        
                        # Draw hand landmarks
                        self.mp_drawing.draw_landmarks(
                            frame,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS
                        )
                        
                        # Handle calibration or normal operation
                        if self.needs_calibration:
                            self.handle_calibration_mode(frame, finger_angles)
                        else:
                            self.draw_hand_info(frame, distances, finger_angles, lm_list, angles)
                            self.send_to_arduino(finger_angles)
            
            # Add calibration instructions to frame
            if self.needs_calibration:
                self.draw_calibration_instructions(frame)
            else:
                self.draw_normal_instructions(frame)
            
            cv2.imshow('Hand Tracking Robot', frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.running = False
                break
            elif key == ord('c'):
                if self.calibration_step == 0:
                    # Start calibration
                    self.start_calibration()
                elif self.calibration_step == 2:
                    # Record closed hand
                    self.record_closed_hand(finger_angles)
            elif key == ord('o') and self.calibration_step == 1:
                # Record open hand
                self.record_open_hand(finger_angles)
            elif key == ord('r') and not self.needs_calibration:
                # Recalibrate
                self.needs_calibration = True
                self.calibration_step = 0
                print("Recalibration started! Press 'c' to begin.")
        
        self.cap.release()
        cv2.destroyAllWindows()
        print("Hand tracking stopped.")

    def show_calibration_screen(self):
        """Show initial calibration screen"""
        print("\n" + "="*50)
        print("CALIBRATION REQUIRED")
        print("="*50)
        print("This program needs to learn your hand positions.")
        print("Press 'c' to start calibration when ready.")
        print("You will need to:")
        print("1. Open your hand and hold it steady")
        print("2. Close your hand and hold it steady")
        print("="*50)

    def handle_calibration_mode(self, frame, finger_angles):
        """Handle calibration mode display and logic"""
        if self.calibration_step == 1:
            # Waiting for open hand
            cv2.putText(frame, "Show OPEN hand and press 'o'", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Current angles: {finger_angles}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        elif self.calibration_step == 2:
            # Waiting for closed hand
            cv2.putText(frame, "Show CLOSED hand and press 'c'", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Open hand recorded: {self.open_hand_angles}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Current angles: {finger_angles}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def draw_calibration_instructions(self, frame):
        """Draw calibration instructions on frame"""
        cv2.putText(frame, "CALIBRATION MODE", (10, frame.shape[0] - 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        if self.calibration_step == 0:
            cv2.putText(frame, "Press 'c' to start calibration", (10, frame.shape[0] - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        elif self.calibration_step == 1:
            cv2.putText(frame, "Press 'o' to record open hand", (10, frame.shape[0] - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        elif self.calibration_step == 2:
            cv2.putText(frame, "Press 'c' to record closed hand", (10, frame.shape[0] - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    def draw_normal_instructions(self, frame):
        """Draw normal operation instructions on frame"""
        cv2.putText(frame, "Press 'r' to recalibrate", (10, 400), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, "Press 'q' to quit", (10, 430), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

if __name__ == "__main__":
    robot = HandTrackingRobot()
    try:
        robot.run()
    except KeyboardInterrupt:
        print("Hand tracking stopped.") 
