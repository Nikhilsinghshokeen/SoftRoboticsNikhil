import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math
import threading
from collections import deque

class HandTrackingRobot:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, camera_index=0):
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

    def map_distance_to_servo(self, distance, min_dist=30, max_dist=150):
        servo_angle = np.interp(distance, [min_dist, max_dist], [0, 180])
        return int(np.clip(servo_angle, 0, 180))

    def send_to_arduino(self, servo_values):
        if not self.arduino_connected or not self.arduino:
            return
        
        current_time = time.time()
        if current_time - self.last_send_time < self.send_interval:
            return
        
        try:
            data = f"T:{servo_values['thumb']},I:{servo_values['index']},M:{servo_values['middle']},R:{servo_values['ring']},P:{servo_values['pinky']}\n"
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
        for finger, (distance, servo) in zip(['Thumb', 'Index', 'Middle', 'Ring', 'Pinky'], 
                                           zip(distances.values(), servo_values.values())):
            cv2.putText(frame, f"{finger}: {servo}°", (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_pos += 25
        
        # Display joint angles if provided
        if angles:
            y_pos += 10
            for finger, joints in angles.items():
                for joint, angle in joints.items():
                    cv2.putText(frame, f"{finger} {joint}: {angle:.1f}°", (10, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    y_pos += 18

    def run(self):
        if not self.cap or not self.cap.isOpened():
            print("Error: Camera not available")
            return
        
        self.running = True
        print("Hand tracking started. Press 'q' to quit.")
        
        while self.running:
            success, frame = self.cap.read()
            if not success:
                print("Failed to read frame, retrying...")
                time.sleep(0.1)
                continue
            
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )
                    
                    distances, lm_list = self.get_hand_gesture(hand_landmarks, frame.shape)
                    
                    if distances:
                        # Calculate angles again for display
                        h, w, _ = frame.shape
                        lm_list_display = []
                        for lm in hand_landmarks.landmark:
                            cx, cy = int(lm.x * w), int(lm.y * h)
                            lm_list_display.append([cx, cy])
                        if len(lm_list_display) == 21:
                            wrist = lm_list_display[0]
                            thumb_cmc = lm_list_display[1]
                            thumb_mcp = lm_list_display[2]
                            thumb_ip = lm_list_display[3]
                            thumb_tip = lm_list_display[4]
                            index_mcp = lm_list_display[5]
                            index_pip = lm_list_display[6]
                            index_dip = lm_list_display[7]
                            index_tip = lm_list_display[8]
                            middle_mcp = lm_list_display[9]
                            middle_pip = lm_list_display[10]
                            middle_dip = lm_list_display[11]
                            middle_tip = lm_list_display[12]
                            ring_mcp = lm_list_display[13]
                            ring_pip = lm_list_display[14]
                            ring_dip = lm_list_display[15]
                            ring_tip = lm_list_display[16]
                            pinky_mcp = lm_list_display[17]
                            pinky_pip = lm_list_display[18]
                            pinky_dip = lm_list_display[19]
                            pinky_tip = lm_list_display[20]
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
                        else:
                            angles = None
                        servo_values = {
                            'thumb': self.map_distance_to_servo(distances['thumb']),
                            'index': self.map_distance_to_servo(distances['index']),
                            'middle': self.map_distance_to_servo(distances['middle']),
                            'ring': self.map_distance_to_servo(distances['ring']),
                            'pinky': self.map_distance_to_servo(distances['pinky'])
                        }
                        
                        with self.lock:
                            self.send_to_arduino(servo_values)
                            self.draw_hand_info(frame, distances, servo_values, lm_list, angles)
            else:
                cv2.putText(frame, "No hand detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow('Hand Tracking Robot', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cleanup()

    def cleanup(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        if self.arduino and self.arduino_connected:
            self.arduino.close()
        print("Hand tracking stopped.")

if __name__ == "__main__":
    robot = HandTrackingRobot()
    try:
        robot.run()
    except KeyboardInterrupt:
        robot.cleanup() 