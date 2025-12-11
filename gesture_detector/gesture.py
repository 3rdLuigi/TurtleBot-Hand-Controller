import cv2
import mediapipe as mp
import yaml
import time
import numpy as np
import os
import math

# --- CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
MOTION_FILE = os.path.join(script_dir, "..", "data", "motion.yaml")
SAFETY_FILE = os.path.join(script_dir, "..", "data", "safety.yaml")


# --- SETUP MEDIAPIPE ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.5, 
    max_num_hands=1
    )

cap = cv2.VideoCapture(0)
last_command = "STOP"

# --- HELPER FUNCTIONS ---
def get_finger_state(hand_landmarks):
    #Returns dictionary of which fingers are OPEN.
    fingers = {
        "index": (8, 6), 
        "middle": (12, 10), 
        "ring": (16, 14), 
        "pinky": (20, 18)
    }
    state = {}
    wrist = hand_landmarks.landmark[0]

    for name, (tip_idx, pip_idx) in fingers.items():
        tip = hand_landmarks.landmark[tip_idx]
        pip = hand_landmarks.landmark[pip_idx]
        dist_tip = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
        dist_pip = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
        state[name] = dist_tip > dist_pip

    # Thumb logic
    thumb_tip = hand_landmarks.landmark[4]
    index_mcp = hand_landmarks.landmark[5]
    thumb_dist = math.hypot(thumb_tip.x - index_mcp.x, thumb_tip.y - index_mcp.y)

    state["thumb"] = thumb_dist > 0.1
    
    return state

def get_thumb_direction(hand_landmarks):
    tip = hand_landmarks.landmark[4]
    ip = hand_landmarks.landmark[3]
    return "ROTATE_LEFT" if tip.x < ip.x else "ROTATE_RIGHT"

def read_safety_status():
    # Reads the safety file to check for obstacles
    try:
        with open(SAFETY_FILE, 'r') as f:
            data = yaml.safe_load(f)
            return data.get('status', 'ALLOWED')
    except:
        return 'ALLOWED' # Default to safe if file read fails

def draw_dashboard(height, active_command, safety_status):
    dashboard = np.zeros((height, 300, 3), dtype=np.uint8)
    
    # COLORS
    GREEN = (0, 255, 0)
    RED   = (0, 0, 255)
    GRAY  = (50, 50, 50)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    
    # SIMPLIFIED BUTTON LAYOUT
    btns = {
        "MOVE_FORWARD":  (75, 50, 150, 60),  # Point Up
        "MOVE_BACKWARD": (75, 150, 150, 60), # Fist
        "ROTATE_LEFT":   (10, 250, 130, 60), # Thumb Left
        "ROTATE_RIGHT":  (160, 250, 130, 60),# Thumb Right
        "STOP":          (75, 350, 150, 60)  # Open Hand
    }
    
    for cmd, (x, y, w, h) in btns.items():
            # Default State
            color = GRAY
            thickness = 2
            text_color = WHITE

            # SPECIAL CASE: Forward is Blocked
            if cmd == "MOVE_FORWARD" and safety_status == "BLOCKED":
                color = RED
                thickness = -1 # Filled Red to show danger
                text_color = BLACK
            
            # STANDARD CASE: Button is Active (and not blocked)
            elif cmd == active_command:
                color = GREEN
                thickness = -1 # Filled Green
                text_color = BLACK

            cv2.rectangle(dashboard, (x, y), (x + w, y + h), color, thickness)
            
            label = cmd.replace("MOVE_", "").replace("ROTATE_", "ROT ")
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            text_x = x + (w - text_size[0]) // 2
            text_y = y + (h + text_size[1]) // 2
            cv2.putText(dashboard, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)

    # WARNING MESSAGE
    if safety_status == "BLOCKED":
        cv2.putText(dashboard, "OBSTACLE DETECTED!", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)
        cv2.putText(dashboard, "FORWARD DISABLED", (35, 450), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)
    return dashboard

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)
        
        current_command = "STOP"

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                fingers = get_finger_state(hand_landmarks)
                
                
                # FORWARD (Index Open, Middle Closed)
                # Pointing Up
                if fingers["index"] and not fingers["middle"]:
                    current_command = "MOVE_FORWARD"

                # ROTATION (Thumb Open AND Index Closed)
                elif fingers["thumb"] and not fingers["index"]:
                    current_command = get_thumb_direction(hand_landmarks)

                # 3. BACKWARD (Fist)
                # All fingers closed
                elif not fingers["index"] and not fingers["middle"] and not fingers["thumb"]:
                    current_command = "MOVE_BACKWARD"

                # 4. STOP (Everything else, including Open Hand)
                else:
                    current_command = "STOP"

        # Read safety status
        safety_status = read_safety_status()

        # Write to motion.yaml
        if current_command != last_command:
            with open(MOTION_FILE, 'w') as file:
                yaml.dump({'timestamp': time.time(), 'command': current_command}, file)
            print(f"Command Sent: {current_command}")
            last_command = current_command

        # Draw UI
        dashboard_img = draw_dashboard(h, current_command, safety_status)
        final_ui = np.hstack((frame, dashboard_img))
        
        cv2.imshow('Gesture Control', final_ui)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()