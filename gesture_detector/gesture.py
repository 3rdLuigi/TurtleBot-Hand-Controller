import cv2
import mediapipe as mp
import yaml
import time
import numpy as np
import os
import math

# --- CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
YAML_FILE = os.path.join(script_dir, "..", "data", "motion.yaml")

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
    """
    Returns dictionary of which fingers are OPEN.
    Logic: Is the TIP farther from wrist than the KNUCKLE?
    """
    fingers = {
        "index": (8, 6), 
        "middle": (12, 10), 
        "ring": (16, 14), 
        "pinky": (20, 18)
    }
    state = {}
    wrist = hand_landmarks.landmark[0]
    
    # 1. Check Index, Middle, Ring, Pinky
    for name, (tip_idx, pip_idx) in fingers.items():
        tip = hand_landmarks.landmark[tip_idx]
        pip = hand_landmarks.landmark[pip_idx]
        dist_tip = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
        dist_pip = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
        state[name] = dist_tip > dist_pip

    # 2. Check Thumb (Special Logic)
    # Thumb is "Open" if the tip is far away from the Index Finger Knuckle (MCP)
    thumb_tip = hand_landmarks.landmark[4]
    index_mcp = hand_landmarks.landmark[5]
    thumb_dist = math.hypot(thumb_tip.x - index_mcp.x, thumb_tip.y - index_mcp.y)
    
    # Threshold: 0.1 is a decent heuristic for normalized coordinates
    state["thumb"] = thumb_dist > 0.1
    
    return state

def get_thumb_direction(hand_landmarks):
    """
    Determines if thumb is pointing Left or Right.
    Compares Thumb Tip (4) vs Thumb IP Knuckle (3)
    """
    tip = hand_landmarks.landmark[4]
    ip = hand_landmarks.landmark[3]
    
    # Positive X is Right, Negative X is Left
    if tip.x < ip.x:
        return "ROTATE_LEFT"
    else:
        return "ROTATE_RIGHT"

def get_index_direction(hand_landmarks):
    """
    Determines direction based on Index Finger vector.
    """
    tip = hand_landmarks.landmark[8]
    mcp = hand_landmarks.landmark[5]
    
    dx = tip.x - mcp.x
    dy = tip.y - mcp.y
    
    if abs(dy) > abs(dx):
        return "MOVE_FORWARD" if dy < 0 else "MOVE_BACKWARD"
    else:
        return "MOVE_LEFT" if dx < 0 else "MOVE_RIGHT"

def draw_dashboard(height, active_command):
    dashboard = np.zeros((height, 300, 3), dtype=np.uint8)
    GREEN = (0, 255, 0)
    GRAY = (50, 50, 50)
    WHITE = (255, 255, 255)
    
    btns = {
        "MOVE_FORWARD":  (75, 20, 150, 60),
        "MOVE_LEFT":     (10, 100, 130, 60),
        "MOVE_RIGHT":    (160, 100, 130, 60),
        "MOVE_BACKWARD": (75, 180, 150, 60),
        "ROTATE_LEFT":   (10, 260, 130, 60),
        "ROTATE_RIGHT":  (160, 260, 130, 60),
        "STOP":          (75, 340, 150, 60)
    }
    
    for cmd, (x, y, w, h) in btns.items():
        color = GREEN if cmd == active_command else GRAY
        thickness = -1 if cmd == active_command else 2
        cv2.rectangle(dashboard, (x, y), (x + w, y + h), color, thickness)
        
        label = cmd.replace("MOVE_", "").replace("ROTATE_", "ROT ")
        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
        text_x = x + (w - text_size[0]) // 2
        text_y = y + (h + text_size[1]) // 2
        text_color = (0, 0, 0) if cmd == active_command else WHITE
        cv2.putText(dashboard, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
        
    return dashboard

# --- MAIN LOOP ---
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
                
                # Check Finger States
                fingers = get_finger_state(hand_landmarks)
                
                # Logic Gates
                # 1. Translation Mode: Index OPEN, Thumb CLOSED
                if fingers["index"] and not fingers["thumb"] and not fingers["middle"]:
                    current_command = get_index_direction(hand_landmarks)
                
                # 2. Rotation Mode: Thumb OPEN, Index CLOSED
                elif fingers["thumb"] and not fingers["index"] and not fingers["middle"]:
                    current_command = get_thumb_direction(hand_landmarks)
                
                # 3. Stop Mode: Both Closed (Fist) or Both Open (Confused)
                else:
                    current_command = "STOP"

        # Write to YAML
        if current_command != last_command:
            with open(YAML_FILE, 'w') as file:
                yaml.dump({'timestamp': time.time(), 'command': current_command}, file)
            print(f"Command Sent: {current_command}")
            last_command = current_command

        # Draw UI
        dashboard_img = draw_dashboard(h, current_command)
        final_ui = np.hstack((frame, dashboard_img))
        
        cv2.imshow('Gesture Control', final_ui)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()