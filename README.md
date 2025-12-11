# TurtleBot Hand Controller
**A gesture-based teleoperation system for TurtleBot using MediaPipe and ROS 2.**

## Project Overview
This project enables real-time control of a TurtleBot using hand gestures captured by a webcam. It decouples the computer vision processing (running natively on the host) from the robot control logic (running in a ROS 2 Docker container) to maximize performance and modularity.

## Tech Stack
* **Languages:** Python 3.10+, C++
* **Middleware:** ROS 2 Jazzy
* **Computer Vision:** MediaPipe, OpenCV
* **Data Serialization:** YAML (Inter-Process Communication)
* **Containerization:** Docker

## System Architecture
The system consists of two distinct components communicating via shared YAML state files:

1.  **Gesture Detector (Host Machine - Python):**
    * Uses **MediaPipe** for low-latency hand landmark detection.
    * Selected over YOLOv8 to reduce computational load and latency.
    * Runs natively on Windows/Linux to ensure maximum webcam frame rate.
    * Visualizes the control dashboard using OpenCV.

2.  **Robot Controller (ROS 2 Docker - C++):**
    * Reads motion commands from the shared YAML file.
    * Implements a **Safety Node** that overrides user input if LiDAR detects obstacles < 0.6m.
    * Publishes velocity commands (`/cmd_vel`) to the TurtleBot.

## System Architecture
This project utilizes a **hybrid development environment** to bridge Windows and Linux:

1.  **Host Machine (Windows 11 + Virtual Env):**
    * Runs the **Gesture Detector** in a Python virtual environment (`venv`).
    * This allows direct access to the webcam hardware and GPU for MediaPipe, bypassing Docker's complex USB pass-through limitations on Windows.
    * Writes command data to a shared `data/` volume.

2.  **Containerized Robot Control (Docker + ROS 2):**
    * Runs the **Robot Controller** inside a Linux Docker container (ROS 2 Jazzy).
    * Mounts the shared `data/` volume to read commands in real-time.
    * Ensures the robotics stack runs in a native-like Linux environment without dual-booting.

## Why this Architecture?
* **Hybrid Efficiency:** Leverages Windows for driver/peripheral support (Webcam) while keeping the ROS 2 core isolated in a stable Linux environment.
* **Dependency Management:** Using a local `venv` prevents version conflicts between MediaPipe (which requires specific Python versions) and the system Python.
* **Portability:** The ROS 2 node is fully containerized, making it easy to deploy to the actual TurtleBot hardware later.

## Usage

### 1. Start the Gesture Detector (Host)
```bash
# Requires Python 3.10+
pip install -r requirements.txt
python gesture_detector/gesture.py
```
### 2. Start the Robot Controller (Docker/ROS)
```bash

# Build the package
colcon build

source install/setup.bash

# Launch Controller + Safety Node
ros2 launch hand_controller start_controller.launch.py
```

## Control Scheme
Point Up (Index Only): Move Forward

Fist: Move Backward

Thumb Out (Hitchhiker): Rotate Left/Right (based on direction)

Open Hand: Stop


## Troubleshooting

### `colcon build` Fails on Virtual Environment
If you receive an error related to `setup.py` inside the `venv/` folder during the build process, it means ROS 2 is trying to compile your Python virtual environment libraries.

**Fix:** Tell `colcon` to ignore the virtual environment directory:
```bash

# Run this from the project root then run colcon build again
touch venv/COLCON_IGNORE