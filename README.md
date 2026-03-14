# Computer Vision Controlled Robot Arm with Hand Tracking

A 6-DOF robotic arm equipped with 6 servo motors and controlled by an ESP32 WiFi processor. The project demonstrates full custom implementation of inverse kinematics (IK) from scratch, including reachability analysis and real-time servo control, followed by integration with computer vision for hand-tracking control.

## Project Goals

1. **Inverse Kinematics Implementation**  
   Develop all software from scratch to solve inverse kinematics for a 2-link planar robotic arm (L₁ = 4 cm, L₂ = 12 cm). The solution calculates all reachable coordinates and automatically orients the arm to any specified (X, Y) position.

2. **Computer Vision Integration**  
   Extend the IK system with real-time hand tracking via webcam. The tip of the index finger is tracked and mapped to robot coordinates, enabling direct gesture-based control of the physical arm.

Both goals were successfully achieved.

## Features

- Full inverse kinematics solver supporting elbow-up and elbow-down configurations  
- Reachability checking in the upper half-plane using the law of cosines  
- Real-time angle remapping for servo compatibility (θ₁: 0°–180°, θ₂: −90° to +90°)  
- Browser-based simulation with live canvas rendering and workspace visualization  
- Computer vision control using MediaPipe hand tracking and OpenCV  
- Live visualization window showing arm pose, workspace, and coordinate grid  
- Automatic serial communication to the ESP32 (115200 baud)  
- Visual feedback: green dot for reachable positions, red dot for out-of-bounds  

## Technologies Used

- **Simulation & Control Interface**: Plain JavaScript (no external libraries) with the Web Serial API  
- **Computer Vision**: Python, OpenCV, MediaPipe, Tkinter, Matplotlib  
- **Microcontroller**: ESP32 programmed via Arduino IDE  
- **Hardware**: 6-DOF robotic arm with 6 servo motors  

## Demonstrations:

### Hand Tracking Control:
### Video Demonstration (Click to Watch!)
### Full Video Demonstration
[![Watch the full demonstration on YouTube](https://img.youtube.com/vi/3YrJATeAfHo/maxresdefault.jpg)](https://youtu.be/3YrJATeAfHo)

### Inverse Kinematics Simulation:
### Video Demonstration (Click to Watch!)
[](https://img.youtube.com/vi/xQjae3qHbV8/maxresdefault.jpg)](https://youtu.be/xQjae3qHbV8)

## How It Works

### Goal 1: Inverse Kinematics (HTML Simulation)
The `HTML_Simulation.html` file implements a complete 2-link IK solver in plain JavaScript.  
- Users can click anywhere on the canvas or enter X/Y coordinates.  
- The algorithm first checks reachability using the law of cosines to compute cos θ₂.  
- Both elbow-up and elbow-down solutions are calculated via `atan2`.  
- The first valid solution that keeps θ₁ within [0°, 180°] is selected.  
- The reachable workspace is drawn as green dots. Axes, the blue arm, black joints, red angle arcs, and dashed labels are rendered in real time.  
- Angles are remapped for servos and sent via the browser Serial API (Chrome) to the ESP32, allowing the physical arm to mirror the simulation instantly.

### Goal 2: Computer Vision Control (Python)
The `Webcam_HandTracking.py` script combines hand tracking with the same IK engine:  
- OpenCV captures the webcam feed and overlays a coordinate grid.  
- MediaPipe detects the index fingertip position.  
- The fingertip coordinates are mapped to the robot’s X/Y space.  
- Inverse kinematics is solved for θ₁ and θ₂.  
- A Tkinter + Matplotlib window displays the arm pose, workspace, and live values.  
- If angles change beyond a threshold, serial commands (“M3 θ₂; M2 θ₁;”) are sent to the ESP32 on COM9 at 115200 baud.  
- The fingertip marker is green when inside the reachable workspace and red when outside.

## Setup and Installation

1. Open the Arduino IDE.  
2. Load the file `ESP32_code_smooth_movement.ino` and upload it to your ESP32 board.  

**Option A – Inverse Kinematics Simulation**  
- Open `HTML_Simulation.html` in Google Chrome.  
- Click “Connect” to pair with the ESP32 via the browser Serial API.  

**Option B – Webcam Hand Tracking**  
- Ensure Python 3 and the required packages (`opencv-python`, `mediapipe`, `matplotlib`, `tkinter`) are installed.  
- Run the script:  
  ```bash
  python Webcam_HandTracking.py
