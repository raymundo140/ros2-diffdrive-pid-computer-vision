# ROS 2 Differential-Drive Robot with PID Control and Traffic Light Detection

This project implements a ROS 2-based PID control system for a differential-drive robot that navigates a predefined path while responding to traffic light signals detected through computer vision.

## 🧠 Project Overview

- **Platform:** ROS 2 Humble on Ubuntu 22.04
- **Hardware:** Jetson Nano, Raspberry Pi Camera, motors with encoders
- **Key Features:**
  - Real-time traffic light detection using OpenCV (HSV masks)
  - Closed-loop control using PID for position and orientation
  - Modular ROS 2 nodes for perception, control, and odometry
  - State machine for reliable behavior transitions


## 🚦 Traffic Light Detection

The system uses HSV thresholding to detect:
- Red (`(0–10)` and `(170–180)` Hue)
- Yellow (`(18–30)` Hue)
- Green (`(40–80)` Hue)

Detection is published to `/light_state` as a string: `"red"`, `"yellow"`, or `"green"`.

## 📍 PID Control Logic

The robot follows a list of goals in the format `[x, y, θ]` using:
- **Linear PID:** To minimize position error
- **Angular PID:** To align robot heading
- **FSM:** Manages WAITING, MOVING, ROTATING states

## 🛠️ How to Run

1. Build the workspace:

```bash
colcon build
source install/setup.bash
```

2. Run each node (in separate terminals):
```bash
ros2 run halfterm traffic_light_detector.py
ros2 run halfterm dead_reckoning.py
ros2 run halfterm traffic_pid_controller.py
```

