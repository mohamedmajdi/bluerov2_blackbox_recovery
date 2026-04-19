
---

# BlueROV UJI 2025 - Autonomous Blackbox Recovery System

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%2FIron-blue)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)



## 🎥 Project Demo

  <p><em>Watch the BlueROV UJI 2025 autonomous blackbox recovery in action!</em></p>
  
  <a href="https://youtube.com/watch?v=F2rxVTdY1eY" target="_blank">
    <img src="https://img.youtube.com/vi/F2rxVTdY1eY/maxresdefault.jpg" 
         alt="BlueROV UJI 2025 Demo" 
         width="100%" 
         style="border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.3);">
  </a>

📹 Watch Full Demo →

## 🌊 Overview
**BlueROV_UJI2025** is a ROS 2 based software stack designed for the **BlueROV2 Heavy** configuration. Its primary mission is to autonomously locate, approach, and recover a "blackbox" object from a pool floor using a **Newton Gripper**.

The system employs a **semi-autonomous** architecture:
1.  **Deep Learning Vision:** Uses **YOLOv11** running on a topside computer to detect the blackbox and its handle in real-time.
2.  **Spatial Validation:** Ensures the detected handle is physically located within the blackbox boundaries to prevent false positives.
3.  **Human-in-the-Loop:** A pilot uses a gamepad or Web UI to switch the robot into "Approaching" mode once a target is confirmed.

## 🛠️ Hardware Requirements

* **Robot:** BlueROV2 (Heavy Configuration).
* **Manipulator:** Blue Robotics Newton Gripper.
* **Sensors:** Low-light USB Camera (or equivalent) for computer vision.
* **Topside Computer:**
    * Must have an **NVIDIA GPU** (Tested on RTX 4050).
    * Required for high-speed YOLOv11 inference.

## 📦 Software Architecture

The project is modularized into several ROS 2 packages:

| Package | Description |
| :--- | :--- |
| `bluerov2_bringup` | Launch files for initializing hardware, MAVROS connections, and parameters. |
| `bluerov2_vision` | Runs YOLOv11 detection nodes, spatial filtering, and visual odometry. |
| `bluerov2_controller` | PID controllers for depth, yaw, and visual servoing logic. |
| `bluerov2_search` | Implements search patterns (e.g., lawnmower) to find the target. |
| `bluerov2_teleop` | Maps Gamepad inputs (Xbox/Logitech) to ROV manual control. |
| `bluerov2_webui` | A web-based interface for mission monitoring and mode switching. |

## 🚀 Installation

### 1. Prerequisites
Ensure you have **ROS 2** (Humble or Iron recommended) and **Ubuntu 22.04** installed. You will also need the NVIDIA Container Toolkit or proper drivers if running detection locally.

### 2. Clone the Repository
```bash
mkdir -p ~/bluerov2_ws/src
cd ~/bluerov2_ws/src
git clone [https://github.com/your-org/BlueROV_UJI2025.git](https://github.com/your-org/BlueROV_UJI2025.git)

```

### 3. Install Dependencies

```bash
# Install system dependencies
cd ~/bluerov2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install Python requirements for Vision (YOLO)
pip3 install ultralytics opencv-python numpy PyYAML

```

### 4. Build the Workspace

```bash
cd ~/bluerov2_ws
colcon build --symlink-install
source install/setup.bash

```

## 🎮 How to Run

To operate the system, you will need to launch the components in separate terminals. Ensure you source the workspace (`source install/setup.bash`) in every terminal.

### Step 1: Bringup (Hardware & Internal Logic)

Initializes the robot parameters and internal state publishers.

```bash
ros2 launch bluerov2_bringup bluerov2_bringup.launch.py

```

### Step 2: Connect to Flight Controller (MAVROS)

Establishes the communication link between the onboard Raspberry Pi/Pixhawk and ROS 2.

```bash
ros2 launch bluerov2_bringup run_mavros.launch.py

```

### Step 3: Start Computer Vision

Launches the YOLOv11 detector on the topside GPU. This node publishes bounding boxes and target coordinates.

```bash
ros2 launch bluerov2_vision launch_detection-only.launch.py

```

### Step 4: Teleoperation

Enables manual control via a connected Gamepad.

```bash
ros2 launch bluerov2_teleop bluerov2_teleop.launch.py

```

### Step 5: Web User Interface

Starts the web server for monitoring the mission and toggling autonomous modes.

```bash
ros2 launch bluerov2_webui launch_webui.launch.py

```

## 🕹️ Controls & Workflow
1. **System Initialization & Deployment:** Human-controlled ROV deployment and system startup.
2. **Search Phase:** Pilot navigates manually or activates autonomous "Search Pattern" via Web UI.
3. **Detection & Validation:** The Vision node identifies the Blackbox and displays a bounding box on the Web UI video feed. The system verifies that the "Handle" is positioned inside the "Blackbox" region.
4. **Hybrid Visual Servoing:** Upon pilot confirmation of detection, the pilot toggles **"Approaching Mode"**. The ROV autonomously centers on the target using visual servoing and descends toward the Blackbox.
5. **Human-Confirmed Grasping:** The pilot confirms final positioning before the system executes autonomous gripper closure to attach the carabiner to the box.
6. **Surface Recovery:** Human-assisted retrieval of the ROV with attached Blackbox to the surface.

The joystick control can be found here ![Joystick defaults](https://github.com/MahmoudAboelrayat/bluerov2_blackbox_recovery/blob/main/joystick-defaults-1024x768.png):
