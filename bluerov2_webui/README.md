## TODO
* add manual stop (verify which topic name)
* add gripper button (verify which topic name)
* Add more..?

# BlueROV2 WebUI Control Station
This package is currently under active development. It is designed to replace `rqt` for day-to-day testing and make teleoperation tuning easier. (Mahmoud ask for it)

## Overview
This is a lightweight, browser-based dashboard for the BlueROV2 built on **ROS2** and **NiceGUI**. It provides a unified interface for:
* Video streaming (Raw vs. Processed/Detection feeds).
* Teleoperation mode switching.
* Live topic echoing (Multi-window).
* Dynamic parameter reconfiguration (Node Inspector).

## Prerequisites
Ensure you have the standard ROS2 desktop packages installed (including `cv_bridge`).

You must install the UI framework:
```bash
pip install nicegui
````

## Installation

Build the package in your ROS2 workspace:

```bash
cd ~/your_ws
colcon build --packages-select bluerov2_webui
source install/setup.bash
```

## Usage

Launch the web server and ROS node:

```bash
ros2 launch bluerov2_webui launch_webui.launch.py
```

Once running, open your browser (usually auto-opens) to:

> **http://localhost:8080**

## Features Guide

### 1\. Top Bar (Teleop)

  * **Mode Selector:** Quickly switch between `MANUAL`, `SERVOING`, `CORRECTION`, and `SEARCHING`.
  * **Status:** Shows connection status.

### 2\. Left Panel (Topic Inspector)

  * **Split View:** Two independent echo terminals (Top/Bottom) to compare topics.
  * **Table Mode:** Toggle the "Table" switch to see data in a clean Field/Value format.
  * **Filtering:** Click the `Settings` (Gear icon) to select specific fields to watch (e.g., just `x` and `y` from Odometry).
  * *Note: Do not try to echo Camera images directly in the text view.*

### 3\. Center Panel (Video)

  * **Toggle Switch:** Switch between the `/camera/image` (Raw) and `camera_detections` (Processed) feeds.

### 4\. Right Panel (Node Inspector)

  * **Dynamic Tabs:** Click `+` to open multiple inspector tabs.
  * **Parameter Tuning:** Select any running ROS2 node to auto-generate sliders and switches for its parameters. Changes are sent live to the robot.

