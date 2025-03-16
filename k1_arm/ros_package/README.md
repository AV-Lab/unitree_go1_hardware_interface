# K1 Arm Control - ROS Package

This ROS package provides control for the NXROBO Sagittarius K1 robotic arm using ROS topics and services. It integrates the external NXROBO Sagittarius SDK (included as a Git submodule) with our custom ROS nodes to deliver joint control, gripper operation, and an arm reset service.

---

## Overview

- Provides precise control of the K1 arm via ROS topics and services.
- Leverages NXROBO Sagittarius SDK (included as a submodule) with custom ROS nodes.
- Supports individual joint control, bulk commands, gripper operations, and arm reset.
- Integrates seamlessly with the Unitree Go1 Hardware Interface project.

![k1_arm_ros_demo](../assets/k1_arm_ros_demo.gif)

---

## Prerequisites

- Ubuntu 18.04 or 20.04
- ROS Noetic
- Eigen3 (`libeigen3-dev`)
- Boost libraries (`libboost-system-dev`, `libboost-thread-dev`)
- A working serial connection to the K1 arm (e.g., `/dev/ttyACM0`)

---

## Installation & Build Instructions

### 1. Clone the Repository with Submodules

Clone the repository into your desired location:

```bash
git clone --recursive https://github.com/YourOrg/unitree_go1_hardware_interface.git
```

If you cloned without submodules, update them with:

```bash
cd unitree_go1_hardware_interface/k1_arm/ros_package/catkin_ws/src
git submodule init
git submodule update
```

### 2. Build the Provided Catkin Workspace

The repository includes a ready-to-use catkin workspace. Navigate to it and build:

```bash
cd unitree_go1_hardware_interface/k1_arm/ros_package/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### Launching the Arm Control Node

To start the arm control node, run the launch file (which is part of the ROS package):

```bash
roslaunch sagittarius_control arm_control.launch port:=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with the appropriate serial port for your setup.

### Control Interfaces

#### Joint Control

- **Individual Joint Control:**  
  Each joint is controlled via dedicated topics (each accepting `std_msgs/Float64`), e.g.:

  - `/arm_control/base`
  - `/arm_control/shoulder`
  - `/arm_control/elbow`
  - `/arm_control/wrist_roll`
  - `/arm_control/wrist_pitch`
  - `/arm_control/wrist_yaw`

  Example:

  ```bash
  rostopic pub /arm_control/base std_msgs/Float64 "data: 0.5"
  ```

- **Bulk Joint Control:**  
  Publish an array of joint angles to `/arm_control/joint_angles` using a `std_msgs/Float64MultiArray` message.

  Example:

  ```bash
  rostopic pub /arm_control/joint_angles std_msgs/Float64MultiArray "data: [0.0, 1.4, -1.4, 0.0, 1.5, 0.0]"
  ```

#### Gripper Control

- **Services:**  
  Control the gripper using:

  - `/gripper/open` (`std_srvs/Trigger`)
  - `/gripper/close` (`std_srvs/Trigger`)

  Example:

  ```bash
  rosservice call /gripper/open
  rosservice call /gripper/close
  ```

#### Reset Service

- **Reset the Arm:**
  ```bash
  rosservice call /arm_control/reset
  ```

---

## Configuration

- **Port Configuration:**  
  The serial port is specified as a launch file argument (e.g., `port:=/dev/ttyACM0`).

- **Joint Limits & Control Rates:**  
  These parameters are set in the source code and can be adjusted as needed.

---

## Troubleshooting

1. **Permission Denied Errors:**  
   If you encounter permission issues with the serial device, add your user to the `dialout` group:

   ```bash
   sudo usermod -a -G dialout $USER
   newgrp dialout
   ```

2. **Device Not Found:**  
   Verify the device is connected:

   ```bash
   ls /dev/ttyACM*
   ```

3. **Build Errors:**  
   Ensure all required dependencies are installed:
   ```bash
   sudo apt-get install libeigen3-dev libboost-system-dev libboost-thread-dev
   ```

---

## Acknowledgments

- **NXROBO:** For providing the Sagittarius SDK.

---
