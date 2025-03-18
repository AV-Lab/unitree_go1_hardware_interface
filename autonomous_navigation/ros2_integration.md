# ROS2 Web Portal Integration

This document describes how the Unitree Go1 Hardware Interface leverages ROS2 for low-level control and was previously tested with a web-based portal for remote monitoring and high-level control. This method was evaluated in earlier tests but is not the current active approach.

## ![autoeye_go1_ros2_demo](../assets/autoeye_go1_ros2_demo.gif)

---

## Overview

The Unitree Go1 Edu robot is primarily supported by official ROS1 tools; however, for enhanced functionality and compatibility with ROS2, two approaches have been explored:

- **Low-Level ROS2 Bridge:**  
  The [`unitree_ros2_to_real`](https://github.com/unitreerobotics/unitree_ros2_to_real) package provides basic low-level control (e.g., direct motor commands) via ROS2. This method, however, lacks high-level abstractions required for navigation and complex behaviors.

- **Enhanced ROS2 Support with unitree_ros:**  
  The [`unitree_ros`](https://docs.ros.org/en/iron/p/unitree_ros/) package acts as middleware between ROS2 and the unitree_legged_sdk. It enables features such as:
  - Velocity control
  - Robot state feedback (including odometry and IMU data)
  - High-level commands (e.g., stand up/down, head LED status)

### Installation & Usage of `unitree_ros`

1. **Install the Package:**

   ```bash
   sudo apt install ros-[distro]-unitree-ros
   ```

2. **Clone the Repository:**

   ```bash
   mkdir -p ~/unitree_ws/src
   cd ~/unitree_ws/src
   git clone --recurse-submodules https://github.com/snt-arg/unitree_ros.git
   ```

3. **Build the Package:**

   ```bash
   cd ~/unitree_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Launch the Driver:**
   ```bash
   ros2 launch unitree_ros unitree_driver_launch.py
   ```

### Topic Details

**Subscribed Topics:**

| Topic Name    | Message Type              | Description        |
| ------------- | ------------------------- | ------------------ |
| `cmd_vel`     | `geometry_msgs/msg/Twist` | Velocity control   |
| `/stand_up`   | `std_msgs/msg/Empty`      | Stand up command   |
| `/stand_down` | `std_msgs/msg/Empty`      | Stand down command |

**Published Topics:**

| Topic Name   | Message Type               | Description     |
| ------------ | -------------------------- | --------------- |
| `/odom`      | `nav_msgs/msg/Odometry`    | Odometry data   |
| `/imu`       | `sensor_msgs/msg/Imu`      | IMU sensor data |
| `/bms_state` | `unitree_ros/msg/BmsState` | Battery status  |

---

## Web Portal Integration (Previously Tested)

### Overview

A previously tested approach involved integrating the Go1 with a web-based portal for remote control and monitoring. This setup allowed for:

- Remote visualization of robot state and sensor data.
- Real-time control using a web interface.
- Goal-based navigation through a graphical interface.

![Previous System Architecture](../assets/previous_system_architecture.png)

### Architecture

The integration followed a client-server model:

- **Web Interface:** A dashboard for interacting with the Go1.
- **Communication Layer:**
  - WebSockets for real-time communication between the browser and the local machine.
  - A TCP/IP connection between the local machine and the Go1.
- **Control Flow:**
  - User commands from the web interface were translated into ROS2 commands.
  - The Go1 executed these commands while sending feedback data back to the web portal.

### Key Features

- **Live Status Monitoring:**
  - Displayed odometry, IMU data, and battery status.
- **Remote Command Execution:**
  - Users could send movement commands or trigger predefined actions.
- **Goal-Based Navigation:**
  - Allowed users to set navigation targets.

### Notes on Testing

This integration was tested to evaluate the feasibility of controlling the Go1 remotely via a web interface. While it demonstrated the potential for remote operation, further improvements were required for long-term reliability.

---

## Summary

This document outlines the previously tested approach for integrating the Go1 with a web-based control portal. While ROS2 control via `unitree_ros` remains the primary method for low-level operation, web-based remote control was explored as an alternative method for high-level interaction.

For additional details on ROS2-based control, please refer to the main documentation on navigation and hardware setup.
