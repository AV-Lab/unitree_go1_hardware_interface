# Autonomous Navigation Documentation

This folder details how the Unitree Go1 achieves both **manual** and **autonomous** navigation via a web portal. It covers hardware setup, networking, and the integration with a ROS bridge to support real-time control and visualization.

---

## Overview

![autonomous_navigation_demo1](../assets/autonomous_navigation_demo1.gif)

1. **Communication & System Integration**

   - **ROS Bridge Architecture:**  
     The system leverages `rosbridge_server` to convert ROS topics into WebSocket messages, enabling seamless communication between the Go1 and an external control PC.
   - **Distributed Processing Advantage:**  
     Forwarding ROS topics to an external PC allows heavy computations (such as advanced visualization and path planning) to be offloaded from the Go1.

2. **Manual Control**

   - **Exploration & Mapping:**  
     Operators use WASD keys or on-screen sliders to manually control the Go1, allowing precise movement for exploration and mapping.
   - **Live Speed Feedback:**  
     The current linear and angular speeds are displayed in real time, enabling fine-tuning of the robot's motion.

3. **Autonomous Navigation**

   - **Goal-Based Movement:**  
     Users can set waypoints (e.g., "Point A" or "Point B") for the Go1 to navigate autonomously.
   - **Collision Avoidance & Emergency Stop:**  
     The system uses costmaps to avoid obstacles and includes a dedicated button to halt autonomous navigation immediately.

4. **Real-Time Visualization**
   - **Dynamic Map Rendering:**  
     Occupancy grids, costmaps, and path plans are rendered live in the browser.
   - **Interactive 3D Interface:**  
     A Three.js-powered interface provides an immersive 3D view, allowing users to select which ROS topics to visualize (similar to RViz).

---

## File Guide

- **[system_setup](system_setup.md)**  
  Covers the full hardware and networking setup, including:

  - **Hardware Setup:** Mounting the K1 arm (if present), LiDAR connection via Ethernet, and power distribution details.
  - **Networking Setup:** Enabling IP forwarding, configuring NAT (iptables) on the Go1’s Pi, and setting up routes on the external PC for ROS topic sharing.

- **[web_portal_integration.md](web_portal_integration.md)**  
  Shows how to launch the rosbridge server and includes sample React/JS code for subscribing/publishing to ROS topics. Demonstrates manual vs. autonomous navigation commands and how to visualize maps/costmaps in real time (e.g., using Three.js).

- **[go1_ros2_web_integration.md](ros2_web_integration.md)**  
  Documents previously tested ROS2 integration methods, including web-based control, and explains their limitations.

---

## Future Improvements

- **Camera Streaming Integration**

  - Stream live video from the Go1’s built-in cameras directly to the web portal.
  - Add options for selecting camera feeds (front, rear, or side views) and displaying real-time footage alongside map visualizations.

- **Enhanced Localization with 3D LiDAR**

  - Upgrade from 2D to 3D LiDAR for improved environmental perception.
  - Implement advanced SLAM algorithms (like LIO-SAM or Cartographer) to enhance mapping accuracy and better localize the Go1 in dynamic environments.

- **K1 Arm Filtering from LiDAR Maps**

  - Refine sensor data processing so the mounted K1 arm is ignored by the LiDAR.
  - Implement custom filters or transform frames (using TF in ROS) to differentiate the arm from obstacles, preventing false-positive detections during navigation.

- **Advanced Web Portal Features**
  - Add 3D visualizations of both the Go1 and K1 arm using interactive models (e.g., via Three.js).
  - Enable goal-setting through point-and-click on the map, with live feedback showing robot/arm paths and planned actions.
