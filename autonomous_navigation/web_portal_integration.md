# Web Portal Integration

This document explains how the Unitree Go1 hardware interface is integrated with our web portal to enable remote control, real-time monitoring, and visualization.

---

## Overview

![web_portal_snapshot1](../assets/web_portal_snapshot1.png)

The integration leverages a **ROS bridge** to convert ROS topics into WebSocket messages, allowing a web client to subscribe and publish data in real time. This enables:

1. **Manual Control**

   - **Manual Exploration & Mapping:**  
     Operators can manually control the Go1 using WASD keys or on-screen sliders, enabling precise movement to explore and map the area effectively.
   - **Real-Time Speed Feedback:**  
     The current linear and angular speeds are displayed live on the web portal, allowing users to fine-tune the robot's movement dynamically.

2. **Autonomous Navigation**

   - **Goal-Based Movement:**  
     Users can specify waypoints (e.g., “Point A” or “Point B”) on the interface, prompting the Go1 to autonomously navigate to the designated location.
   - **Collision Avoidance:**  
     The system uses local and global costmaps to detect and avoid obstacles during navigation.
   - **Emergency Stop:**  
     A dedicated button immediately halts the robot’s autonomous movement when necessary.

3. **Real-Time Visualization**
   - **Dynamic Map Rendering:**  
     Occupancy grids, costmaps, and planned paths are rendered live in the web portal.
   - **Customizable Topic Selection:**  
     Similar to RViz, users can select which ROS topics (e.g., `/map`, `/slam_planner_node/local_costmap/footprint`) to visualize.
   - **Interactive 3D Interface:**  
     A custom Three.js-based visualization provides an immersive, interactive environment for monitoring the robot's movement and surroundings.

The system typically runs the ROS bridge within a ROS Melodic container, ensuring a consistent environment on the Go1.

---

## ROS Bridge Setup

The `rosbridge_server` exposes ROS topics over a WebSocket connection. On the external control PC, launch the server with:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=9090
```

This allows the web client to interact with topics such as `/cmd_vel`, `/map`, and others.

---

## Web Client Code Examples

### Using ROSLIB.js in React

Below is an example React hook that connects to the rosbridge server and subscribes to the `/cmd_vel` topic:

```javascript
useEffect(() => {
  const rosInstance = new ROSLIB.Ros({
    url: "ws://localhost:9090",
  });

  rosInstance.on("connection", () => {
    console.log("Connected to ROS websocket server");
    setIsConnected(true);

    // Initialize cmd_vel topic
    const cmdVel = new ROSLIB.Topic({
      ros: rosInstance,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    setCmdVelTopic(cmdVel);
  });

  rosInstance.on("error", (error) => {
    console.error("Error connecting to ROS websocket server:", error);
    setIsConnected(false);
  });

  rosInstance.on("close", () => {
    console.log("Connection to ROS websocket server closed");
    setIsConnected(false);
  });

  setRos(rosInstance);

  return () => {
    if (rosInstance) {
      rosInstance.close();
    }
  };
}, []);
```

### Using Native WebSocket

This snippet shows how to connect directly and subscribe to topics such as `/map` and `/slam_planner_node/local_costmap/footprint`:

```javascript
const ws = new WebSocket("ws://localhost:9090");

ws.onopen = () => {
  console.log("Connected to ROS bridge!");
  // Subscribe to the map topic
  ws.send(
    JSON.stringify({
      op: "subscribe",
      topic: "/map",
      type: "nav_msgs/OccupancyGrid",
    })
  );

  // Subscribe to the local costmap footprint topic
  ws.send(
    JSON.stringify({
      op: "subscribe",
      topic: "/slam_planner_node/local_costmap/footprint",
      type: "geometry_msgs/PolygonStamped",
    })
  );
};
```

---

## Visualization with Three.js

The web portal also includes a custom 3D visualization interface built with **Three.js**. This interactive view lets users:

- **Select ROS Topics:**  
  Choose which data (e.g., maps, costmaps) to render.
- **Monitor Robot Movement:**  
  Visualize the robot’s trajectory, current position, and planned paths, similar to RViz.
- **Interact with the Scene:**  
  Zoom, pan, and rotate the 3D view to examine the environment in detail.

---

## Summary

- **ROS Bridge:** Converts ROS topics to WebSocket messages.
- **Web Client:** Uses React/JS (with ROSLIB.js or native WebSocket) to send commands and receive sensor data.
- **Visualization:** Offers an interactive, Three.js-powered 3D interface for monitoring the robot's environment.
- **Integration Environment:** Typically run within a ROS Melodic container for consistency on the Go1.

This integration is a key component of the AutoEye project for fleet management and control. For more details on AutoEye, please visit the [AutoEye Project Repository](https://github.com/AV-Lab/AutoEye).
