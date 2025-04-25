# 🧠 Sagittarius K1 Arm - Perception Stack

This repository contains the full perception + grasping stack for the **Sagittarius K1 robotic arm**, including:

- RealSense camera drivers
- Object detection based on color
- Motion planning via MoveIt
- Gripper control and pick-return routine

---

## 📁 Repository Structure

```
perception_stack/
├── docker-compose.yml             # (Optional) Template to set up container
├── sagittarius_control/          # 🔧 Custom ROS package for perception + grasping
│   ├── config/                   # YAML files for HSV ranges and pre/post pick poses
│   ├── launch/                   # Launch files for detection, pre-pick, and motion logic
│   ├── nodes/                    # Python scripts: detector, move logic, etc.
│   └── src/                      # Optional C++ nodes (e.g., RViz joint mirror)
├── sagittarius_descriptions/     # 🤖 URDF and robot description files
├── sagittarius_moveit/           # 🧠 MoveIt motion planning setup
├── sdk_sagittarius_arm/          # 🧩 Vendor-provided driver package
├── realsense-ros/                # 🎥 RealSense D435 camera drivers
└── README.md
```

---

## 🛠️ Prerequisites

- **Ubuntu 20.04** (recommended)
- **ROS Noetic**
- Python 3.8+
- RealSense D435 camera
- Sagittarius K1 robotic arm

If you want to containerize the stack, use this `docker-compose.yml` as a starting point.

---

## 🔧 Setup Instructions

### 1. Clone the Repo

```bash
git clone https://github.com/AV-Lab/unitree_go1_hardware_interface.git
cd unitree_go1_hardware_interface/k1_arm/perception_stack
```

### 2. Build the Workspace

```bash
catkin_make
source devel/setup.bash
```

---

## 🚀 How to Run the Full Pipeline

### 1. Start the RealSense Camera and Object Detector:

```bash
roslaunch sagittarius_control detect_objects.launch color_name:=green
```

(Defaults to green if no color is passed)

### 2. Run the Main Pick Logic:

```bash
roslaunch sagittarius_control auto_pick_pipeline.launch
```

This will:

- Visualize detected object in RViz
- Move to the object
- Close the gripper
- Return to the configured **post-pick** pose

---

## ⚙️ Configuration Files

### `sagittarius_control/config/color_config.yaml`

HSV threshold ranges for each color (green, red, blue, etc.):

```yaml
green:
  lower: [40, 70, 70]
  upper: [80, 255, 255]
blue:
  lower: [100, 150, 0]
  upper: [140, 255, 255]
```

### `sagittarius_control/config/pre_pick_pose.yaml`

Specifies starting and final pose values:

```yaml
post_pick_rad: [0.0, 0.3, 0.0, 0.4, -1.8, 0.0]
```

---

## 📸 Demos

(Include your demo GIFs, screenshots or videos here)

---

## 🧠 Credits

This stack integrates:

- ROS packages from the official NXROBO Sagittarius K1 SDK
- Intel RealSense camera drivers
