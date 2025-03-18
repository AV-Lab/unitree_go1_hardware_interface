# K1 Arm Control - Standalone Implementation

This directory contains a **standalone C++ implementation** for controlling the NXROBO K1 robotic arm. It provides two primary methods of control:

1. **Terminal-based control** via `arm_control.cpp`
2. **Web portal control** via WebSockets in `arm_ws.cpp`

## Overview

- **Direct terminal control** of the K1 arm via a C++ API.
- **Web interface control** using WebSockets for remote operation.
- **Standalone operation** on a Raspberry Pi or PC, independent of ROS.

![k1_arm_sdk_demo](../assets/k1_arm_sdk_demo.gif)

---

## Prerequisites

- **Ubuntu 18.04/20.04** or **Raspberry Pi OS**
- **C++17** compatible compiler
- **Eigen3** (for matrix operations)
- **Boost** (for threading and async operations)
- **uWebSockets** (for WebSocket server)
- **OpenSSL** and **zlib** (for building uWebSockets)
- **Sagittarius SDK** (cloned as a submodule in this repo)

---

## Installation

### Step 1: Clone the Repository with Submodules

```bash
git clone --recursive https://github.com/AV-Lab/unitree_go1_hardware_interface.git
```

If you cloned without submodules, initialize and update them:

```bash
cd unitree_go1_hardware_interface/k1_arm/sdk_control
git submodule init
git submodule update
```

### Step 2: Navigate to the `sdk_control` Folder

```bash
cd unitree_go1_hardware_interface/k1_arm/sdk_control
```

### Step 3: Install Dependencies

```bash
# Install Eigen3
sudo apt-get install libeigen3-dev

# Create symbolic link for easier includes
cd /usr/include/
sudo ln -sf eigen3/Eigen Eigen
cd -

# Install Boost libraries
sudo apt-get install libboost-system-dev libboost-thread-dev

# Install WebSocket dependencies
sudo apt-get install libssl-dev zlib1g-dev

# Build and install uWebSockets
git clone https://github.com/uNetworking/uWebSockets.git
cd uWebSockets
make
sudo make install
cd ..
```

### Step 4: Install the Sagittarius SDK Library

Copy the correct dynamic library for your platform into `/usr/lib/`:

- **For x86 (64-bit PC):**
  ```bash
  cd sagittarius_sdk
  sudo cp ./lib/x86_64/libsagittarius_sdk.so /usr/lib/
  ```
- **For arm64 (Raspberry Pi):**
  ```bash
  cd sagittarius_sdk
  sudo cp ./lib/arm64/libsagittarius_sdk.so /usr/lib/
  ```

### Step 5: Compile the Control Programs

From within `k1_arm/sdk_control/`, compile each program directly from the `src` folder.

**Terminal-based control** (`arm_control.cpp`):

- **For x86 (64-bit PC):**

  ```bash
  g++ -std=c++17 -I./sagittarius_sdk -I./src \
      -o arm_control \
      src/arm_control.cpp \
      -L./sagittarius_sdk/lib/x86_64 \
      -lsagittarius_sdk \
      -lpthread \
      -lboost_system \
      -lboost_thread
  ```

- **For arm64 (Raspberry Pi):**
  ```bash
  g++ -std=c++17 -I./sagittarius_sdk -I./src \
      -o arm_control \
      src/arm_control.cpp \
      -L./sagittarius_sdk/lib/arm64 \
      -lsagittarius_sdk \
      -lpthread \
      -lboost_system \
      -lboost_thread
  ```

**Web portal control** (`arm_ws.cpp`):

- **For x86 (64-bit PC):**

  ```bash
  g++ -std=c++17 -I./sagittarius_sdk -I./uWebSockets/src -I./src \
      -o arm_ws \
      src/arm_ws.cpp \
      -L./sagittarius_sdk/lib/x86_64 \
      -lsagittarius_sdk \
      -lpthread \
      -lboost_system \
      -lboost_thread \
      -lz \
      -lssl \
      -lcrypto
  ```

- **For arm64 (Raspberry Pi):**
  ```bash
  g++ -std=c++17 -I./sagittarius_sdk -I./uWebSockets/src -I./src \
      -o arm_ws \
      src/arm_ws.cpp \
      -L./sagittarius_sdk/lib/arm64 \
      -lsagittarius_sdk \
      -lpthread \
      -lboost_system \
      -lboost_thread \
      -lz \
      -lssl \
      -lcrypto
  ```

---

## Usage

### Terminal-based Control

Run the terminal-based program:

```bash
./arm_control [PORT]
```

- **Default port** is `/dev/ttyUSB0`.
- This program allows you to directly control the arm via a command-line interface.

### Web Portal Control

Run the WebSocket server for remote control:

```bash
./arm_ws [PORT] [WS_PORT]
```

- **Default arm port** is `/dev/ttyUSB0`.
- **Default WebSocket port** is `8080`.

Once running, connect to `ws://YOUR_IP:8080` (or the chosen port) from a web client to send commands.

---

## WebSocket API

When using the web portal control, the server accepts JSON commands like:

```json
{
  "command": "move",
  "joints": [90, 45, 30, 0, 0, 0]
}

{
  "command": "gripper",
  "state": "open"
}
```

---

## Troubleshooting

- **Permission Denied on /dev/ttyUSB0**  
  Run:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  ```
- **Library Not Found**  
  Verify you copied the correct `libsagittarius_sdk.so` for your architecture.
- **Compilation Errors**  
  Make sure you’ve installed all required dependencies (Eigen, Boost, uWebSockets, OpenSSL, zlib).

---

## Acknowledgments

- **NXROBO** for providing the Sagittarius SDK.
- **uWebSockets** for the WebSocket library.
