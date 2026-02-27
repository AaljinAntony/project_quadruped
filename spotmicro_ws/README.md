# 🐕 Project Quadruped: Master Operations Manual

**Stack:** ROS 2 Humble (Docker) + Gazebo (Sim) + Micro-ROS (Real Hardware)  
**Supported Modes:** Virtual Simulation & Physical Field Testing

This repository contains the complete "Mission Control" software stack and the "Ground Crew" firmware for the SpotMicro quadruped robot.

---

## 📂 Repository Structure

* **`spotmicro_ws/`**: The ROS 2 workspace containing the CHAMP walking engine, URDFs, and logic nodes.
* **`SpotMicro_Firmware/`**: The C++ PlatformIO/Arduino code for the ESP32-WROOM-32.
* **`docker-compose.yaml`**: Orchestrates the ROS 2 environment, GPU pass-through, and Micro-ROS Agent.
* **`scripts/`**: Automation scripts (`start.sh`, `prerequisites.sh`) for host setup.

---

## ✅ Part 1: Installation & Prerequisites

To run this stack (especially Simulation), your host machine must have Docker, Docker Compose, and the NVIDIA Container Toolkit installed.

We have provided an automated script to handle this setup:

```bash
# From the project root
chmod +x prerequisites.sh
sudo ./prerequisites.sh

```

**What this does:**

* Installs Docker & GPG keys.
* Installs `nvidia-container-toolkit` for GPU acceleration.
* Sets execution permissions for startup scripts.

---

## 🎮 Part 2

Use this mode to test code logic, walking gaits, and inverse kinematics without risking hardware damage.

### 1. Launch the Environment

Because Gazebo requires graphical windows (X11), use our wrapper script instead of standard Docker commands. This script authorizes the container to use your host display.

```bash
./start.sh

```

### 2. Enter the Container

Once the background container is running:

```bash
docker exec -it spotmicro_container bash

```

### 3. Build & Launch

Inside the container:

```bash
# Build the workspace (only needed after modifying src files)
colcon build --symlink-install
source install/setup.bash

# Launch the Gazebo Simulation(**if needed**)
# (the simulation robot has vibration due to gazebo physics calculations, it will not be in real robot)
ros2 launch spotmicro_config gazebo.launch.py

```

---

## 🤖 Part 3: Field Mode (Real Hardware)

Use this mode to control the physical robot.

### 1. Hardware Setup (Ground Crew)

* **Wiring:**
* **ESP32** -> **PCA9685** (I2C): `SDA` (GPIO 21), `SCL` (GPIO 22).
* **Power:** PCA9685 must be powered by an external 5V/6V BEC (Not USB).


* **Firmware:**
1. Open `SpotMicro_Firmware` in VS Code/PlatformIO.
2. Edit `main.cpp`: Update `WIFI_SSID`, `WIFI_PASS`, and `AGENT_IP` (Your Laptop's IP).
3. Flash to ESP32.



### 2. Start Mission Control

For real hardware, we need the **Micro-ROS Agent** to translate Wi-Fi packets into ROS 2 topics.

```bash
# From the project root (spotmicro_ws)
# (Ensure start.sh or docker compose is running)
#(the below command is already running if you ran the start.sh script, continue with the next step)
docker compose up -d microros_agent

```

**Verify Connection:**
Power on the robot and check logs:

```bash
docker compose logs -f microros_agent
# Look for: [microros_agent] Session established for client...

```

### 3. Operation Protocols

#### A. Calibration (The "Horns Off" Rule)

* **When:** First assembly or after repairs.
* **Action:**
1. Prop robot on a box (legs in air).
2. Run: `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
3. Align legs perfectly straight using sliders.
4. Save offset values into ESP32 firmware `joint_offsets`.



#### B. Autonomous Walking

* **Action:**
```bash
# Inside the container
ros2 launch spotmicro_config bringup.launch.py use_sim_time:=False

```

Here is the **Joystick / Gamepad** section for your `README.md`.

It includes the critical **"Deadman Switch"** instruction, which is the #1 reason teammates think the robot is broken (when they just aren't holding the safety button).

---

### 🕹️ Joystick / Gamepad Control (Recommended)(Keyboard section below this)

For the best experience, use a USB or Bluetooth Gamepad (Xbox 360/One, PS4, or Logitech F710).

**1. Connect the Controller**
* Plug in your joystick **BEFORE** starting the Docker container.
* Ensure your OS sees it (usually as `/dev/input/js0`).

**2. Launch the Teleop Node**
Run this in a new terminal inside the container:
```bash
ros2 launch champ_teleop teleop.launch.py

```

**3. ⚠️ The "Deadman" Safety Switch**
The robot will **NOT** move if you just push the sticks. You must hold the safety button:

* **Enable (Hold):** `L1` (PS4) or `LB` (Xbox)
* **Turbo (Hold):** `R1` (PS4) or `RB` (Xbox)

**4. The Controls**
| Stick | Axis | Action |
| :--- | :--- | :--- |
| **Left Stick** | Up / Down | Walk Forward / Backward |
| **Left Stick** | Left / Right | Strafe (Side-step) |
| **Right Stick** | Left / Right | Turn (Yaw) |

> **Note for Docker Users:** If the container cannot find the joystick (error: `file not open`), you may need to restart the container with the joystick plugged in, or add the `--device /dev/input` flag to your run command.

### 🎮 Keyboard Teleop (No Joystick?)

If you don't have a gamepad, you can drive the robot using your keyboard.

**1. Launch the Keyboard Node**
Run this command inside the running container:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**2. The Controls (Holonomic Mode)**

* **Walk Forward/Back:** `i` / `,`
* **Strafe Left/Right:** `Shift + j` / `Shift + l`
* **Turn Left/Right:** `j` / `l`
* **Stop Instantly:** `k`

**⚠️ CRITICAL SAFETY TIP:**
Keyboards are "binary" (0% or 100% speed), which can make the robot jerk and fall.

* **Before moving:** Press `z` repeatedly to lower the speed to **0.1 m/s**.
* **To speed up:** Press `q` to increase speed incrementally.
* **To stop:** Always press `k` instead of just letting go of the key.

---

## 🚨 Troubleshooting

| Issue | Context | Solution |
| --- | --- | --- |
| **Gazebo GUI crashes / "Authorization required"** | Sim | Ensure you ran `./start.sh` (not just docker compose) to run `xhost +local:root`. |
| **Robot explodes on spawn** | Sim | Check `base_link` collision meshes. Remove `<collision>` from root dummy links. |
| **"nvidia" runtime unknown** | Sim | Missing drivers. Run `./prerequisites.sh` or remove `deploy:` block from `docker-compose.yaml` to run CPU-only. |
| **"Session established" missing** | Real | 1. Check Wi-Fi/IP match. <br>2. Disable Laptop Firewall. <br>3. Ensure `network_mode: host` is set in docker-compose. |
| **Violent Leg Jitter** | Real | PID gains are too high for physical servos. Lower the `P` gain in `ros2_controllers.yaml`. |