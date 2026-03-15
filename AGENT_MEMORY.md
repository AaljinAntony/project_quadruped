# AGENT_MEMORY.md

> Memory file for tracking architectural decisions, constraints, discoveries, trade-offs, and issues as defined by `GEMINI.md`.

## Architectural Decisions
- **Core Architecture**: The system uses a hybrid architecture: ROS 2 Humble (running in a Docker container/Gazebo for simulation and heavy compute like the CHAMP walking engine) and Micro-ROS on an ESP32 for real-time hardware control.
- **Hardware Integration**: The physical robot uses an ESP32 connected to a PCA9685 PWM servo driver via I2C (SDA: GPIO 21, SCL: GPIO 22). The PCA9685 requires an external 5V/6V BEC.
- **Sensors**: Included an MPU6050 IMU on the shared I2C bus (0x68) and an HC-SR04 Ultrasonic sensor (Trig: GPIO 4, Echo: GPIO 18).
- **Communication Protocol**: The ESP32 connects to the host machine running the Micro-ROS agent via Wi-Fi. It subscribes to the `/joint_states` topic published by the CHAMP engine, and publishes to `/imu/data` (20Hz) and `/ultrasonic/range` (5Hz).
- **Control Interface**: Teleoperation is handled via Gamepad (`champ_teleop`) or Keyboard (`teleop_twist_keyboard`) in a holonomic drive mode.
- **Agent Framework**: The repository uses the Antigravity Kit framework containing 20 specialist agents, 36 skills, and 11 workflows orchestrating all AI assistant behavior.

## Constraints & Assumptions
- The physical robot's servos (DS3235) have safe pulse width limits of 500us to 2500us.
- Host machine must support Docker, Docker Compose, and NVIDIA Container Toolkit for full simulation capabilities.
- The `SpotMicro_Firmware/src/main.cpp` requires hardcoded Wi-Fi credentials (`ssid`, `psk`) and the Host's IP (`agent_ip`) to communicate with the Micro-ROS Agent.

- **Network Isolation Discovery**: ESP32 communication failed on mobile hotspots due to "Client Isolation" (phone hotspots) and 5GHz band incompatibility. 2.4GHz is mandatory. Link-layer (ARP) resolution is the definitive reachability test. (2026-03-15)
- **Diagnostic Firmware Strategy**: Replaced standard Micro-ROS code with a "Diagnostic Suite" for step-by-step verification (Scan -> Connect -> Gateway Ping -> Host Ping -> UDP Loopback).
- **Docker Status**: Only the `micro-ros-agent` container is currently running. The main `spotmicro_container` is exited.

## Trade-offs
- **Wi-Fi vs. Wired for Micro-ROS**: Using Wi-Fi for the ESP32 connection enables a completely untethered physical robot, but introduces potential latency and reliability issues compared to a hardwired serial connection.
- **Pre-allocating Micro-ROS Strings**: Using C-style `malloc` for the `rosidl_runtime_c__String` capacities guarantees stability in tight microcontroller memory but requires careful manual memory management during initialization in `main.cpp`.

## Known Issues & Unresolved Questions
- **Gazebo UI Crashes**: Running the simulation without `./start.sh` (which handles `xhost +local:root`) leads to GUI crashes due to X11 authorization issues.
- **Micro-ROS Agent Connection**: Requires host network mode in docker-compose. If "Session established" is missing, host firewall or IP mismatches are the primary causes.
