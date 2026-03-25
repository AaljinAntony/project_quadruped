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
- **Windows Support**: Added `start.bat` and `prerequisites.ps1` for Windows 10/11 users (WSL 2 / WSLg required for GUI).
- The `SpotMicro_Firmware/src/main.cpp` requires hardcoded Wi-Fi credentials (`ssid`, `psk`) and the Host's IP (`agent_ip`) to communicate with the Micro-ROS Agent.

- **Network Isolation Discovery**: Initial failures were caused by host firewall (`ufw`) and router "AP Isolation". Switching to a laptop-hosted hotspot (`SpotMicro`) resolved the link-layer reachability issues. (2026-03-15)
- **Final Networking Proof**: ESP32 IP `10.42.0.128` successfully pings Host `10.42.0.1`. Handshake confirmed in Agent logs with "session established".

### 📶 Final Working Network Configuration
- **Method**: Laptop Hotspot (Workaround for Router AP Isolation)
- **SSID**: `SpotMicro`
- **Pass**: `spotpassword`
- **Host IP**: `10.42.0.1`
- **ESP32 IP**: `10.42.0.128`
- **Port**: `8888` (UDP)
- **Status**: **SUCCESS - Session Established** ✅

## Trade-offs
- **Wi-Fi vs. Wired for Micro-ROS**: Using Wi-Fi for the ESP32 connection enables a completely untethered physical robot, but introduces potential latency and reliability issues compared to a hardwired serial connection.
- **Pre-allocating Micro-ROS Strings**: Using C-style `malloc` for the `rosidl_runtime_c__String` capacities guarantees stability in tight microcontroller memory but requires careful manual memory management during initialization in `main.cpp`.

## Known Issues & Unresolved Questions
- **Gazebo UI Crashes**: Running the simulation without `./start.sh` (which handles `xhost +local:root`) leads to GUI crashes due to X11 authorization issues.
- **Micro-ROS Agent Connection**: Requires `network_mode: host` in `docker-compose.yaml`.
- **ESP32 Serial Monitor**: After initialization, `main.cpp` enters a loop waiting for ROS 2 messages. It will NOT print anything to the serial monitor unless there's an error or a specific print statement is triggered.
### Motor Control & Calibration (New Hardware Mapping)
**Verified Offsets Table (Reference Center: 307 PWM):**
| Joint | PCA Pin | PWM Neutral | Degree Offset (Approx) |
|---|---|---|---|
| FL Foot | 0 | 362 | **+27.4°** |
| FL Leg | 1 | 292 | **-7.5°** |
| FL Shoulder | 2 | 302 | **-2.5°** |
| RL Foot | 4 | 375 | **+33.9°** |
| RL Leg | 5 | 302 | **-2.5°** |
| RL Shoulder | 6 | 303 | **-2.0°** |
| FR Foot | 8 | 229 | **-38.9°** |
| FR Leg | 9 | 305 | **-1.0°** |
| FR Shoulder | 10 | 306 | **-0.5°** |
| RR Foot | 12 | 224 | **-41.4°** |
| RR Leg | 13 | 311 | **+2.0°** |
| RR Shoulder | 14 | 320 | **+6.5°** |

- **Verification Logic**: 115 ticks per Rad = ~2.01 ticks per Degree. 
- **Mirroring Logic**: Left side (Idx 0-5) positive angle is PWM increase. Right side (Idx 6-11) positive angle is PWM decrease.
- **Physical vs Mathematical Asymmetry**: While mathematically averaging left/right yielded symmetrical angles naturally in code, the physical hardware of the limbs required completely custom baselines tuned leg-by-leg to achieve physical standing symmetry.

- **Control Strategy**: 
  - Host-side Python CLI (`scripts/motor_calibration_cli.py`) sends raw PWM ticks (approx 100-500).
  - **Pose Transitions**: Firmware implements 2-second linear interpolation (`is_transitioning`) for macroscopic stance shifts (Sit/Stand/Neutral) to prevent instantaneous hardware jerking. Active walking or direct micro-ROS commands automatically interrupt and override this smoothing to maintain low latency.
  - **Baseline**: Script resets to **300 PWM** on motor switch.
  - **Logging**: Interactive results are logged to `/workspace/calibration_results.txt` for automated extraction.
  - **Human Readability**: Script now logs **Approximate Degrees** alongside PWM (`deg = (PWM * 4.8828 - 1500) * 0.09`).
  - **Reliability**: CLI handles `Ctrl+C` for clean exits and terminal restoration.
  - ESP32 provides feedback on `/motor_status` including battery voltage.

### 🏆 Milestone: Stable Walking Achieved (2026-03-25)
- **Status**: SUCCESS ✅
- **Details**: The robot now achieves a stable Trot gait using a unified controller architecture.
- **Firmware**: `SpotMicro_Firmware/src/main.cpp` contains the production stance control with hardcoded physical offsets.
- **Controller**: `spotmicro_ws/src/spotmicro_config/scripts/spotmicro_unified_controller.py` implements the high-level IK and gait phases.
- **Backup**: Created `main.cpp.stable_walking` and `spotmicro_unified_controller.py.stable_walking` for safe restoration.
- **Confirmation**: Physical walking performance verified. Offsets are validated leg-by-leg.

### Known Issues & Unresolved Questions
- **Battery Sense**: ADC pin used for voltage sensing might need specific calibration for the user's divider ratio.
- **CHAMP Alignment**: Once calibration is done, the offsets must be transferred to the production `main.cpp`.
