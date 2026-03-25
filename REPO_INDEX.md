# REPO_INDEX.md

> File-level memory index. Append concise summaries of files upon first reading as outlined in `GEMINI.md`.

## File Summaries

### `spotmicro_ws/README.md`
- **Purpose**: Main documentation for the project stack.
- **Key Responsibilities**: Explains the setup, prerequisites, execution commands for Docker/Gazebo simulation, hardware setup (Ground Crew), and troubleshooting for both virtual and physical modes.
- **Dependencies**: Docker, Docker Compose, NVIDIA Container Toolkit.

### `.agent/ARCHITECTURE.md`
- **Purpose**: Defines the structure and capabilities of the Antigravity Kit AI framework.
- **Key Responsibilities**: Outlines the 20 specialist agents, 36 modular skills, 11 slash command workflows, and master validation scripts. Explains the agent routing and skill loading mechanisms.
- **Dependencies**: N/A.

### `SpotMicro_Firmware/src/main.cpp`
- **Purpose**: Production firmware for unified stance and walking control.
- **Key Responsibilities**: Manages micro-ROS communication, implements smooth-step stance transitions, and provides direct joint control with integrated physical offsets (calibrated for hardware asymmetry).
- **Mapping**: Matches the 12-motor PCA9685 configuration.

### `spotmicro_ws/src/spotmicro_config/scripts/spotmicro_unified_controller.py`
- **Purpose**: Unified IK and Trot gait controller (Python/ROS 2).
- **Key Responsibilities**: Solves 2D planar IK, manages gait timing (75% stance, 25% swing), and provides keyboard-driven state transitions (Sit, Stand, Trot).
- **Dependencies**: `rclpy`, `sensor_msgs`.

### `spotmicro_ws/src/spotmicro_config/scripts/motor_calibration_cli.py`
- **Purpose**: Interactive CLI for motor calibration with persistence.
- **Key Responsibilities**: Key-driven interface (W/S/E/D/A/F) for adjusting individual servos. Features a **300 PWM baseline** on motor switch and **persistent logging** to `/workspace/calibration_results.txt`.

### `SpotMicro_Firmware/src/main.cpp.normal`
- **Purpose**: Stable production firmware for CHAMP control.
- **Key Responsibilities**: Receives joint positions and performs the inverse kinematics/mapping for production walking.
### `spotmicro_ws/start.bat` [NEW]
- **Purpose**: Batch script for Windows users to launch the Docker environment.
- **Key Responsibilities**: Checks for Docker and runs `docker compose up -d`. Notes WSLg support for GUIs.
- **Dependencies**: Docker Desktop.

### `spotmicro_ws/prerequisites.ps1` [NEW]
- **Purpose**: PowerShell script to automate prerequisite checks on Windows.
- **Key Responsibilities**: Verifies Administrator privileges, WSL 2 installation, Docker Desktop, and NVIDIA drivers. Provides `winget` installation tips.
- **Dependencies**: PowerShell (Admin).
