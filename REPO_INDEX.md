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

### `SpotMicro_Firmware/src/main.cpp` (Calibration Active)
- **Purpose**: Currently holds the calibration firmware for tuning servos.
- **Key Responsibilities**: Receives PWM ticks over `/motor_calibration` and drives PCA9685 pins.
- **Mapping**: Matches the user-provided CH 0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14 structure.

### `spotmicro_ws/src/spotmicro_config/scripts/motor_calibration_cli.py`
- **Purpose**: Interactive CLI for motor calibration with persistence.
- **Key Responsibilities**: Key-driven interface (W/S/E/D/A/F) for adjusting individual servos. Features a **300 PWM baseline** on motor switch and **persistent logging** to `/workspace/calibration_results.txt`.

### `SpotMicro_Firmware/src/main.cpp.normal`
- **Purpose**: Stable production firmware for CHAMP control.
- **Key Responsibilities**: Receives joint positions and performs the inverse kinematics/mapping for production walking.
