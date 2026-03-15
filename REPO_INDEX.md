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
- **Purpose**: Micro-ROS bridge firmware for hardware control.
- **Key Responsibilities**: Manages Wi-Fi connectivity to the host hotspot, initializes Micro-ROS transport, and subscribes to `/joint_states`. It maps incoming ROS 2 joint positions to physical PCA9685 PWM pins while applying calibration offsets.
- **Dependencies**: `micro_ros_platformio`, `Adafruit_PWMServoDriver`, `WiFi`.

### `spotmicro_ws/docker-compose.yaml`
- **Purpose**: Orchestration for the ROS 2 and Micro-ROS environment.
- **Key Responsibilities**: Defines the `spotmicro_dev` (simulation/compute) and `microros_agent` services. Critically uses `network_mode: host` to allow the Micro-ROS agent to bind to the laptop's hotspot interface for ESP32 discovery.
- **Dependencies**: Docker Compose.

### `SpotMicro_Firmware/scripts/udp_diagnostic.py`
- **Purpose**: Host-side UDP listener for network diagnostics.
- **Key Responsibilities**: Listens for UDP packets from the ESP32 on port 8888 and responds with an ACK packet to verify bidirectional reachability outside of the Micro-ROS stack.
- **Dependencies**: Python 3.

### `SpotMicro_Firmware/platformio.ini`
- **Purpose**: Build configuration for the ESP32 firmware.
- **Key Responsibilities**: Configures the `esp32dev` environment, sets the framework to `arduino`, configures the Micro-ROS transport to `wifi` (distro: `humble`), and lists external library dependencies.
- **Dependencies**: PlatformIO.

### `spotmicro_ws/src/spotmicro_config/package.xml`
- **Purpose**: ROS 2 package manifest for SpotMicro configurations.
- **Key Responsibilities**: Defines the build dependencies and export types for the CHAMP configuration package specific to SpotMicro.
- **Dependencies**: `ament_cmake`, `launch_ros`.

### `spotmicro_ws/src/spotmicro_description/package.xml`
- **Purpose**: ROS 2 package manifest for SpotMicro URDF description.
- **Key Responsibilities**: Defines the package containing the robot's physical model, collision meshes, and Gazebo model path exports.
- **Dependencies**: `ament_cmake`.
