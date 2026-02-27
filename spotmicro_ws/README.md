# SpotMicro ROS 2 Gazebo Simulation

This repository contains the containerized ROS 2 (Humble) packages for simulating the SpotMicro quadruped robot in Gazebo.

## Prerequisites

To run this simulation with hardware GUI acceleration, your host machine must have Docker, Docker Compose, and the NVIDIA Container Toolkit installed. 

We have provided an automated installation script to handle this for you:

```bash
cd spotmicro_ws
chmod +x prerequisites.sh
sudo ./prerequisites.sh
```

**Note:** If you run this script, it will automatically make sure your system has the correct Docker GPG keys and NVIDIA packages installed, and it also sets `start.sh` as executable.

## Automated Setup & Launch

Because Gazebo requires drawing graphical windows (X11) onto your host machine screen, passing the standard `docker compose up` command will often crash the GUI with an `Authorization required` error.

To automatically grant the container permission to use your host's display and spin up the environment with GPU support, use the included startup script:

```bash
cd spotmicro_ws
chmod +x start.sh
./start.sh
```

## Running the Simulation

Once the container is running in the background via the start script, you need to open a bash session inside it to build the ROS 2 workspace and launch Gazebo:

1. **Enter the container:**
   ```bash
   docker exec -it spotmicro_container bash
   ```

2. **Build the ROS 2 packages:**
   ```bash
   colcon build --symlink-install
   ```
   *(Note: You only need to run this when you modify `src/` files, such as `.launch.py` or `.xacro` URDFs).*

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

4. **Launch the Robot in Gazebo:**
   ```bash
   ros2 launch spotmicro_config gazebo.launch.py
   ```

## Troubleshooting

- **Robot exploding on spawn / joints breaking out of nowhere:** 
  Ensure the root `base_link` and chassis dummy links do not have `<collision>` mesh geometries at the `Z=0` origin coordinate. This causes Gazebo math singularities.
- **Gazebo GUI doesn't appear / silently crashes:**
  Double-check that you ran `./start.sh` so `xhost +local:root` was executed on the host system to grant screen privileges.
- **Docker "nvidia" runtime unknown error:**
  You are missing the `nvidia-container-toolkit` on the host machine. You can temporarily disable GPU rendering by removing the `deploy:` block entirely from `docker-compose.yaml`.
