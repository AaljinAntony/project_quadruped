#!/bin/bash
# Grant local docker containers X11 permission to open GUIs (like Gazebo/RViz)
xhost +local:root

# Start the docker containers in the background
docker compose up -d

echo "✅ Docker environment started with GUI permissions."
echo "You can now enter the container with: docker exec -it spotmicro_container bash"
