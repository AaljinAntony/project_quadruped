#!/bin/bash

# Ensure script is run with sudo privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root (sudo ./prerequisites.sh)"
  exit 1
fi

echo "============================================================"
echo "Checking ROS 2 Docker Simulation Prerequisites..."
echo "============================================================"

# 1. Update system package index
echo "--> Updating apt package index..."
apt-get update

# 2. Check and Install Docker Engine
if ! command -v docker >/dev/null 2>&1; then
    echo "--> Docker is not installed. Installing Docker..."
    apt-get install -y ca-certificates curl gnupg lsb-release
    mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
else
    echo "--> Docker is already installed. (Version: $(docker --version))"
fi

# 3. Check and Install Docker Compose
if ! docker compose version >/dev/null 2>&1; then
    echo "--> Docker Compose plugin is missing. Installing..."
    apt-get install -y docker-compose-plugin
else
    echo "--> Docker Compose is ready."
fi

# 4. Check and Install NVIDIA Container Toolkit
if ! dpkg -l | grep -q nvidia-container-toolkit; then
    echo "--> NVIDIA Container Toolkit not found. Installing..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    apt-get update
    apt-get install -y nvidia-container-toolkit

    # Configure the Docker daemon to use the Nvidia runtime
    nvidia-ctk runtime configure --runtime=docker
    systemctl restart docker

    echo "--> NVIDIA Container Toolkit installed successfully."
else
    echo "--> NVIDIA Container Toolkit is already installed."
fi

# 5. Make the start.sh script executable
echo "--> Setting executable permissions on start.sh..."
chmod +x start.sh || true

echo "============================================================"
echo "All prerequisites satisfied! You can now run: ./start.sh"
echo "============================================================"
