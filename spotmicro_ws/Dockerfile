FROM osrf/ros:humble-desktop

# Remove invalid nvidia apt source if it somehow exists in the base image
RUN rm -f /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install build tools and essential dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# Set the working directory
WORKDIR /workspace

# Copy the source code into the container to resolve dependencies during build
COPY src /workspace/src

# Install all ROS 2 dependencies based on the packages in the src folder
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

# Compile the workspace automatically
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Auto-source the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc