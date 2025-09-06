#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Update package lists and install dependencies
sudo apt-get update && sudo apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-wstool \
    python3-catkin-tools \

# Clean up the apt cache to reduce image size
sudo rm -rf /var/lib/apt/lists/*

echo "Dependencies installed successfully."