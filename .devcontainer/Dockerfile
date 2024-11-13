# Base image with Ubuntu 22.04 (you can adjust the version)
FROM osrf/ros:jazzy-desktop

# Install Python 3 and virtual environment packages
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /workspace