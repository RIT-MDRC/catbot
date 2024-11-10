# Base image with Ubuntu 22.04 (you can adjust the version)
FROM osrf/ros:jazzy-desktop

# Install Python 3 and virtual environment packages
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /workspace

# Create a virtual environment
RUN python3.12 -m venv /workspace/main-catbot/.venv

# Activate the virtual environment and install any initial Python packages
RUN . /workspace/main-catbot/.venv/bin/activate && pip install --upgrade pip

# Source the ROS 2 setup script by default
RUN echo "source /workspace/main-catbot/.venv/bin/activate" >> /root/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
