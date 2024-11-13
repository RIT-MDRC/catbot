# Base image with Ubuntu 22.04 (you can adjust the version)
FROM osrf/ros:jazzy-desktop

# Install Python 3 and virtual environment packages
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /workspaces

# Create a virtual environment
RUN python3.12 -m venv /workspaces/main-catbot/.venv

# Use a shell command to find the only folder in the workspace and create a virtual environment
RUN PROJECT_FOLDER=$(ls -d /workspaces/*/ | head -n 1) && \
    python3.12 -m venv "$PROJECT_FOLDER/.venv" && \
    . "$PROJECT_FOLDER/.venv/bin/activate" && pip install --upgrade pip

# Source the virtual environment and ROS 2 setup script by default
RUN PROJECT_FOLDER=$(ls -d /workspaces/*/ | head -n 1) && \
    echo "source $PROJECT_FOLDER/.venv/bin/activate" >> /root/.bashrc
    
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
