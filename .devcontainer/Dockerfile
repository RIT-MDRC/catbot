FROM ros:jazzy

# Create the user
RUN apt-get update && apt-get upgrade -y
RUN sudo apt-get install -y \
  python3-venv \
  python3-pip \
  libbullet-dev \
  python3-pytest-cov \ 
  ros-dev-tools \
  ros-jazzy-ros-gz

# RUN sudo apt-get update \
# sudo apt-get install curl lsb-release gnupg

# RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
#   sudo apt-get update \
#   sudo apt-get install gz-harmonic

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/local_setup.bash" >> /root/.bashrc
CMD ["/bin/bash"]