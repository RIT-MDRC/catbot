FROM ros:jazzy

# Create the user
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
  python3-venv \
  python3-pip \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
ENV SHELL=/bin/bash

RUN echo "source /opt/ros/jazzy/setup.bash" >> /.bashrc

CMD ["/bin/bash"]