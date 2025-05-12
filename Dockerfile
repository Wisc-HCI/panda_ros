
FROM osrf/ros:noetic-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl wget \
    nano \
    build-essential \ 
    cmake \
    libeigen3-dev\
    python3-catkin-tools \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions\
    usbutils

# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Set the default command to execute
CMD ["bash"]



