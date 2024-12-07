# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:noetic-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl wget\
    nano\
    python3-pip\
    build-essential\ 
    cmake\
    libeigen3-dev\
    python3-catkin-tools\
    ros-noetic-libfranka

# Install Julia for RelaxedIK
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.1-linux-x86_64.tar.gz
RUN tar zxvf julia-1.8.1-linux-x86_64.tar.gz
RUN echo 'export PATH="$PATH:/julia-1.8.1/bin"' >> ~/.bashrc
RUN source ~/.bashrc

# Install Julia dependencies
RUN julia -e 'import Pkg; Pkg.add(Pkg.PackageSpec(name="Rotations", version="0.13.0"))'
RUN julia -e 'import Pkg; Pkg.add(["YAML", "BenchmarkTools", "ForwardDiff", "Calculus", "ReverseDiff", "StaticArrays", "Flux", "BSON", "NLopt", "Knet", "Random", "RobotOS", "Distributions", "PyCall", "Dates", "LinearAlgebra", "Zygote", "Distances"]) '
ENV PYTHON=/usr/bin/python2.7
RUN julia -e 'import Pkg; Pkg.add("PyCall"); Pkg.build("PyCall")'


# Install python packages
RUN pip install websockets\
    scipy


COPY . /workspace
WORKDIR /workspace/


# Set the default command to execute
CMD ["bash"]



