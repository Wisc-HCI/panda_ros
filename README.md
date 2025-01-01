# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## 1. Prequisites

Here is what you need to start with:
* Robot system version: 4.2.X (FER pandas)
* Robot / Gripper Server version: 5 / 3
* Ubuntu 


Here is what we are going to install:
* ROS Noetic
* Libfranka  version 0.9.2
* franka_ros version 0.10.0
* Various apt/ROS packages 


## 2. Setting up this repository
First clone this repository to your local computer and open a terminal in the repository's directory. All further commands should be run from there.
Run the following command to grab the proper submodules:
```bash
git submodule init
git submodule update
```

The following submodules should have been cloned:
- [spacenavd v1.3](https://github.com/FreeSpacenav/spacenavd)

## 3. Setting Up Your Container

First set up display forwarding:
```bash
xhost +local:
```
Now  build the container image and start the container. Make sure you are in this root directory (NIST_Benchmark). These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
```bash
sudo docker build -t panda-prim-controller .

# Start the container with real-time kernel privileges, allow access to usb devices,  mount onto the current directory, and allow display forwarding. Container is removed once it exits.
sudo docker run --rm -it --privileged --device=/dev/input/event* --cap-add=SYS_NICE --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host panda-prim-controller
```

Add necessary environment variables:
Replace with your Panda's IP 
```bash
export PANDA_IP=192.168.1.3
```

## 4. Compilation:

### Compile non-ROS package (PandaController)

If first time, first configure:
```bash
cd src/PandaController
mkdir build
cd build
cmake ..
cd ../../..
```

Anytime, run:
```bash
cd src/PandaController/build
make install
cd ../../..
```

### Compile ROS package
Compile individually each ros packages:
```bash
# catkin build relaxed_ik --no-notify
catkin build panda_ros_msgs --no-notify
catkin build panda_ros --no-notify
#catkin build dmp_deformations --no-notify
catkin build inputs_ros --no-notify
catkin build controller --no-notify
```


## 5. Running with ROS
1. Run `source devel/setup.bash` inside the root directory
2. Start the launch files related to the application:
	* Space mouse:
	    - Terminal 1: 
			```bash
			spacenavd
			roslaunch inputs_ros space_mouse.launch
			```

## Updating to Noetic/Python3 Progress
- [x] Update Dockerfile to Noetic and Python3
- [x] Make sure libraries in Dockerfile still work or upgrade as needed.
- [x] Update libfranka to 0.9.2 (This can be done in DockerFile with apt-get).
- [x] Update franka_ros to 0.10.0 (This version can be done in the DockerFile through apt-get. This version should be compatible but I've only tested 0.8.0 before. If we end up  needing 0.8.0, that will need to be downloaded through source code.)
- [x] Fix any CMAKE issues.
- [X] Check if changes in changelog of libfranka and franka_ros show effect code.
- [X] Update python code necessarey to get Space Mouse working.
- [ ] Upgrade RelaxedIK to RangedIK.
- [ ] Figure out which code is relevent to ros_gui and upgrade that.
- [ ] Possibly delete any remaining outdated/unessesary code.

## Resources
* Noetic/Python3 Migration:
	* https://wiki.ros.org/noetic/Migration
	* https://mil.ufl.edu/docs/software/noetic_migration.html
* SpaceNav ROS:
	* https://github.com/ros-drivers/joystick_drivers/tree/ros1/spacenav_node
* ROS Headers:
	* https://medium.com/@smilesajid14/how-custom-msg-works-in-ros-7d5a14bf5781


## Testing

* To test the space mouse, run each of these commands in their own terminal. The last command will subscribe to the spacemouse topic and the numbers outputted should change as you move the mouse.
	- Terminal 1: 
		```bash
		roscore
		```
	- Terminal 2: 
		```bash
		spacenavd  
		rosrun spacenav_node spacenav_node
		```
	- Terminal 3: 
		```bash
		rostopic echo /spacenav/joy
		```	


