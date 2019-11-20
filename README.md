# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## Compiling steps
If starting, simply run ./compile

If compiling individual package:
* Ros package (relaxed\_ik, falcon\_ros and panda\_ros): run ` catkin_make --only-pkg-with-deps <package name>`.
* Non-ros package (PandaController): go to the package's build folder and run `make install'


## Running with ROS
1. Run `source devel/setup.sh` inside the PandaFCI directory
2. Start the ros nodes related to the application:
    * Space mouse:
	    - Terminal 1: `roslaunch spacenav_node classic.launch`
	    - Terminal 2: `rosrun panda_ros rosPandaVelocity _control_mode:=cartesian_velocity`


## Running without ROS
Falcon controller running a real-time 1kHz control loop:
1. Move to PandaFCI/devel/bin directory
2. Run `Falcon <robot ip>`
