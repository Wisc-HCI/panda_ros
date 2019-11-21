# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## Compile with ROS
To compile all the ros packages, simply run `./compile` inside the PandaFCI directory

To compile individual ros packages:
* Ros packages (relaxed\_ik, falcon\_ros and panda\_ros): run ` catkin_make --only-pkg-with-deps <package name>`.

## Compile without ROS
To compile non-ros package (PandaController): go to the package's build folder and run `make install`

## Running with ROS
1. Run `source devel/setup.sh` inside the PandaFCI directory
2. Start the ros nodes related to the application:
    * Falcon:
		- Terminal 1: `roslaunch relaxed_ik relaxed_ik_julia.launch`
		- Terminal 2: `rosrun falcon_ros ______`
		- Terminal 3: `rosrun panda_ros rosPandaVelocity -control_mode:=________`
	* Space mouse:
	    - Terminal 1: `roslaunch spacenav_node classic.launch`
	    - Terminal 2: `rosrun panda_ros rosPandaVelocity _control_mode:=cartesian_velocity`


## Running without ROS
Falcon controller running a real-time 1kHz control loop:
1. Move to PandaFCI/devel/bin directory
2. Run `Falcon <robot ip>`
