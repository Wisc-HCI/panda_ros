# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## Compilation:
To compile all the packages, simply run `./compile` inside the PandaFCI directory
### Compile ROS package
Compile individually each ros packages:
* Ros packages (relaxed\_ik, inputs\_ros and panda\_ros): run ` catkin_make --only-pkg-with-deps <package name>`.
### Compile non-ROS package
* PandaController: 
    - If first time, configure:
        - `cd src/PandaController`
        - `mkdir build`
        - `cd build`
        - `cmake ..`
    -  Run:
        - `cd src/PandaController/build`
        - `make install`
        
## Running with ROS
1. Run `source devel/setup.bash` inside the PandaFCI directory
2. Start the launch files related to the application:
    * Falcon:
		- Terminal 1: `roslaunch relaxed_ik relaxed_ik_julia.launch`
		- Terminal 2: `roslaunch inputs_ros falcon.launch`
	* Space mouse:
	    - Terminal 1: `roslaunch inputs_ros space_mouse.launch`

## Running without ROS
Falcon controller running a real-time 1kHz control loop:
1. `source devel/setup.sh`
2. `./devel/bin/Falcon <robot ip>` (if IP blank, will use Panda's ip on HCILab wifi)
