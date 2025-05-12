
# panda_ros
This is a ROS package interface for the C++ Franka Emika Panda [PandaController](https://github.com/Wisc-HCI/PandaController).


## 1. Prequisites

Here is what you need to start with:
* Ubuntu Machine with the [Real Time Kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
	* Static IP of 192.168.1.XXX (Ex/ 192.168.1.5) and Netmask of 255.255.255.0 for the ethernet connected to the Panda. For the force torque sensor, the ethernet connected to that needs to be set to a static IP of 192.168.2.XXX (Ex/ 192.168.2.5).
	* [Docker Engine](https://docs.docker.com/engine/install/)
	* A ROS Workspace with this package clone under `src/`.
    * The following ROS package also cloned under `src/` in your ROS workspace:
        * [PandController](https://github.com/Wisc-HCI/PandaController)
		* [panda_ros_msgs](https://github.com/emmanuel-senft/panda-ros-msgs)
* Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html) and set to static IP of 192.168.1.XXX (Ex/ 192.168.1.3) and Netmask to 255.255.255.0.
	* Robot system version: 4.2.X (FER pandas)
	* Robot / Gripper Server version: 5 / 3
* [Axio80-M20 Force Torque Sensor](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20) installed on the Panda's End Effector and connected to the host computer via ethernet with IP 192.168.2.2 (or change the IP in src/PandaController/src/ForceTorqueListener.cpp).


Here is what we are going to install with docker:
* ROS Noetic
* Libfranka  version 0.9.2
* franka_ros version 0.10.0
* Various apt/ROS packages 


## 2. Setting Up Your Container
This should be run in the root of your ROS Workspace (above the `/src` folder).

First set up display forwarding:
```bash
xhost +local:
```
Now  build the container image and start the container. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
```bash
sudo docker build -f src/panda_ros/Dockerfile  -t panda-controller .

sudo docker run --rm -it --privileged --device=/dev/input/event* --cap-add=SYS_NICE --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host panda-controller
```

Add necessary environment variables:
Replace with your Panda's IP 
```bash
export PANDA_IP=192.168.1.3
```

## 3. Compilation:

### Compile non-ROS package (PandaController)

If first time, first configure:
```bash
cd src/PandaController/
mkdir -p build
cd build
[ -f Makefile ] || cmake ..  # Only initialize if not  already
make install
cd ../../..
```

### Compile ROS package
Compile individually each ros packages:
```bash
catkin build
```


## 5. Running with ROS
1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
2. Run `source devel/setup.bash` inside the root directory
3. Start the launch files related to the application:
	```bash
	roslaunch panda_ros mover.launch
	```


## Resources
* Noetic/Python3 Migration:
	* https://wiki.ros.org/noetic/Migration
	* https://mil.ufl.edu/docs/software/noetic_migration.html
* ROS Headers:
	* https://medium.com/@smilesajid14/how-custom-msg-works-in-ros-7d5a14bf5781
* Force Torque Sensor:
	* https://www.ati-ia.com/app_content/Documents/9610-05-Ethernet%20Axia80.pdf
	* https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20
* Force Dimension Input:
	* https://www.forcedimension.com/software/sdk



## Troubleshooting/Testing

* To test if the Force/Torque Sensor is properly connected, go to the sensor's IP (ex/ 192.168.1.6) in a webrowser on your computer. If the ATI Configuration page shows up, that means you are properly connected. If not, reference section 4 of the [Sensor Guide](https://www.ati-ia.com/app_content/Documents/9610-05-Ethernet%20Axia80.pdf). Once you can access the ATI site, you can go to Demo and download the Java application to see the values coming from the sensor.

