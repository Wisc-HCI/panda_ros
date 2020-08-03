#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

// ROS
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

// Force Dimension
#include "dhdc.h"
#include "drdc.h"

using namespace std;
using namespace std::chrono;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension_Deformation_Emmanuel.cpp\n";
    dhdEnableForce (DHD_OFF);
    dhdClose();
    exit(sig);
}

bool init_forcedimension() {
    cout <<"Setting up Force Dimension\n";
    if (dhdOpen () < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (2.0);
        return false;
    }
    cout << "Force Dimension Setup Complete" << endl;
}

void fd_neutral_position(double *x_curr, double *y_curr, double *z_curr){
    // Zero displacement desired
    double x_d = 0.0; double y_d = 0.0; double z_d = 0.0;
    double v_x, v_y, v_z;

    // Overdamped PD
    double stiffness = 200;
    double damping = 2*sqrt(stiffness);

    // Get current position and velocity
    dhdGetPosition (x_curr, y_curr, z_curr);
    dhdGetLinearVelocity(&v_x,&v_y,&v_z);
    
    // Command back to the center position with PD controller
    dhdSetForceAndTorque(-stiffness*(*x_curr-x_d)-damping*v_x,-stiffness*(*y_curr-y_d)-damping*v_y,
    -stiffness*(*z_curr-z_d)-damping*v_z,0.0,0.0,0.0);
}

int main(int argc, char **argv) {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    // Set up ROS
    ros::init(argc, argv, "ForceDimensionDeformation");
    ros::NodeHandle n("~");

    //Publisher for deformations and button press 
    ros::Publisher deformation_pub = 
        n.advertise<geometry_msgs::Twist>("/fd/twist", 5);
    
    ros::Publisher button_pub = 
        n.advertise<std_msgs::Bool>("/fd/button", 5);

    if (!init_forcedimension()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    double def_x, def_y, def_z;
    geometry_msgs::Twist deformation;
    std_msgs::Bool button_pressed;

    // Only 3 DOF for now
    deformation.angular.x = 0.0;
    deformation.angular.y = 0.0;
    deformation.angular.z = 0.0;

    while(ros::ok())
    {
        // Send current button
        button_pressed.data = dhdGetButton(0);
        button_pub.publish(button_pressed);

        // Deformation iteration
        fd_neutral_position(&def_x,&def_y,&def_z);
        deformation.linear.x = def_x; deformation.linear.y = def_y; deformation.linear.z = def_z;
        deformation_pub.publish(deformation);
        ros::spinOnce();
    }

    dhdEnableForce (DHD_OFF);
    dhdClose();
    return 0;
}
