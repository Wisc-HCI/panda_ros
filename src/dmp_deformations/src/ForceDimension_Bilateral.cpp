#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

 #include <ros/ros.h>
 #include "std_msgs/String.h"
 #include <panda_ros_msgs/HybridPose.h>

// Force Dimension
#include "dhdc.h"

// Socket Libraries for FT sensor readings
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h> 
#include <eigen3/Eigen/Geometry>
#include <thread> 


using namespace std;
using namespace std::chrono;

array<double,3> workspace_center = {0.42, 0.1, 0.25};
array<double, 3> force_dimension = {0.0, 0.0, 0.0};
array<double, 3> actual_pos;

std::ofstream outputfile;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
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

void actualPose(geometry_msgs::Pose pose) {
    actual_pos[0] = pose.position.x;
    actual_pos[1] = pose.position.y;
    actual_pos[2] = pose.position.z;
}

void publishPose(ros::Publisher hybrid_pub, std::array<double, 7> panda_pose) {
    panda_ros_msgs::HybridPose hybridPose;
    hybridPose.pose.position.x = panda_pose[0];
    hybridPose.pose.position.y = panda_pose[1];
    hybridPose.pose.position.z = panda_pose[2];
    hybridPose.pose.orientation.x = panda_pose[3];
    hybridPose.pose.orientation.y = panda_pose[4];
    hybridPose.pose.orientation.z = panda_pose[5];
    hybridPose.pose.orientation.w = panda_pose[6];
    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = 0.0;
    hybridPose.constraint_frame.x=0.0;
    hybridPose.constraint_frame.y=0.0;
    hybridPose.constraint_frame.z=0.0;
    hybridPose.constraint_frame.w=1.0;
    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};
    hybrid_pub.publish(hybridPose);
}

void poll_forcedimension(ros::Publisher hybrid_pub, bool buttonPressed, bool resetCenter, double velcenterx, double velcentery,double velcenterz) {

    // Scaling Values
    array<double,3> scaling_factors = {-4.0, -4.0, 4.0};

    array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    
    double vx=0.0;
    double vy=0.0;
    double vz=0.0;
    double scaling = 0.001;

    if (buttonPressed){
        array<double, 3> force_dimension_temp = {0.0, 0.0, 0.0};
        dhdGetPosition (&force_dimension_temp[0], &force_dimension_temp[1], &force_dimension_temp[2]);
        workspace_center[0]=workspace_center[0]-scaling*(force_dimension_temp[0]-velcenterx);
        workspace_center[1]=workspace_center[1]-scaling*(force_dimension_temp[1]-velcentery);
        workspace_center[2]=workspace_center[2]+scaling*(force_dimension_temp[2]-velcenterz);

    }

    else{
        dhdGetPosition (&force_dimension[0], &force_dimension[1], &force_dimension[2]);
        if(resetCenter){
            cout << "Reset Center" << endl;
            workspace_center[0]=workspace_center[0]-scaling_factors[0]*(force_dimension[0]-velcenterx);
            workspace_center[1]=workspace_center[1]-scaling_factors[1]*(force_dimension[1]-velcentery);
            workspace_center[2]=workspace_center[2]-scaling_factors[2]*(force_dimension[2]-velcenterz);
        }
    }

    panda_pos[0] = scaling_factors[0] * force_dimension[0] + workspace_center[0];
    panda_pos[1] = scaling_factors[1] * force_dimension[1] + workspace_center[1];
    panda_pos[2] = scaling_factors[2] * force_dimension[2] + workspace_center[2];

    publishPose(hybrid_pub, panda_pos);

}

void feedbackFD(geometry_msgs::Wrench wrench) {
    double scale = 0.1; // force reflection
    double stiffness = 200; // for replay
    double viscous = 50; // friction

    array<double, 3> forceDimensionPos = {0,0,0};
    array<double, 3> forceDimensionVel = {0,0,0};
    dhdGetLinearVelocity(&forceDimensionVel[0],&forceDimensionVel[1],&forceDimensionVel[2]);
    // Send force (bilateral + friction) to the falcon
    dhdSetForceAndTorque(-wrench.force.x * scale-viscous*forceDimensionVel[0], 
            -wrench.force.y * scale-viscous*forceDimensionVel[1], 
            -wrench.force.z * scale-viscous*forceDimensionVel[2],0.0,0.0,0.0);
}

int main(int argc, char **argv) {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    // All of the required ros topics
    ros::init(argc, argv, "ForceDimensionDMP");
    ros::NodeHandle n("~");  
    ros::Publisher gripper_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackFD);
    ros::Subscriber actual_pose_sub = n.subscribe("/panda/pose_actual", 10, actualPose);
    ros::Publisher hybrid_pub = 
        n.advertise<panda_ros_msgs::HybridPose>("/panda/hybrid_pose", 1); 

    if (!init_forcedimension()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button again to enable Forces and Panda Control" << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    poll_forcedimension(hybrid_pub,false,false,0.0,0.0,0.0);

    double velcenterx = 0;
    double velcentery = 0;
    double velcenterz = 0;
    
    buttonPressed = false;
    bool velocity_mode = false;
    auto start = high_resolution_clock::now(); 
    
    bool quit = false;


    bool resetCenter = true;

    while (ros::ok() && !quit) {
        poll_forcedimension(hybrid_pub, velocity_mode,resetCenter, velcenterx,velcentery,velcenterz);

        if (resetCenter){ //only allow one correction
            resetCenter=false;
        }
    
        // If button pressed and released in less than 0.3 seconds,
        // it is a gripping action

        // If held greater than 0.3 seconds, it is a velocity control action

        if(!buttonPressed && dhdGetButton(0)==1) // button initially pressed
        {
            buttonPressed = true;
            start = high_resolution_clock::now(); 
        }

        
        if(buttonPressed) // button held
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()>=300000)
            {
                if (velocity_mode==false)
                {
                    velocity_mode=true;
                    // When starting a potential velocity mode action
                    // Need to see where the current center value is
                    dhdGetPosition (&velcenterx, &velcentery, &velcenterz);
                    cout << "Velocity Mode" << endl;
                }
               
            }
            
        }

        if(buttonPressed && dhdGetButton(0)==0) // button released
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()<300000)
            {
                cout << "Toggle Gripper" << endl;
                std_msgs::String temp_str;
                temp_str.data = "toggleGrip";
                gripper_pub.publish(temp_str);
            }

            buttonPressed=false;
        
            if (velocity_mode==true){
                // Deal with discontinuity
                double tempx;
                double tempy;
                double tempz;
                resetCenter=true;

                velocity_mode=false;
            }
        }

        if (dhdKbHit()) {
            char keypress = dhdKbGet();
            if (keypress == 'q'){
                cout << "Quitting! " << endl;
                quit = true;
                } 
        }
    ros::spinOnce();
    usleep(1000);
    }

    dhdEnableForce (DHD_OFF);
    dhdClose();
    return 0;
}
