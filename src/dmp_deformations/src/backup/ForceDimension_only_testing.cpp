#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include "std_msgs/String.h"


// Force Dimension
#include "dhdc.h"
#include "drdc.h"

#include <thread> 


using namespace std;
using namespace std::chrono;

array<double,3> workspace_center = {0.42, 0.1, 0.25};
array<double, 3> force_dimension = {0.0, 0.0, 0.0};

std::ofstream outputfile;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
    dhdEnableForce (DHD_OFF);
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


void fd_neutral_position(){
    
    double x_d = 0.0125;
    double y_d = 0.0;
    double z_d = 0.025;

    double v_x, v_y, v_z;
    double x_curr, y_curr, z_curr;

    double stiffness = 200;
    double damping = 2*sqrt(stiffness);

    // Get current position and velocity
    dhdGetPosition (&x_curr, &y_curr, &z_curr);
    dhdGetLinearVelocity(&v_x,&v_y,&v_z);

    cout << x_curr/0.0625 << " " << y_curr/0.12 << " " << z_curr/0.105 << endl;
    
    // Command to that position with P controller
    dhdSetForceAndTorque(-stiffness*(x_curr-x_d)-damping*v_x,-stiffness*(y_curr-y_d)-damping*v_y,
    -stiffness*(z_curr-z_d)-damping*v_z,0.0,0.0,0.0);

}


void poll_forcedimension(bool buttonPressed, bool resetCenter, double velcenterx, double velcentery,double velcenterz) {
    // Scaling Values
    array<double,3> scaling_factors = {-4.0, -4.0, 4.0};

    array<double, 6> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
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

    cout << "FD: " << force_dimension[0]  << "," << force_dimension[1] << "," << force_dimension[2] << endl;
    //cout << "WC: " << workspace_center[0]  << "," << workspace_center[1] << "," << workspace_center[2] << endl;
}


int main(int argc, char **argv) {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "ForceDimensionDMP");
    ros::NodeHandle n("~");  
    if (!init_forcedimension()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }


    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // State flow based on button presses
    bool start_recording = false;

    // Initialize data for storage
    double x = 0;
    double y = 0;
    double z = 0;
    double fx = 0;
    double fy = 0;
    double fz = 0;

    double velcenterx = 0;
    double velcentery = 0;
    double velcenterz = 0;
    
    bool buttonPressed = false;
    bool velocity_mode = false;
    auto start = high_resolution_clock::now(); 
    bool gripping = false;
    bool recording = false;
    
    bool replayMode = false;

    int file_iter = 0;

    bool resetCenter = true;

    while (ros::ok() && !replayMode) {
        poll_forcedimension(velocity_mode,resetCenter, velcenterx,velcentery,velcenterz);

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
                cout << "GRIPPER ACTION" << endl;
            }

            cout << "Button Unpressed" << endl;
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
            if (keypress == 'p'){
                cout << "Initiating Replay " << endl;
                replayMode = true;
                } 
        }
    ros::spinOnce();

    }

    if(replayMode)
    {
        // Replay loop if the correct key has been pressed!
        cout << "REPLAY MODE" << endl;

        while(ros::ok() && replayMode){
            
            fd_neutral_position();
            if (dhdKbHit()) {
                
                char keypress = dhdKbGet();

                if (keypress == 'q'){
                    replayMode=false;
                }

            }
        }
        
    }

    dhdEnableForce (DHD_OFF);
    dhdClose();
    
    return 0;
}
