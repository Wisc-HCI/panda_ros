#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

// Panda
#include "PandaController.h"
#include <franka/robot_state.h>

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

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;
// End FT Sensor Socket

using namespace std;
using namespace std::chrono;
//using namespace libnifalcon;
//using namespace StamperKinematicImpl;

//FalconDevice m_falconDevice;
byte request[8]; /* The request data sent to the Net F/T. */
int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */

array<double,3> workspace_center = {0.42, 0.1, 0.25};
array<double, 3> force_dimension = {0.0, 0.0, 0.0};

std::ofstream outputfile;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
    dhdEnableForce (DHD_OFF);
    cout << "Closing Panda" << endl;
    PandaController::stopControl();
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


    bool buttonPressed = false;

    cout << endl << endl << "(Press Force Dimension Button to Start Control)" << endl;

    // maybe add timeout?
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
}

void poll_forcedimension(bool buttonPressed, bool resetCenter, double velcenterx, double velcentery,double velcenterz) {

    // Scaling Values
    array<double,3> scaling_factors = {-3.0, -3.0, 3.0};

    array<double, 6> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double vx=0.0;
    double vy=0.0;
    double vz=0.0;
    double scaling = 0.001;

    if (buttonPressed){
        array<double, 3> force_dimension_temp = {0.0, 0.0, 0.0};
        dhdGetPosition (&force_dimension_temp[0], &force_dimension_temp[1], &force_dimension_temp[2]);
        panda_pos[0] = scaling_factors[0] * velcenterx + workspace_center[0];
        panda_pos[1] = scaling_factors[1] * velcentery + workspace_center[1];
        panda_pos[2] = scaling_factors[2] * velcenterz + workspace_center[2];

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

    //cout << "FD: " << panda_pos[0]  << "," << panda_pos[1] << "," << panda_pos[2] << endl;
    //cout << "WC: " << workspace_center[0]  << "," << workspace_center[1] << "," << workspace_center[2] << endl;
    PandaController::writeCommandedPosition(panda_pos);

}

void feedback_forcedimension(double *x, double *y, double *z, double *fx, double *fy, double *fz, bool buttonPressed){
    franka::RobotState state = PandaController::readRobotState();
    double scale = -1.0;
    //cout << "Forces: " << state.O_F_ext_hat_K[0]  << "," << state.O_F_ext_hat_K[1] << "," << state.O_F_ext_hat_K[2] << endl;
    double vx;
    double vy;
    double vz;
    double b = 30;
    dhdGetLinearVelocity(&vx,&vy,&vz);
    if (!buttonPressed){
        dhdSetForceAndTorque(-state.O_F_ext_hat_K[0] * scale-vx*b,-state.O_F_ext_hat_K[1] * scale-vy*b,state.O_F_ext_hat_K[2] * scale-vz*b,0.0,0.0,0.0);
    }
    *x = state.O_T_EE[12];
    *y = state.O_T_EE[13];
    *z = state.O_T_EE[14];
    *fx = -state.O_F_ext_hat_K[0];
    *fy = -state.O_F_ext_hat_K[1];
    *fz = -state.O_F_ext_hat_K[2];
}

void log_demonstration(double x, double y, double z, double fx, double fy, double fz){
    outputfile << x << "," << y << "," << z << "," << fx << "," << fy << "," << fz << "\n";
}

void feedback_ft_forcedimension(bool buttonPressed, double velcenterx, double velcentery, double velcenterz,
double *x, double *y, double *z, double *fx, double *fy, double *fz){

    double vx;
    double vy;
    double vz;
    double b = 50; // injected damping
    double scale = -0.4; //force scaling
    double vel_mode_stiffness = 5000;

    double fd_x;
    double fd_y;
    double fd_z;

    dhdGetLinearVelocity(&vx,&vy,&vz);
    dhdGetPosition (&fd_x, &fd_y, &fd_z);
    std::array<double, 6> FTData = PandaController::readFTForces();
    
    Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(FTData.data(),3);

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-M_PI/6, Eigen::Vector3d::UnitZ());
    v = m * v;

    FTData[0] = v[0];
    FTData[1] = v[1];
    FTData[2] = v[2];


    if(!buttonPressed){ 
        dhdSetForceAndTorque(FTData[0] * scale -vx*b,-FTData[1]* scale -vy*b,FTData[2]* scale -vz*b,0.0,0.0,0.0);
    }

    else{
        dhdSetForceAndTorque(0.0,0.0,0.0,0.0,0.0,0.0);
    }
    
    franka::RobotState state = PandaController::readRobotState();
    *x = state.O_T_EE[12];
    *y = state.O_T_EE[13];
    *z = state.O_T_EE[14];
    *fx = -state.O_F_ext_hat_K[0];
    *fy = -state.O_F_ext_hat_K[1];
    *fz = -state.O_F_ext_hat_K[2];

    // TODO: Fix this to be the forces from the actual force torque sensor

    //cout << "FT: " << -FTData[0] * scale -vx*b  << "," << -FTData[1]* scale << "," << FTData[2]* scale -vz*b << endl;

}

int main() {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    //setup_ft();

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


    // Start Panda controller and poll force dimension to make sure it has reasonable starting values
    PandaController::initPandaController();
    

    // Initialize Hybrid Controller
    std::array<double, 3> selectionVector = {1.0, 1.0, 1.0};
    std::array<double, 6> FT_command = {0.0, 0.0, -10.0, 0.0, 0.0, 0.0};
    PandaController::writeSelectionVector(selectionVector);
    PandaController::writeCommandedFT(FT_command);

    poll_forcedimension(false,false,0.0,0.0,0.0);

    // State flow based on button presses
    bool exit=false;
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
    
    buttonPressed = false;
    bool velocity_mode = false;
    auto start = high_resolution_clock::now(); 
    bool gripping = false;
    bool recording = false;
    bool resetCenter = true;

    int file_iter = 0;

    while (PandaController::isRunning()) {
        poll_forcedimension(velocity_mode,resetCenter,velcenterx,velcentery,velcenterz);

        if (resetCenter){ //only allow one correction
            resetCenter=false;
        }
        //feedback_forcedimension(&x,&y,&z,&fx,&fy,&fz,buttonPressed);
        feedback_ft_forcedimension(velocity_mode, velcenterx, velcentery, velcenterz,&x,&y,&z,&fx,&fy,&fz);


        // If button pressed and released in less than 0.3 seconds,
        // it is a gripping action

        // If held greater than 0.3 seconds, it is a velocity control action

        if(!buttonPressed && dhdGetButton(0)==1) // button initially pressed
        {
            buttonPressed = true;
            start = high_resolution_clock::now(); 
            // When starting a potential velocity mode action
            // Need to see where the current center value is
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
                if(!gripping){
                    PandaController:: graspObject();
                    gripping=true;
                }
                else{
                    PandaController::releaseObject();
                    gripping=false;
                }
            }

            cout << "Button Unpressed" << endl;
            buttonPressed=false;
        
            if (velocity_mode==true){
                // Deal with discontinuity
                double tempx;
                double tempy;
                double tempz;
                cout << "DISCONT" << endl;
                resetCenter = true;
                velocity_mode=false;
            }
        }

        if (dhdKbHit()) {
            if (dhdKbGet() == 'r'){
                if(recording==false){
                    string filename = {"panda_demo_"+to_string(file_iter)+".csv"};
                    remove( filename.c_str() );
                    outputfile.open (filename.c_str());
                    cout << "Starting Recording: " << filename.c_str() <<  endl;
                    file_iter++;
                    recording=true;
                }
                else{
                    outputfile.close();
                    cout << "Ending Recording" << endl;
                    recording=false;
                }
            } 
        }

        if (recording){
            log_demonstration(x,y,z,fx,fy,fz);
        }
        


      
    }

    dhdEnableForce (DHD_OFF);
    cout << "Closing Panda" << endl;
    PandaController::stopControl();
    
    return 0;
}
