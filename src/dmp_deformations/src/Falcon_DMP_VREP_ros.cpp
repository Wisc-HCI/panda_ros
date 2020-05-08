#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <relaxed_ik/EEPoseGoals.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/String.h"

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;

FalconDevice m_falconDevice;

// For calculating the falcon velocity
array<double, 3> falcon_vel = {0.0, 0.0, 0.0};
std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
std::array<double, 3> last_falcon = {0.0, 0.0, 0.0};

bool last_falcon_updated = false;

// For recording
std::ofstream outputfile;
double x, y, z, fx, fy, fz;
double dmp_fx, dmp_fy, dmp_fz;

bool replay_active = false;

// This writes all of the state information to the open CSV file
void log_demonstration(double x, double y, double z, double fx, double fy, double fz){
    outputfile << x << "," << y << "," << z << "," << fx << "," << fy << "," << fz << "\n";
}


void falconVelocity() {
    double delta_T = 1000;
    // Compute the velocity to add some viscous friction to help with stability
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    // Provide there is a last position to compute the velocity, calculate with backwards differencing
    if(last_falcon_updated)
    {
        // Velocity calculated as basic differencing
        falcon_vel = {(falconPos[0]-last_falcon[0])/delta_T,(falconPos[1]-last_falcon[1])/delta_T,(falconPos[2]-last_falcon[2])/delta_T};
    }

    //Update last falcon  for use in the velocity calculation
    last_falcon[0] = falconPos[0];
    last_falcon[1] = falconPos[1];
    last_falcon[2] = falconPos[2];

    last_falcon_updated = true;
}


void readDemo(vector<vector<array<double,3>>> &dmps,vector<array<double,3>> &selections,vector<array<double,3>> &starting_points,vector<array<double,3>> &attractor_points){
    std::ifstream dmpfile("learneddmp.csv");
    double junk;
    string temp;

    vector<array<double,3>> dmp_temp;
    // Read entire file into the vectors
    if(dmpfile.good())
    {
        while(getline(dmpfile,temp,',')){
            // Either a new mode definition or continued values
            if(temp=="mode")
            {
                // New mode, need to create vector
                // for trajectory, get selection and
                // starting/attractor points
                // maybe make this the DMP stiffness and damping?
                getline(dmpfile,temp,',');
                junk = atof(temp.c_str());
                getline(dmpfile,temp);
                junk = atof(temp.c_str());

                // Read Selection Vector
                array<double,3> selection_temp;

                getline(dmpfile,temp,','); selection_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); selection_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp); selection_temp[2] = atof(temp.c_str());
                selections.push_back(selection_temp);

                // Read Starting Points
                array<double,3> starting_temp;
                getline(dmpfile,temp,','); starting_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp); starting_temp[2] = atof(temp.c_str());
                starting_points.push_back(starting_temp);

                // Read Attractor Points
                array<double,3> attractor_temp;
                getline(dmpfile,temp,','); attractor_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp); attractor_temp[2] = atof(temp.c_str());
                attractor_points.push_back(attractor_temp);

                // Send the previous DMP if it has something in it
                // this avoids sending an empty DMP at the beginning
                if(dmp_temp.size()>0)
                {
                    dmps.push_back(dmp_temp);
                }

                // Reinitialize the temporary vector
                // to store the trajectory
                dmp_temp.clear();
            }

            else{ // Add new value to dmp_vector
                array<double,3> state_vector_temp;
                // temp has first value already
                state_vector_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp);
                state_vector_temp[2] = atof(temp.c_str());
                dmp_temp.push_back(state_vector_temp);
            }

        }

        // Publish the final dmp
        if(dmp_temp.size()>0)
        {
            dmps.push_back(dmp_temp);
        }


    }

    //close the filestream
    dmpfile.close();
}

void replay_demo(ros::Publisher selection_vector_pub, ros::Publisher wrench_goal_pub, ros::Publisher pose_goal_pub, ros::Publisher pose_path_pub, ros::Publisher dmp_replay_pub){

    std_msgs::String replay_str;

    // All of the data from the DMPs is stored in 
    vector<vector<array<double,3>>> dmps;
    vector<array<double,3>> selections;
    vector<array<double,3>> starting_points;
    vector<array<double,3>> attractor_points;

    geometry_msgs::Pose pose;
    geometry_msgs::Wrench ft;
    geometry_msgs::Vector3 selection;
    
    // DMP Parameters - TODO: READ THESE FROM THE FILE
    double k=50;
    double b=sqrt(2*k);
    readDemo(dmps,selections,starting_points,attractor_points);

    // Action: Tell the robot the replay is starting over
    replay_str.data = "reset";
    dmp_replay_pub.publish(replay_str);

    // Tell the robot to go to the overall starting point
    // Pose path pub will interpolate the path
        pose.position.x = starting_points[0][0];
        pose.position.y = starting_points[0][1];
        pose.position.z = starting_points[0][2];
        pose_path_pub.publish(pose);
        
        // Sleep for 2.5 seconds to allow path completion
        for(int jj=0; jj<2500; jj++)
        {
            ros::spinOnce();
            falconVelocity();
            usleep(1000);
        }

    // Action: Tell the robot the replay is starting
    cout << "Replay Starting..." << endl;
    replay_str.data = "start";
    dmp_replay_pub.publish(replay_str);   

    // Variables needed for transitions and deformation collision-limiting
    bool previous_dmp_no_contact = true;
    double transition_x=0.0, transition_y=0.0, transition_z=0.0;
    bool dmp_x_limiting;
    bool dmp_y_limiting;
    bool dmp_z_limiting;
    double dmp_x_collision = 0.0;
    double dmp_y_collision = 0.0;
    double dmp_z_collision = 0.0;

    // Loop through to replay the demo
    for(int ii = 0; ii<selections.size();ii++)
    {
        // First, publish selection vector
        selection.x =selections[ii][0];
        selection.y =selections[ii][1];
        selection.z =selections[ii][2];
        selection_vector_pub.publish(selection);

        // If selection has force control, need force onloading
        if((selection.x==0 || selection.y==0 || selection.z==0)){
            if(previous_dmp_no_contact){
                // Do force onloading with the first sample
                cout << "FORCE ONLOADING STARTED" << endl;
                ft.force.x = starting_points[ii][0];
                ft.force.y = starting_points[ii][1];
                ft.force.z = starting_points[ii][2];
                pose.position.x = starting_points[ii][0];
                pose.position.y = starting_points[ii][1];
                pose.position.z = starting_points[ii][2];
                pose_goal_pub.publish(pose);
                wrench_goal_pub.publish(ft);

                bool proper_contact = false;
                while(!proper_contact)
                {
                    // TODO - MAKE THIS MORE GENERAL!!!!
                    cout << "FZ: " << fz << " " << starting_points[ii][2] << endl;
                    if(fz<0.95*starting_points[ii][2] && fz>1.05*starting_points[ii][2])
                    {
                        proper_contact = true;
                    }
                    
                    for (int yy=0; yy<10; yy++)
                    {
                        falconVelocity();
                        usleep(1000);
                    }
                    ros::spinOnce();

                }
                cout << "FORCE ONLOADING COMPLETE" << endl;
            }
            previous_dmp_no_contact = false;
        }

        else{
            previous_dmp_no_contact = true;
        }

        // This is used to contact the "Nearest contact condition" to consider
        // minimizing the deformations
        dmp_x_limiting = false;
        dmp_y_limiting = false;
        dmp_z_limiting = false;

        // There is a next DMP, check for pos to force transition
        if (ii+1<selections.size()){
            if(selections[ii][0] ==1 && selections[ii+1][0]==0){
                dmp_x_collision = attractor_points[ii][0];
                dmp_x_limiting=true;
            }
            if(selections[ii][1] ==1 && selections[ii+1][1]==0){
                dmp_y_collision = attractor_points[ii][1];
                dmp_y_limiting=true;
            }
            if(selections[ii][2] ==1 && selections[ii+1][2]==0){
                dmp_z_collision = attractor_points[ii][2];
                dmp_z_limiting=true;
            }
        }

        // There is a previous DMP, check for force to pos transition
        if (ii>0){
            if(selections[ii-1][0] ==0 && selections[ii][0]==1){
                dmp_x_collision = starting_points[ii][0];
                dmp_x_limiting=true;
            }
            if(selections[ii-1][1] ==0 && selections[ii][1]==1){
                dmp_y_collision = starting_points[ii][1];
                dmp_y_limiting=true;
            }
            if(selections[ii-1][2] ==0 && selections[ii][2]==1){
                dmp_z_collision = starting_points[ii][2];
                dmp_z_limiting=true;
            }
        }

        // Set up the values for the new DMP
        double ddx, ddy, ddz;
        double dx = 0.0, dy = 0.0, dz = 0.0;
        double x=starting_points[ii][0]+transition_x, y=starting_points[ii][1]+transition_y, z=starting_points[ii][2]+transition_z;
        
        // For the general impedance model
        double ddx_imp, ddy_imp, ddz_imp;
        double dx_imp = 0.0, dy_imp=0.0, dz_imp=0.0;
        double x_imp = starting_points[ii][0], y_imp=starting_points[ii][1], z_imp=starting_points[ii][2];
        double x_def = 0.0, y_def=0.0, z_def=0.0;
        
        double s = 0; // phase variable used for time
        double delta_s = 1.0;

        // Now replay the entirety of the demonstration
        while(s<dmps[ii].size()){
            ros::spinOnce();

            // Interpolate each of the applied forces for the non-integer value s
            double dmp_x = dmps[ii][(int)floor(s)][0]+(dmps[ii][(int)ceil(s)][0]-dmps[ii][(int)floor(s)][0])*(s-floor(s));
            double dmp_y = dmps[ii][(int)floor(s)][1]+(dmps[ii][(int)ceil(s)][1]-dmps[ii][(int)floor(s)][1])*(s-floor(s));
            double dmp_z = dmps[ii][(int)floor(s)][2]+(dmps[ii][(int)ceil(s)][2]-dmps[ii][(int)floor(s)][2])*(s-floor(s));

            // Compute a potential DMP diminishing scale based on collisions
            double dmp_scaling_x = 1.0;
            double dmp_scaling_y = 1.0;
            double dmp_scaling_z = 1.0;
            if(dmp_x_limiting) dmp_scaling_x = 1-exp(-200*abs(x-dmp_x_collision));
            if(dmp_y_limiting) dmp_scaling_y = 1-exp(-200*abs(y-dmp_y_collision));
            if(dmp_z_limiting) dmp_scaling_z = 1-exp(-200*abs(z-dmp_z_collision));
            //cout << "DMP scaling Z: " << dmp_scaling_z << endl;

            
            // Force mode gain, Position mode gain
            std::array<double, 2> sel_gains = {6000, 150};

            bool dmp_integration=true;

            /////////////////////////////////////////////////////////
            // DMP Integration Method                              //
            /////////////////////////////////////////////////////////
            if(dmp_integration)
            {
                // Calculate New X (State 1)
                ddx = k*(attractor_points[ii][0]-x)-b*dx+dmp_x+sel_gains[(int) selection.x]*dmp_scaling_x*dmp_fx;
                dx = dx + ddx*0.01*delta_s;
                x = x+dx*0.01*delta_s;

                // Calculate New Y (State 2)
                ddy = k*(attractor_points[ii][1]-y)-b*dy+dmp_y+sel_gains[(int) selection.y]*dmp_scaling_y*dmp_fy;
                dy = dy + ddy*0.01*delta_s;
                y = y+dy*0.01*delta_s;

                // Calculate New Z (State 3)
                ddz = k*(attractor_points[ii][2]-z)-b*dz+dmp_z+sel_gains[(int) selection.z]*dmp_scaling_z*dmp_fz;
                dz = dz + ddz*0.01*delta_s;
                z = z+dz*0.01*delta_s;

                // Compute the new velocity factor and
                // Update the time variable
                // TODO: Maybe use a covariance to try and remove units?
                double dir_x = dx/sqrt(dx*dx+dy*dy);
                double dir_y = dy/sqrt(dx*dx+dy*dy);
                double dp_in_dir = dir_x*20*dmp_fx + dir_y*20*dmp_fy;
                
                if(dp_in_dir > 0)
                {
                    dp_in_dir=0.0; // only allow slowing down
                }
                delta_s = 1.0+dp_in_dir;
                s+=delta_s;
                //cout << "deltaS:" << delta_s << endl;
            }

            /////////////////////////////////////////////////////////
            // Impedance Method                                    //
            /////////////////////////////////////////////////////////

            else{
                // Still uses DMP, but the deformation is treated as an
                // overdamped deviation from the current state variable
                // TODO: make that true

                // Force mode gain, Position mode gain
                std::array<double, 2> imp_gains = {200, 5};

                // Calculate New X (State 1)
                ddx = k*(attractor_points[ii][0]-x_imp)-b*dx+dmp_x;
                dx = dx + ddx*0.01*delta_s;
                x_imp = x_imp+dx*0.01*delta_s;
                
                ddx_imp = k*(-x_def)-b*dx_imp+sel_gains[(int) selection.x]*dmp_fx;
                dx_imp = dx_imp + ddx_imp*0.01*delta_s;
                x_def = x_def + dx_imp*0.01*delta_s;

                x = x_imp+x_def; 

                // Calculate New Y (State 2)
                ddy = k*(attractor_points[ii][1]-y_imp)-b*dy+dmp_y;
                dy = dy + ddy*0.01*delta_s;
                y_imp = y_imp+dy*0.01*delta_s;
                
                ddy_imp = k*(-y_def)-b*dy_imp+sel_gains[(int) selection.y]*dmp_fy;
                dy_imp = dy_imp + ddy_imp*0.01*delta_s;
                y_def = y_def + dy_imp*0.01*delta_s;

                y = y_imp+y_def; 

                // Calculate New Z (State 3)
                ddz = k*(attractor_points[ii][2]-z_imp)-b*dz+dmp_z;
                dz = dz + ddz*0.01*delta_s;
                z_imp = z_imp+dz*0.01*delta_s;
                
                ddz_imp = k*(-z_def)-b*dz_imp+sel_gains[(int) selection.z]*dmp_fz;
                dz_imp = dz_imp + ddz_imp*0.01*delta_s;
                z_def = z_def + dz_imp*0.01*delta_s;

                z = z_imp+z_def; 
                s+=1; // No variable time scaling
            }
            
            //cout << " DEF X:" << sel_gains[(int) selection.x]*dmp_fx << " Y:" << sel_gains[(int) selection.y]*dmp_fy << " F:" << sel_gains[(int) selection.z]*dmp_fz << endl;

            // Publish everything to the simulation
            ft.force.x = x;
            ft.force.y = y;
            ft.force.z = z;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose_goal_pub.publish(pose);
            wrench_goal_pub.publish(ft);
            
            // Pause for 0.01 seconds, but calculate velocities at 0.001s
            for (int yy=0; yy<10; yy++)
            {
                falconVelocity();
                usleep(1000);
            }
        }

        // Combat the transition discontinuities if there is still a transition left
        if((ii+1)<selections.size()){
            transition_x = (x-attractor_points[ii][0])*(double)(selections[ii][0]==selections[ii+1][0]);
            transition_y = (y-attractor_points[ii][1])*(double)(selections[ii][1]==selections[ii+1][1]);;
            transition_z = (z-attractor_points[ii][2])*(double)(selections[ii][2]==selections[ii+1][2]);;
        }
    }

    replay_str.data = "end";
    dmp_replay_pub.publish(replay_str);
}
 
bool init_falcon() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware

    if(!m_falconDevice.open(0)) //Open falcon @ index
    {
        cout << "Failed to find Falcon\n";
        return false;
    }
    else
    {
        cout << "Falcon Found\n";
    }

    bool skip_checksum = false;
    bool firmware_loaded = false;
    
    // MH: forcing the firmware to reload seems to solve the issue where
    // we had to keep re-plugging it in

    if(!firmware_loaded)
    {
        cout << "Loading firmware\n";

        uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
        long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

        for(int i = 0; i < 50; ++i)	//Attempt to load firmware 50 times
        {
            if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, firmware_size, firmware_block))
            {
                cout << "Firmware loading try failed\n";
            }
            else
            {
                firmware_loaded = true;
                break;
            }
        }
    }

    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue\n";
        return false;
    }
    cout << "Firmware loaded\n";

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set\n";

    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    usleep(100000);
    
    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) {
            continue;
        }
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in.\n";
            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
            cout << "Falcon homed.\n";
            stop = true;
        }
    }
    m_falconDevice.setFalconGrip<libnifalcon::FalconGripFourButton>();

    m_falconDevice.runIOLoop();
    
    return true;
}

void publishPose(ros::Publisher pose_goal_pub, std::array<double, 7> panda_pose) {
    geometry_msgs::Pose pose_goal;
    pose_goal.position.x = panda_pose[0];
    pose_goal.position.y = panda_pose[1];
    pose_goal.position.z = panda_pose[2];
    pose_goal.orientation.x = panda_pose[3];
    pose_goal.orientation.y = panda_pose[4];
    pose_goal.orientation.z = panda_pose[5];
    pose_goal.orientation.w = panda_pose[6];
    pose_goal_pub.publish(pose_goal);
}

void pollFalcon(ros::Publisher pose_goal_pub, ros::Publisher command_pub, double* scaling_factors, double* offsets, bool* clutch, bool* reset_center, bool* freeze) {
    static bool lastCenterButton = false;
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();



    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 3> normalized_falcon = {0.0, 0.0, 0.0};
    
    // These convert the falcon to match the respective directions on the Kinova!
    normalized_falcon[0] = falconPos[0];
    normalized_falcon[1] = falconPos[2];
    normalized_falcon[2] = falconPos[1];

    // The first time the clutch is initiated, freeze the position of the falcon
    if (*freeze)
    {
        frozen_position[0] = normalized_falcon[0];
        frozen_position[1] = normalized_falcon[1];
        frozen_position[2] = normalized_falcon[2];
        *freeze = false;
    }

    // If still clutching, keep the position consistent based on the frozen position
    if(*clutch)
    {
        panda_pos[0] = scaling_factors[0] * frozen_position[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * frozen_position[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * frozen_position[2] + offsets[2];
        panda_pos[6] = 1;
    }

    else{ // Not clutching

        // If this is the first non-clutching sample, update the center of the workspace
        // based on the movement during the clutching action
        if(*reset_center)
        {
            offsets[0] = offsets[0]-scaling_factors[0]*(normalized_falcon[0]-frozen_position[0]);
            offsets[1] = offsets[1]-scaling_factors[1]*(normalized_falcon[1]-frozen_position[1]);
            offsets[2] = offsets[2]-scaling_factors[2]*(normalized_falcon[2]-frozen_position[2]);
            *reset_center = false;
        }

        // When not clutching, command based on the actual falcon position
        panda_pos[0] = scaling_factors[0] * normalized_falcon[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * normalized_falcon[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * normalized_falcon[2] + offsets[2];
        panda_pos[6] = 1;

        // For recording
        x = panda_pos[0];
        y = panda_pos[1];
        z = panda_pos[2];
    }
    
    publishPose(pose_goal_pub, panda_pos);
}


void feedbackFalcon(geometry_msgs::Wrench wrench) {
    double scale = 0.1; // force reflection
    double viscous = 50; // friction
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    double delta_T = 0.001;

    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    if(!replay_active){
    // Send force (bilateral + friction) to the falcon
    m_falconDevice.setForce({
                -wrench.force.x * scale-viscous*falcon_vel[0], 
                -wrench.force.z * scale-viscous*falcon_vel[1], 
                -wrench.force.y * scale-viscous*falcon_vel[2]});
    }

    else{
        // zero displacement mode
        // falcon has offset in z
        m_falconDevice.setForce({
                -stiffness*falconPos[0]-viscous_replay*falcon_vel[0], 
                -stiffness*falconPos[1]-viscous_replay*falcon_vel[1], 
                -stiffness*(falconPos[2]-0.1)-viscous_replay*falcon_vel[2]});
    }

    // For recording and hybrid replay
    fx = wrench.force.x;
    fy = wrench.force.y;
    fz = wrench.force.z;

    // Store forcing from falcon for deformations
    dmp_fx = -(falconPos[0]);
    dmp_fy = (falconPos[2]-0.1);
    dmp_fz = falconPos[1];


}

int main(int argc, char **argv) {    
    ros::init(argc, argv, "Falcon");
    ros::NodeHandle n("~");  
    std::vector<double> scaling_factors = {-6.0, 6.0, 6.0};
    std::vector<double> offsets = {-0.249, -0.60, 1.4110};
    //n.getParam("offsets",offsets);
    //n.getParam("scaling_factors",scaling_factors);

    if (!init_falcon()) {
        cout << "Failed to init falcon" << endl;
        return -1;
    }
    
    ros::Publisher pose_goal_pub = 
        n.advertise<geometry_msgs::Pose>("/panda/ee_pose_goals", 5);

     ros::Publisher pose_path_pub = 
        n.advertise<geometry_msgs::Pose>("/panda/ee_path_goals", 5);    

    ros::Publisher wrench_goal_pub = 
        n.advertise<geometry_msgs::Wrench>("/panda/ee_wrench_goals", 5);

    ros::Publisher selection_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/selection", 5);

    ros::Publisher command_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackFalcon);

    ros::Publisher file_pub = 
        n.advertise<std_msgs::String>("/dmp/filepub", 5);
    ros::Publisher reset_pub = 
        n.advertise<std_msgs::String>("/dmp/reset", 5);

    ros::Publisher gripper_toggle = 
        n.advertise<std_msgs::String>("/gripperToggle", 1);

    ros::Publisher dmp_replay_pub = 
        n.advertise<std_msgs::String>("/dmp/replay", 5);

    bool last_buttons[4] = {false, false, false, false};
    bool clutch = false;
    bool reset_center = false;
    bool freeze = false;
    bool recording = false;
    int file_iter = 0;
    bool replay_mode = false;
    
    bool timing_press = false;
    chrono::time_point<chrono::steady_clock> start_timer;
    chrono::steady_clock sc;

    while(ros::ok() && (!replay_mode)){      
        // Check button presses
        unsigned int my_buttons = m_falconDevice.getFalconGrip()->getDigitalInputs();
        bool buttons[4];
        // Clutch is center button
        buttons[0] = (my_buttons & libnifalcon::FalconGripFourButton::CENTER_BUTTON)  ? 1 : 0;
        // Change between record and replay mode with plus button
        buttons[1] = (my_buttons & libnifalcon::FalconGripFourButton::PLUS_BUTTON)    ? 1 : 0;
        // Reset DMP with minus button (arrow)
        buttons[2] = (my_buttons & libnifalcon::FalconGripFourButton::MINUS_BUTTON)   ? 1 : 0;
        // Record DMP with forward button (lightning)
        buttons[3] = (my_buttons & libnifalcon::FalconGripFourButton::FORWARD_BUTTON) ? 1 : 0;
        
        if(buttons[0]!=last_buttons[0]){
            // Clutching functionality
            if(buttons[0]==1){
                clutch = true;
                freeze = true;
                cout << "Clutch On" << endl;
            }

            else{
                clutch = false;
                reset_center = true;
                cout << "Clutch Off" << endl;
            }
        }

        if(buttons[1]!=last_buttons[1]){
            cout << "Mode!" << endl;
            replay_mode = true;
        }

        if(buttons[2]!=last_buttons[2]){
            if(buttons[2]==true){
                gripper_toggle.publish(to_string(1));
            }
        }

        if(buttons[3]!=last_buttons[3]){
            if(buttons[3]==true){
                if(recording==false){
                    // Short press is record - Long press is reset
                    timing_press = true;
                    start_timer = sc.now();
                }
                else{
                    outputfile.close();
                    cout << "Ending Recording" << endl;
                    recording=false;
                    // Let ROS know a file has been published to update the DMP
                    file_pub.publish("panda_demo_"+to_string(file_iter-1)+".csv");
                }
            }

            else{
                if(timing_press)
                {
                    timing_press = false;
                    // Get amount of time button was held
                    auto end_timer = sc.now();
                    auto time_span = static_cast<chrono::duration<double>>(end_timer - start_timer);
                    if(time_span.count()>0.4) // long hold is erase
                    {
                        cout << "Reset DMP!" << endl;
                        reset_pub.publish(to_string(1));
                    }
                    else{ // Short hold is record
                         string filename = {"panda_demo_"+to_string(file_iter)+".csv"};
                        remove( filename.c_str() );
                        outputfile.open (filename.c_str());
                        cout << "Starting Recording: " << filename.c_str() <<  endl;
                        file_iter++;
                        recording=true;
                    }
                }
            }
        }

        last_buttons[0] = buttons[0];
        last_buttons[1] = buttons[1];
        last_buttons[2] = buttons[2];
        last_buttons[3] = buttons[3];

        pollFalcon(pose_goal_pub,command_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze);
        falconVelocity();

        if (recording){
            log_demonstration(x,y,z,fx,fy,fz);
        }

        ros::spinOnce();
        usleep(1000);   
    }

    // Switch into replay mode
    // TODO: switch this to a loop and something that can go back and forth
    replay_active = true;
    bool quit_replay = false;
    while(ros::ok() && !quit_replay)
    {
        unsigned int my_buttons = m_falconDevice.getFalconGrip()->getDigitalInputs();
        bool buttons[4];
        // Replay is center button
        buttons[0] = (my_buttons & libnifalcon::FalconGripFourButton::CENTER_BUTTON)  ? 1 : 0;
        // Quit
        buttons[1] = (my_buttons & libnifalcon::FalconGripFourButton::PLUS_BUTTON)    ? 1 : 0;
        // TBD
        buttons[2] = (my_buttons & libnifalcon::FalconGripFourButton::MINUS_BUTTON)   ? 1 : 0;
        // TBD
        buttons[3] = (my_buttons & libnifalcon::FalconGripFourButton::FORWARD_BUTTON) ? 1 : 0;
        
        if(buttons[0]!=last_buttons[0]){
            // Run Replay
            if(buttons[0]==1){
                cout << "RUN THE REPLAY" << endl;
                replay_demo(selection_vector_pub,wrench_goal_pub, pose_goal_pub, pose_path_pub, dmp_replay_pub);
            }
        }

        if(buttons[1]!=last_buttons[1]){
            // Quit
            if(buttons[1]==1){
                quit_replay=true;
            }
        }

        last_buttons[0] = buttons[0];
        last_buttons[1] = buttons[1];
        last_buttons[2] = buttons[2];
        last_buttons[3] = buttons[3];
        falconVelocity();
        usleep(1000);
        ros::spinOnce();
    }
    
    m_falconDevice.close();
    return 0;
}
