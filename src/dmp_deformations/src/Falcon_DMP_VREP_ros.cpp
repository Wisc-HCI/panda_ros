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

// For calculated a filtered falcon velocity at 1000 Hz
array<double, 3> falcon_vel = {0.0, 0.0, 0.0};
std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
std::array<double, 3> last_falcon = {0.0, 0.0, 0.0};
std::array<double, 3> last_vel = {0.0, 0.0, 0.0};
std::array<double, 3> two_vel = {0.0, 0.0, 0.0};

bool last_falcon_updated = false;

// For recording
std::ofstream outputfile;
double x, y, z, fx, fy, fz;
double dmp_fx, dmp_fy, dmp_fz;

bool replay_active = false;

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
        //falcon_vel = {1.819*last_vel[0]-0.819*two_vel[0]+0.9048*falconPos[0]-0.9048*last_falcon[0],
        //1.819*last_vel[1]-0.819*two_vel[1]+0.9048*falconPos[1]-0.9048*last_falcon[1],
        //1.819*last_vel[2]-0.819*two_vel[2]+0.9048*falconPos[2]-0.9048*last_falcon[2]};
        //falcon_vel = {0.9048*last_vel[0]+10.0*falconPos[0]-10*last_falcon[0],0.9048*last_vel[1]+10.0*falconPos[1]-10*last_falcon[1],
        //0.9048*last_vel[2]+10.0*falconPos[2]-10*last_falcon[2]};
        falcon_vel = {(falconPos[0]-last_falcon[0])/delta_T,(falconPos[1]-last_falcon[1])/delta_T,(falconPos[2]-last_falcon[2])/delta_T};
    }

    //Update last falcon
    last_falcon[0] = falconPos[0];
    last_falcon[1] = falconPos[1];
    last_falcon[2] = falconPos[2];

    two_vel[0] = last_vel[0];
    two_vel[1] = last_vel[1];
    two_vel[2] = last_vel[2];
    last_vel[0] = falcon_vel[0];
    last_vel[1] = falcon_vel[1];
    last_vel[2] = falcon_vel[2];
    last_falcon_updated = true;
}

void replay_demo(ros::Publisher selection_vector_pub, ros::Publisher wrench_goal_pub, ros::Publisher pose_goal_pub, ros::Publisher pose_path_pub, ros::Publisher dmp_replay_pub){
    std::ifstream dmpfile("learneddmp.csv");
    double starting_x;
    double starting_y;
    double starting_f;

    double attractor_x;
    double attractor_y;
    double attractor_f;

    std_msgs::String replay_str;
    replay_str.data = "reset";
    dmp_replay_pub.publish(replay_str);
    
    double x, y, z, f;

    string temp;
    if(dmpfile.good())
    {
        double k=50; // eventually move these parameters to within the file
        double b=sqrt(2*k);

        double ddx, ddy, ddf;

        double dx = 0.0;
        double dy = 0.0;
        double df = 0.0; 

        double f_x, f_y, f_f;

        double fd_x, fd_y, fd_z;

        geometry_msgs::Pose pose;
        
    
        geometry_msgs::Wrench ft;
        geometry_msgs::Vector3 selection;

        // Read first mode command
        string throwaway;
        getline(dmpfile,throwaway,',');
        getline(dmpfile,throwaway,',');
        getline(dmpfile,temp);

        // Read Selection Vector
        getline(dmpfile,temp,',');
        selection.x = atof(temp.c_str());
        getline(dmpfile,temp,',');
        selection.y = atof(temp.c_str());
        getline(dmpfile,temp);
        selection.z = atof(temp.c_str());
        selection_vector_pub.publish(selection);

        // Read Starting Points
        getline(dmpfile,temp,',');
        starting_x = atof(temp.c_str());
        getline(dmpfile,temp,',');
        starting_y = atof(temp.c_str());
        getline(dmpfile,temp);
        starting_f = atof(temp.c_str());

        // Read Attractor Points
        getline(dmpfile,temp,',');
        attractor_x = atof(temp.c_str());
        getline(dmpfile,temp,',');
        attractor_y = atof(temp.c_str());
        getline(dmpfile,temp);
        attractor_f = atof(temp.c_str());

        // Reset
        x = starting_x;
        y = starting_y;
        f = starting_f;

        dx = 0.0;
        dy = 0.0;
        df = 0.0; 

        cout << "STARTING: " << starting_x << "," << starting_y << "," << starting_f << endl;

        // GO TO STARTING POINT AND WAIT A SECOND
        pose.position.x = starting_x;
        pose.position.y = starting_y;
        pose.position.z = starting_f;
        pose_path_pub.publish(pose);
        
        // Sleep for 2.2 seconds to allow path
        for(int jj=0; jj<2500; jj++)
        {
            ros::spinOnce();
            falconVelocity();
            usleep(1000);
        }


        cout << "REPLAYING MOTION" << endl;
        replay_str.data = "start";
        dmp_replay_pub.publish(replay_str);

        while(getline(dmpfile,temp,',')){
            ros::spinOnce();
            if(temp=="mode")
            {
                getline(dmpfile,temp,',');
                f_y = atof(temp.c_str());
                getline(dmpfile,temp);
                f_f = atof(temp.c_str());

                // Read Selection Vector
                getline(dmpfile,temp,',');
                selection.x = atof(temp.c_str());
                getline(dmpfile,temp,',');
                selection.y = atof(temp.c_str());
                getline(dmpfile,temp);
                selection.z = atof(temp.c_str());

                // Read Starting Points
                getline(dmpfile,temp,',');
                starting_x = atof(temp.c_str());
                getline(dmpfile,temp,',');
                starting_y = atof(temp.c_str());
                getline(dmpfile,temp);
                starting_f = atof(temp.c_str());

                // Read Attractor Points
                getline(dmpfile,temp,',');
                attractor_x = atof(temp.c_str());
                getline(dmpfile,temp,',');
                attractor_y = atof(temp.c_str());
                getline(dmpfile,temp);
                attractor_f = atof(temp.c_str());

                // Reset
                x = starting_x;
                y = starting_y;
                f = starting_f;

                dx = 0.0;
                dy = 0.0;
                df = 0.0; 

                selection_vector_pub.publish(selection);

                if(selection.x==0 || selection.y==0 || selection.z==0){
                    // Do force onloading with the first sample
                    cout << "FORCE ONLOADING STARTED" << endl;
                    ft.force.x = starting_x;
                    ft.force.y = starting_y;
                    ft.force.z = starting_f;
                    pose.position.x = starting_x;
                    pose.position.y = starting_y;
                    pose.position.z = starting_f;
                    pose_goal_pub.publish(pose);
                    wrench_goal_pub.publish(ft);

                    bool proper_contact = false;
                    while(!proper_contact)
                    {
                        // TODO - MAKE THIS MORE GENERAL!!!!
                        cout << "FZ: " << fz << " " << starting_f << endl;
                        if(fz<0.95*starting_f && fz>1.05*starting_f)
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
            }
            
            else{
                f_x = atof(temp.c_str());
                getline(dmpfile,temp,',');
                f_y = atof(temp.c_str());
                getline(dmpfile,temp);
                f_f = atof(temp.c_str());
                
                // Force mode gain, Position mode gain
                std::array<double, 2> sel_gains = {12000, 150};

                // Calculate New X
                ddx = k*(attractor_x-x)-b*dx+f_x+sel_gains[(int) selection.x]*dmp_fx;
                dx = dx + ddx*0.01;
                x = x+dx*0.01;

                // Calculate New Y
                ddy = k*(attractor_y-y)-b*dy+f_y+sel_gains[(int) selection.y]*dmp_fy;
                dy = dy + ddy*0.01;
                y = y+dy*0.01;

                // Calculate New F
                ddf = k*(attractor_f-f)-b*df+f_f+sel_gains[(int) selection.z]*dmp_fz;
                df = df + ddf*0.01;
                f = f+df*0.01;

                cout << " DEF X:" << sel_gains[(int) selection.x]*dmp_fx << " Y:" << sel_gains[(int) selection.y]*dmp_fy << " F:" << sel_gains[(int) selection.z]*dmp_fz << endl;

                ft.force.x = x;
                ft.force.y = y;
                ft.force.z = f;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = f;
                pose_goal_pub.publish(pose);
                wrench_goal_pub.publish(ft);
                for (int yy=0; yy<10; yy++)
                {
                    falconVelocity();
                    usleep(1000);
                }
            }
        }

        replay_str.data = "end";
        dmp_replay_pub.publish(replay_str);
    }
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
