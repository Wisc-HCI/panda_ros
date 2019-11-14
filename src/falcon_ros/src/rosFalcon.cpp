#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <relaxed_ik/EEPoseGoals.h>
#include "geometry_msgs/Pose.h"

#include "falcon/core/FalconDevice.h"
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

bool init_falcon() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
    //m_falconDevice.setFalconGrip<FalconGripFourButton>();
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
    //See if we have firmware
    bool firmware_loaded = false;
    //firmware_loaded = m_falconDevice.isFirmwareLoaded();
    
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

    m_falconDevice.runIOLoop();
    //falconFirmware = m_falconDevice.getFalconFirmware();
    //falconKinematic = new FalconKinematicStamper(true);
    return true;
}

void publishPose(ros::Publisher pose_goal_pub, std::array<double, 7> panda_pose) {
    relaxed_ik::EEPoseGoals ee_goal;
    geometry_msgs::Pose pose_goal;
    pose_goal.position.x = panda_pose[0];
    pose_goal.position.y = panda_pose[1];
    pose_goal.position.z = panda_pose[2];
    pose_goal.orientation.x = panda_pose[3];
    pose_goal.orientation.y = panda_pose[4];
    pose_goal.orientation.z = panda_pose[5];
    pose_goal.orientation.w = panda_pose[6];
    ee_goal.ee_poses.push_back(pose_goal);
    pose_goal_pub.publish(ee_goal);
}

void pollFalcon(ros::Publisher pose_goal_pub) {
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();
    // std::array<double, 3> panda_pos = {0.0, 0.0, 0.0};
    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //panda_pos[0] = -.12 + .5 * 0.26 * ((falconPos[2] - 0.074) / 0.099);
    panda_pos[0] = -.05 + .5 * 0.26 * ((falconPos[2] - 0.074) / 0.099);
    panda_pos[1] = -.16 + .5 * 0.47 * ((falconPos[0] + 0.047) / 0.067);
    panda_pos[2] = -.08 + .5 * 0.3 * ((falconPos[1] + 0.05) / 0.098);
    panda_pos[6] = 1;
    //panda_pos[0] = 0.26 + 0.26 * ((falconPos[2] - 0.074) / 0.099);
    //panda_pos[1] = -0.12 + 0.47 * ((falconPos[0] + 0.047) / 0.067);
    //panda_pos[2] = 0.29 + 0.3 * ((falconPos[1] + 0.05) / 0.098);
    //TODO: write the pose instead of the position
    // PandaController::writeCommandedPosition(panda_pos);
    //PandaController::writePoseGoal(panda_pos);
    publishPose(pose_goal_pub, panda_pos);
}

// void feedbackFalcon() {
//     franka::RobotState state = PandaController::readRobotState();
//     double scale = -0.1;
//     m_falconDevice.setForce({
//                 state.O_F_ext_hat_K[1] * scale, 
//                 state.O_F_ext_hat_K[2] * scale, 
//                 state.O_F_ext_hat_K[0] * scale});
// }

int main(int argc, char **argv) {
    if (!init_falcon()) {
        cout << "Failed to init falcon" << endl;
        return -1;
    }
    ros::init(argc, argv, "Falcon");
    ros::NodeHandle n;
        
    ros::Publisher pose_goal_pub = 
        n.advertise<relaxed_ik::EEPoseGoals>("/relaxed_ik/ee_pose_goals", 5);
    while(ros::ok()){
        pollFalcon(pose_goal_pub);
        usleep(1000);   
    }
    
    m_falconDevice.close();
    return 0;
}
