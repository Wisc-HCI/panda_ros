#include <ros/ros.h>
#include "PandaController.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>

using namespace std;

void updateCallbackCartPos(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 3> position;
        position[0] = msg->position.x;
        position[1] = msg->position.y;
        position[2] = msg->position.z;
        
        PandaController::writeCommandedPosition(position);
    }
}

void updateCallbackJointPos(const relaxed_ik::JointAngles::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 7> position;
        position[0] = msg->angles.data[0];
        position[1] = msg->angles.data[1];
        position[2] = msg->angles.data[2];
        position[3] = msg->angles.data[3];
        position[4] = msg->angles.data[4];
        position[5] = msg->angles.data[5];
        position[6] = msg->angles.data[6];
        PandaController::writeJointAngles(position);
    }
}

void updateCallbackCartVel(const geometry_msgs::Twist::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 6> position;
        position[0] = msg->linear.x/2.5;
        position[1] = msg->linear.y/2.5;
        position[2] = msg->linear.z/2.5;
        position[3] = 0;
        position[4] = 0;
        position[5] = 0;
        PandaController::writeCommandedVelocity(position);
    }
}

void callbackCommands(const std_msgs::String& msg){
    if(msg.data == "button_pressed"){
        cout<<"Grasping"<<endl;
        PandaController::graspObject();
    }
    if(msg.data == "button_released"){
        cout<<"Releasing"<<endl;
        PandaController::releaseObject();
        }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n("~");
    std::string mode_str;
    n.getParam("control_mode", mode_str);

    PandaController::ControlMode mode;

    if(mode_str == "cartesian_velocity")
        mode = PandaController::ControlMode::CartesianVelocity;
    if(mode_str == "joint_velocity")
        mode = PandaController::ControlMode::JointVelocity;
    if(mode_str == "cartesian_position")
        mode = PandaController::ControlMode::CartesianPosition;
    if(mode_str == "joint_position")
        mode = PandaController::ControlMode::JointPosition;
    if(mode_str == "none")
        mode = PandaController::ControlMode::None;
        
    PandaController::initPandaController(mode);
    
    ros::Subscriber sub_commands = n.subscribe("/interaction/commands", 10, callbackCommands);
    ros::Subscriber sub_position;
    switch(mode){
        case PandaController::ControlMode::CartesianVelocity:
            sub_position = n.subscribe("/spacenav/twist", 10, updateCallbackCartVel);
            break;
        case PandaController::ControlMode::JointVelocity:
            //TODO: fix to call correct method
            // sub_position = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJointVel);
            sub_position = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJointPos);
            break;
        case PandaController::ControlMode::CartesianPosition:
            sub_position = n.subscribe("/panda/pose", 10, updateCallbackCartPos);
            break;
        case PandaController::ControlMode::JointPosition:
            sub_position = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJointPos);
            break;
        case PandaController::ControlMode::None:
            break;
            
    }

    franka::RobotState currentState;
    std::array<double, 6> forces;
    ros::Publisher wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    ros::Rate loopRate(1000);
    while (ros::ok() && PandaController::isRunning()) {
        currentState = PandaController::readRobotState();
        forces = currentState.O_F_ext_hat_K;

        geometry_msgs::Wrench wrench;
        wrench.force.x = forces[0];
        wrench.force.y = forces[1];
        wrench.force.z = forces[2];
        wrench.torque.x = forces[3];
        wrench.torque.y = forces[4];
        wrench.torque.z = forces[5];

        wrenchPub.publish(wrench);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
