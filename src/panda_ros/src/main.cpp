#include <ros/ros.h>
#include "PandaController.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>

using namespace std;

void updateCallbackCart(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 3> position;
        position[0] = msg->position.x;
        position[1] = msg->position.y;
        position[2] = msg->position.z;
        
        PandaController::writeCommandedPosition(position);
    }
}

void updateCallbackJoint(const relaxed_ik::JointAngles::ConstPtr& msg){
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
        cout << "Updating joint angles" << endl;
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
    ros::NodeHandle n;
    // PandaController::ControlMode mode  = PandaController::ControlMode::JointVelocity;
    PandaController::ControlMode mode  = PandaController::ControlMode::None;
    //PandaController::ControlMode mode  = PandaController::ControlMode::CartesianVelocity;
    PandaController::initPandaController(mode);
    
    n.subscribe("/interaction/commands", 10, callbackCommands);
    switch(mode){
        case PandaController::ControlMode::CartesianVelocity:
            n.subscribe("/panda/pose", 10, updateCallbackCart);
            break;
        case PandaController::ControlMode::JointVelocity:
            n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJoint);
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
        ros::spinOnce();
        cout << "looping" << endl;
        loopRate.sleep();
    }

    return 0;
}
