#include <ros/ros.h>
#include <signal.h>
#include "PandaController.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>
#include <csignal>

using namespace std;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in panda_ros.cpp\n";
    PandaController::stopControl();
    ros::NodeHandle n("~");
    ros::Publisher wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    geometry_msgs::Wrench wrench;
    wrench.force.x = 0;
    wrench.force.y = 0;
    wrench.force.z = 0;
    wrench.torque.x = 0;
    wrench.torque.y = 0;
    wrench.torque.z = 0;

    wrenchPub.publish(wrench);   
    ros::shutdown();

    std::array<double, 6> data = {0.0,0.0,0.0,0.0,0.0,0.0};
    PandaController::writeCommandedVelocity(data);
    exit(sig);
}

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

void updateCallbackJointVel(const relaxed_ik::JointAngles::ConstPtr& msg){
    //TODO
    return;
}

void updateCallbackCartVel(const geometry_msgs::Twist::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 6> velocity;
        velocity[0] = msg->linear.x;
        velocity[1] = msg->linear.y;
        velocity[2] = msg->linear.z;
        velocity[3] = msg->angular.x;
        velocity[4] = msg->angular.y;
        velocity[5] = msg->angular.z;
        PandaController::writeCommandedVelocity(velocity);
    }
}

void callbackCommands(const std_msgs::String& msg){

    if(msg.data == "grasp"){
        cout<<"Grasping"<<endl;
        PandaController::graspObject();
    }
    if(msg.data == "release"){
        PandaController::releaseObject();
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n("~");
    std::string mode_str;
    //Always specify parameter, it uses cached parameter instead of default value.
    n.param<std::string>("control_mode", mode_str,"none");
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
        
    //Setup the signal handler for exiting, must be called after ros is intialized
    signal(SIGINT, signalHandler); 

    PandaController::initPandaController(mode);
    
    ros::Subscriber sub_commands = n.subscribe("/panda/commands", 10, callbackCommands);
    ros::Subscriber sub_position;
    switch(mode){
        case PandaController::ControlMode::CartesianVelocity:
            sub_position = n.subscribe("/panda/cart_vel", 10, updateCallbackCartVel);
            break;
        case PandaController::ControlMode::JointVelocity:
            //TODO: we don't actually have anything that uses this, not set up correctly in PandaController
            sub_position = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJointVel);
            break;
        case PandaController::ControlMode::CartesianPosition:
            sub_position = n.subscribe("/panda/cart_pose", 10, updateCallbackCartPos);
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
