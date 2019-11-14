#include <ros/ros.h>
#include "PandaController.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <relaxed_ik/JointAngles.h>

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
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n;
    PandaController::ControlMode mode  = PandaController::ControlMode::JointVelocity;
    ros::Subscriber sub;
    PandaController::initPandaController(mode);
    
    /*
    std::array<double, 3> initialPosition;
    initialPosition[0] = 0.26;
    initialPosition[1] = 0.19;
    initialPosition[2] = 0.41;
    PandaController::writeCommandedPosition(initialPosition);
    */
    switch(mode){
        case PandaController::ControlMode::CartesianVelocity:
            sub = n.subscribe("PandaCommandedPose", 10, updateCallbackCart);
        case PandaController::ControlMode::JointVelocity:
            sub = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJoint);
    }
    ros::spin();

    return 0;
}
