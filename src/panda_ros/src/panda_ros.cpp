#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <signal.h>
#include "PandaController.h"
#include "Trajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <deque>
#include <boost/algorithm/string.hpp>
#include "panda_ros/VelocityBoundPath.h"

using namespace std;

namespace {
    PandaController::KinematicChain kinematicChain;
}

void setKinematicChain(const std_msgs::String& msg) {
    if (msg.data == "pandaGripper") {
        kinematicChain = PandaController::KinematicChain::PandaGripper;
    }
}

void setCartPos(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        PandaController::setKinematicChain(kinematicChain);
        PandaController::EulerAngles angles = 
            PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z
            ));
        vector<double> position{
            position[0] = msg->position.x,
            position[1] = msg->position.y,
            position[2] = msg->position.z,
            position[3] = angles.roll,
            position[4] = angles.pitch,
            position[5] = angles.yaw
        };
        PandaController::writeCommandedPosition(position);
    }
}

void setStampedTrajectory(vector<vector<double>> path, vector<double> timestamps) {
    PandaController::setKinematicChain(kinematicChain);
    PandaController::setTrajectory(PandaController::Trajectory(
        PandaController::TrajectoryType::Cartesian, 
        [path, timestamps]() {
            double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
            int goal_index = path.size() - 1;
            for(int i = 0; i < path.size(); i++) {
                if (timestamps[i] > now_ms) {
                    goal_index = i;
                    break;
                }
            }
            
            return path[goal_index];
        }
    ));
}

void setStampedPath(const nav_msgs::Path::ConstPtr& msg) {
    if (PandaController::isRunning()){
        PandaController::setKinematicChain(kinematicChain);
        vector<vector<double>> path = vector<vector<double>>();
        vector<double> timestamps = vector<double>();
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto poseStamped = msg->poses[i];
            long secs = poseStamped.header.stamp.sec;
            long nsecs = poseStamped.header.stamp.nsec;
            PandaController::EulerAngles angles = 
                PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                    poseStamped.pose.orientation.w,
                    poseStamped.pose.orientation.x,
                    poseStamped.pose.orientation.y,
                    poseStamped.pose.orientation.z
                ));

            timestamps.push_back(secs * 1000 + nsecs / 1000000);
            vector<double> command{
                poseStamped.pose.position.x,
                poseStamped.pose.position.y,
                poseStamped.pose.position.z,
                angles.roll,
                angles.pitch,
                angles.yaw
            };
            path.push_back(command);
        }
        setStampedTrajectory(path, timestamps);
    }
}

void setVelocityBoundPath(const panda_ros::VelocityBoundPath::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        PandaController::setKinematicChain(kinematicChain);
        auto position = PandaController::getEEVector();

        vector<double> current_position{
            position[0],
            position[1],
            position[2],
            position[3],
            position[4],
            position[5]
        };

        double maxV = msg->maxV;

        vector<vector<double>> path = vector<vector<double>>();
        vector<double> timestamps = vector<double>();
        double now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto pose = msg->poses[i];
            PandaController::EulerAngles angles = 
                PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z
                ));
            vector<double> command{
                pose.position.x,
                pose.position.y,
                pose.position.z,
                angles.roll,
                angles.pitch,
                angles.yaw
            };

            double distance = sqrt(
                (command[0] - current_position[0]) * (command[0] - current_position[0]) + 
                (command[1] - current_position[1]) * (command[1] - current_position[1]) + 
                (command[2] - current_position[2]) * (command[2] - current_position[2])
            );
            double timestamp = now + distance / (maxV / 1000);
            timestamps.push_back(timestamp);
            path.push_back(command);
            current_position = command;
            now = timestamp;
        }

        setStampedTrajectory(path, timestamps);
    }
}

void setVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        PandaController::setKinematicChain(kinematicChain);
        auto twist = msg->twist;
        Eigen::VectorXd velocity(6);
        velocity << 
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z;
        long secs = msg->header.stamp.sec;
        long nsecs = msg->header.stamp.nsec;
        double end_time = secs * 1000 + nsecs / 1000000;

        auto start_pos = PandaController::getEEVector();
        double start_time = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        PandaController::setTrajectory(PandaController::Trajectory(
            PandaController::TrajectoryType::Cartesian, 
            [start_pos, start_time, velocity, end_time]() {
                double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
                double dt = now_ms - start_time;
                auto target_pos = start_pos + (now_ms < end_time)*(dt / 1000)*velocity;
                return vector<double>({
                    target_pos[0],
                    target_pos[1],
                    target_pos[2],
                    target_pos[3],
                    target_pos[4],
                    target_pos[5]
                });
            }
        ));
    }
}

void setSelectionVector(const geometry_msgs::Vector3::ConstPtr& msg){
    if (PandaController::isRunning()){
        PandaController::setKinematicChain(kinematicChain);
        std::array<double, 3> vec;
        vec[0] = msg->x;
        vec[1] = msg->y;
        vec[2] = msg->z;
        PandaController::writeSelectionVector(vec);
    }
}

void callbackCommands(const std_msgs::String& msg){
    std::vector<std::string> command;
    boost::split(command, msg.data, [](char c){return c == ';';});
    if(msg.data == "grasp"){
        cout<<"Grasping"<<endl;
        PandaController::graspObject();
    }
    if(msg.data == "release"){
        PandaController::releaseObject();
    }
    if(msg.data == "toggleGrip") {
        PandaController::toggleGrip();
    }
    if(command[0] == "setMaxForce") {
        cout<<"Setting max force to "<<command[1]<<endl;
        PandaController::writeCommandedFT({0,0,-stod(command[1]),0,0,0});
    }
    
}

void publishJointState(franka::RobotState robot_state, ros::Publisher jointPub){
    const vector<string> joint_names{"panda_joint1", "panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"};
    franka::GripperState gripperState = PandaController::readGripperState();

    sensor_msgs::JointState states;
    states.effort.resize(joint_names.size());
    states.name.resize(joint_names.size());
    states.position.resize(joint_names.size());
    states.velocity.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++) {
        states.name[i] = joint_names[i];
    }
    states.header.stamp = ros::Time::now();
    for (size_t i = 0; i < joint_names.size()-2; i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
    }
    states.position[joint_names.size()-2] = gripperState.width/2.;
    states.position[joint_names.size()-1] = gripperState.width/2.;
    
    jointPub.publish(states);
}

void publishTf(franka::RobotState robot_state){
    static tf2_ros::TransformBroadcaster br;
    
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    // Align the orientation of the end-effector with panda_link0
    Eigen::Quaterniond rot(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));
    orientation *= rot;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = "end_effector";
    transformStamped.transform.translation.x = position[0];
    transformStamped.transform.translation.y = position[1];
    transformStamped.transform.translation.z = position[2];
    
    transformStamped.transform.rotation.x = orientation.coeffs()[0];
    transformStamped.transform.rotation.y = orientation.coeffs()[1];
    transformStamped.transform.rotation.z = orientation.coeffs()[2];
    transformStamped.transform.rotation.w = orientation.coeffs()[3];

    br.sendTransform(transformStamped);
}

void publishWrench(franka::RobotState robot_state, ros::Publisher wrenchPub){
    std::array<double, 6> forces;
    //forces = robot_state.O_F_ext_hat_K;
    forces = PandaController::readFTForces();

    geometry_msgs::Wrench wrench;
    wrench.force.x = forces[0];
    wrench.force.y = forces[1];
    wrench.force.z = forces[2];
    wrench.torque.x = forces[3];
    wrench.torque.y = forces[4];
    wrench.torque.z = forces[5];

    wrenchPub.publish(wrench);
}

void publishState(ros::Publisher wrenchPub, ros::Publisher jointPub){
    franka::RobotState robot_state = PandaController::readRobotState();
    publishJointState(robot_state, jointPub);
    publishTf(robot_state);
    publishWrench(robot_state, wrenchPub);
}

void signalHandler(int sig)
{
    PandaController::stopControl();
    ros::shutdown();
    exit(sig);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n("~");
    
    //Setup the signal handler for exiting, must be called after ros is intialized
    signal(SIGINT, signalHandler); 

    PandaController::initPandaController();
    
    ros::Subscriber sub_commands = n.subscribe("/panda/commands", 10, callbackCommands);
    ros::Subscriber sub_position = n.subscribe("/panda/cart_pose", 10, setCartPos);
    ros::Subscriber sub_trajectory = n.subscribe("/panda/path", 10, setStampedPath);
    ros::Subscriber sub_vel_trajectory = n.subscribe("/panda/velocity_bound_path", 10, setStampedPath);
    ros::Subscriber sub_selectionVector = n.subscribe("/panda/selection_vector", 10, setSelectionVector);
    ros::Subscriber sub_kinematicChain = n.subscribe("/panda/set_kinematic_chain", 10, setKinematicChain);
    ros::Subscriber sub_velocity = n.subscribe("/panda/cart_velocity", 10, setVelocity);

    ros::Publisher wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    ros::Publisher jointPub = n.advertise<sensor_msgs::JointState>("/panda/joint_states", 1);
    ros::Rate loopRate(1000);
    while (ros::ok() && PandaController::isRunning()) {
        publishState(wrenchPub,jointPub);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
