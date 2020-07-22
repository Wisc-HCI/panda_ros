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
#include "panda_ros_msgs/VelocityBoundPath.h"
#include "panda_ros_msgs/HybridPose.h"

using namespace std;

namespace {
    PandaController::KinematicChain kinematicChain;
    PandaController::EELink eeLink;
}

void setKinematicChain(const std_msgs::String& msg) {
    if (msg.data == "pandaFlange") {
        kinematicChain = PandaController::KinematicChain::PandaFlange;
    }
}

void setEELink(const std_msgs::String& msg) {
    if (msg.data == "pandaGripper") {
        eeLink = PandaController::EELink::PandaGripper;
    }
}

void setCartPos(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        PandaController::setKinematicChain(kinematicChain, eeLink);
        PandaController::EulerAngles angles = 
            PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z
            ));
        vector<double> position{
            msg->position.x,
            msg->position.y,
            msg->position.z,
            angles.roll,
            angles.pitch,
            angles.yaw
        };
        PandaController::writeCommandedPosition(position);
    }
}

void setStampedTrajectory(vector<Eigen::VectorXd> path, vector<double> timestamps) {
    PandaController::setKinematicChain(kinematicChain, eeLink);
    PandaController::setTrajectory(PandaController::Trajectory(
        PandaController::TrajectoryType::Cartesian, 
        [path, timestamps]() {
            double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
            int goal_index = path.size();
            for(int i = 0; i < path.size(); i++) {
                if (timestamps[i] > now_ms) {
                    goal_index = i;
                    break;
                }
            }

            Eigen::VectorXd goal(6);
            Eigen::Quaterniond goal_q;

            if (goal_index == 0) {
                goal.topLeftCorner(3, 1) = path[0].topLeftCorner(3,1);
                goal_q = Eigen::Quaterniond(path[0].bottomRightCorner(4,1).data());
            } else if (goal_index == path.size()) {
                goal.topLeftCorner(3, 1) = path[goal_index-1].topLeftCorner(3,1);
                goal_q = Eigen::Quaterniond(path[goal_index-1].bottomRightCorner(4,1).data());
            } else {
                double t = (now_ms - timestamps[goal_index-1]) / (timestamps[goal_index] - timestamps[goal_index-1]);
                goal.topLeftCorner(3, 1) = path[goal_index-1].topLeftCorner(3,1) + t * (path[goal_index].topLeftCorner(3,1) - path[goal_index-1].topLeftCorner(3,1));
                goal_q = Eigen::Quaterniond(path[goal_index-1].bottomRightCorner(4,1).data()).slerp(t, Eigen::Quaterniond(path[goal_index].bottomRightCorner(4,1).data()));
            }
            auto goal_angles = PandaController::quaternionToEuler(goal_q.normalized());
            goal[3] = goal_angles.roll;
            goal[4] = goal_angles.pitch;
            goal[5] = goal_angles.yaw;
            return vector<double>(goal.data(), goal.data() + 6);
        }
    ));
}

void setStampedPath(const nav_msgs::Path::ConstPtr& msg) {
    if (PandaController::isRunning()){
        vector<Eigen::VectorXd> path;//{PandaController::getEEVector()};
        double now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        vector<double> timestamps{now};
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
            path.push_back(
                (Eigen::VectorXd(6) << 
                    poseStamped.pose.position.x,
                    poseStamped.pose.position.y,
                    poseStamped.pose.position.z,
                    angles.roll,
                    angles.pitch,
                    angles.yaw).finished()
            );
        }
        setStampedTrajectory(path, timestamps);
    }
}

void setVelocityBoundPath(const panda_ros_msgs::VelocityBoundPath::ConstPtr& msg) {
    if (PandaController::isRunning()) {
        Eigen::VectorXd current_position(7);
        current_position.topLeftCorner(3, 1) = PandaController::getEEPos();
        current_position.bottomRightCorner(4, 1) = PandaController::getEEOrientation().coeffs();
        double maxV = msg->maxV;

        double now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        vector<Eigen::VectorXd> path{current_position};
        vector<double> timestamps{now};
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto pose = msg->poses[i];
            Eigen::VectorXd command(7);
            command << 
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w;

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
        PandaController::setKinematicChain(kinematicChain, eeLink);
        auto twist = msg->twist;

        auto start_pos = PandaController::getEEPos();
        auto start_orientation = PandaController::getEEOrientation();
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

        double start_time = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        PandaController::setTrajectory(PandaController::Trajectory(
            PandaController::TrajectoryType::Cartesian, 
            [start_pos, start_orientation, start_time, velocity, end_time]() {
                double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
                double dt = (max(min(now_ms, end_time), start_time) - start_time) / 1000;
                Eigen::Vector3d target_pos = start_pos.topLeftCorner(3, 1) + dt * velocity.topLeftCorner(3, 1);

                PandaController::EulerAngles angles;
                angles.roll = dt * velocity[3]; angles.pitch = dt * velocity[4]; angles.yaw = dt * velocity[5];
                Eigen::Quaterniond rotation = PandaController::eulerToQuaternion(angles);
                auto target_angles = PandaController::quaternionToEuler((rotation * start_orientation).normalized());
                return vector<double>({
                    target_pos[0],
                    target_pos[1],
                    target_pos[2],
                    target_angles.roll,
                    target_angles.pitch,
                    target_angles.yaw
                });
            }
        ));
    }
}

void setHybrid(const panda_ros_msgs::HybridPose::ConstPtr& msg){
    if (PandaController::isRunning()){        
        PandaController::setKinematicChain(kinematicChain, eeLink);
        vector<double> command{
            //Position
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w,
            //Selection vector
            double(msg->sel_vector[0]),
            double(msg->sel_vector[1]),
            double(msg->sel_vector[2]),
            double(msg->sel_vector[3]),
            double(msg->sel_vector[4]),
            double(msg->sel_vector[5]),
            //Wrench
            msg->wrench.force.x,
            msg->wrench.force.y,
            msg->wrench.force.z,
            msg->wrench.torque.x,
            msg->wrench.torque.y,
            msg->wrench.torque.z,
            //Constraint frame
            msg->constraint_frame.x,
            msg->constraint_frame.y,
            msg->constraint_frame.z,
            msg->constraint_frame.w
        };

        PandaController::writeHybridCommand(command);
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
        PandaController::writeMaxForce(stod(command[1]));
        //PandaController::writeCommandedFT({0,0,-stod(command[1]),0,0,0});
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
    ros::Subscriber sub_vel_trajectory = n.subscribe("/panda/velocity_bound_path", 10, setVelocityBoundPath);
    ros::Subscriber sub_kinematicChain = n.subscribe("/panda/set_kinematic_chain", 10, setKinematicChain);
    ros::Subscriber sub_velocity = n.subscribe("/panda/cart_velocity", 10, setVelocity);
    ros::Subscriber sub_hybrid = n.subscribe("/panda/hybrid_pose", 10, setHybrid);

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
