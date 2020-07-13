#ifndef PANDA_CONTROLLER_H
#define PANDA_CONTROLLER_H

#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <franka/gripper.h>
#include "Trajectory.h"


namespace PandaController {
    enum KinematicChain {PandaGripper};
    struct EulerAngles {
        double roll, pitch, yaw;
    };

    void stopControl();
    void initPandaController(bool = false);

    std::vector<double> getNextCommand(TrajectoryType & t);
    Eigen::Matrix4d getEETransform();
    Eigen::VectorXd getEEVector();

    void setTrajectory(Trajectory);
    void writeCommandedPosition(std::vector<double> data);
    EulerAngles quaternionToEuler(Eigen::Quaterniond q);

    std::array<double, 6> readFTForces();
    void writeFTForces(std::array<double, 6> data);

    std::array<double, 7> readPoseGoal();
    void writePoseGoal(std::array<double, 7> data);

    std::array<double, 7> readJointAngles();
    void writeJointAngles(std::array<double, 7> data);

    std::array<double, 3> readSelectionVector();
    void writeSelectionVector(std::array<double, 3> data);

    std::array<double, 6> readCommandedFT();
    void writeCommandedFT(std::array<double, 6> data);

    franka::RobotState readRobotState();
    void writeRobotState(franka::RobotState data);
    std::array<double, 42> readJacobian();
    void printRobotJointAngles(franka::RobotState data);

    void startLogging();
    bool isRunning();

    void writeMaxForce(double val);
    double readMaxForce();
    
    franka::GripperState readGripperState();
    void toggleGrip(std::function<void ()> onToggle = NULL);
    void graspObject(std::function<void ()> onGrasp = NULL);
    void releaseObject(std::function<void ()> onRelease = NULL);

    void forceTorqueListener();
    void setKinematicChain(KinematicChain chain);

} //end namespace PandaController

#endif

