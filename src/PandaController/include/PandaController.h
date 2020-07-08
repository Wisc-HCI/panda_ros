#ifndef PANDA_CONTROLLER_H
#define PANDA_CONTROLLER_H

#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <franka/gripper.h>


namespace PandaController {

    enum ControlMode {CartesianPosition, JointPosition, HybridControl, None};

    struct EulerAngles {
        double roll, pitch, yaw;
    };

    void stopControl();
    void initPandaController(ControlMode, bool = false);

    std::array<double, 6> readCommandedPosition();
    void writeCommandedPosition(std::array<double, 6> data);
    void writeCommandedPath(const std::array<double, 7>* data, const int & length);
    EulerAngles quaternionToEuler(Eigen::Quaterniond q);

    std::array<double, 6> readCommandedVelocity();
    void writeCommandedVelocity(std::array<double, 6> data);

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
    void consumeBuffer(int &, franka::RobotState*, long *);

    void startLogging();
    bool isRunning();

    void writeMaxForce(double val);
    double readMaxForce();
    
    franka::GripperState readGripperState();
    void toggleGrip(std::function<void ()> onToggle = NULL);
    void graspObject(std::function<void ()> onGrasp = NULL);
    void releaseObject(std::function<void ()> onRelease = NULL);

    void forceTorqueListener();

} //end namespace PandaController

#endif

