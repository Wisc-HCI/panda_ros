#include "PandaController.h"
#include "Kinematics.h"
#include "Trajectory.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <array>
#include <deque>
#include "DHA.h"

using namespace std;

namespace PandaController {
    Trajectory motionlessTrajectory();
    namespace {
        boost::mutex mutex;
        boost::mutex trajectory_mutex;
        Trajectory trajectory = motionlessTrajectory();
        franka::RobotState current_state;

        array<double, 7> pose_goal{}; //<x, y, z, x, y, z, w>

        array<double, 42> jacobian{};

        array<double, 3> selection_vector{};
        array<double, 6> FT_command{};
        
	    double maxForce = 15;

        vector<DHA> PandaGripperDHA{
            DHA(       0,   0.333,       0, -1),
            DHA(       0,       0, -M_PI/2,  0),
            DHA(       0, 0.31599,  M_PI/2,  1),
            DHA( 0.08249,       0,  M_PI/2,  2),
            DHA(-0.08249,   0.384, -M_PI/2,  3),
            DHA(       0,       0,  M_PI/2,  4),
            DHA(  0.0879,       0,  M_PI/2,  5),
            DHA(       0,  0.1069,       0, -1)
        };
        vector<DHA> ee_chain = PandaGripperDHA;
    }

    void setKinematicChain(KinematicChain chain) {
        switch (chain){
            case KinematicChain::PandaGripper:
                ee_chain = PandaGripperDHA;
                break;
            default:
                ee_chain = PandaGripperDHA;
                break;
        }
    }

    void setTrajectory(Trajectory t) {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        trajectory = t;
    }

    vector<double> getNextCommand(TrajectoryType & t) {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        t = trajectory.type;
        return trajectory();
    }

    Trajectory motionlessTrajectory() {
        return Trajectory(
            TrajectoryType::Joint,
            []() {
                auto q = readRobotState().q;
                return vector<double>({q[0], q[1], q[2], q[3], q[4], q[5], q[6]});
            }
        );
    }

    void dontMove() {
        boost::lock_guard<boost::mutex> guard(trajectory_mutex);
        auto t = motionlessTrajectory();
        trajectory = t;
    }

    bool isMotionDone(array<double, 7> command) {
        long timePoint = (long)command[0];
        long deltaT = chrono::duration_cast<chrono::milliseconds>(
                chrono::time_point<chrono::system_clock>(chrono::milliseconds(timePoint)) - chrono::system_clock::now()
        ).count();
        // If the waypoint is in the future. 
        // Then we aren't done yet
        if (deltaT > 0) {
            return false;
        }
        // If it is sufficiently far in the past
        // Then lets forget about it.
        return true;
    }

    void updateInterpolationCoefficients() {
        // double now_ms = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
        // // Fit to poly of the form w0 + w1 t + w2 t^2 + ...
        // // This has 3 parameters from the position, velocity and acceleration at the beginning.
        // // + 1 parameter for each positional waypoint in the middle.
        // // + 3 more parameters for the end goal position, end velocity (0) and and acceleration (0);
        // // Only interpolate between the next few points.
        // int m = 6;
        // Eigen::MatrixXd A(m,m); A.fill(0);
        // Eigen::MatrixXd B(m, 6); B.fill(0);
        // // t goes from 0 (now) to 1 (last point)
        // array<double, 7> command = commanded_position.front();
        // double timeWidth = command[0] - now_ms;
        // // First 3 rows look like:
        // // 1 0 0 0 ...
        // // 0 1 0 0 ...
        // // 0 0 2 0 ...
        // A(0,0) = 1;
        // A(1,1) = 1;
        // A(2,2) = 2;

        // Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
        // Eigen::Vector3d positionVector(transformMatrix.translation());
        // Eigen::Quaterniond orientationVector(transformMatrix.linear());
        // EulerAngles euler = quaternionToEuler(orientationVector);
        // for (size_t i = 0; i < 3; i++) {
        //     B(0,i) = positionVector[i];
        // }
        // B(0,3) = euler.roll - M_PI;
        // if (B(0,3) < -M_PI) B(0,3) += 2* M_PI;
        // else if (B(0,3) > M_PI) B(0,3) -= 2 * M_PI;
        // B(0,4) = euler.pitch;
        // B(0,5) = euler.yaw;
        // //B.row(1) = Eigen::Map<const Eigen::VectorXd>(currentVelocity.data(), 6);
        // //B.row(2) = Eigen::Map<const Eigen::VectorXd>(currentAcceleration.data(), 6); // currentAcceleration is currently not set.

        // A.row(3) << 1, 1, 1, 1, 1, 1;
        // B.row(3) << command[1], command[2], command[3], command[4], command[5], command[6];
        // //Huh?
        // A.row(4) << 0, 1, 2, 3, 4, 5;
        // A.row(4) << 0, 0, 6, 12, 20, 30;
        // Eigen::MatrixXd W = A.completeOrthogonalDecomposition().solve(B);
        // for (size_t i = 0; i < (m * 6); i++) {
        //     interpolationCoefficients[i] = W.data()[i];
        // }
        // pathStartTime_ms = (long)now_ms;
        // pathEndTime_ms = (long)command[0];
    }

    array<double, 6> readCommandedFT(){
        boost::lock_guard<boost::mutex> guard(mutex);

        return FT_command;
    }

    void writeCommandedFT(array<double, 6> data){
        boost::lock_guard<boost::mutex> guard(mutex);

        FT_command = data;
    }

    void writeCommandedPosition(vector<double> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        trajectory = Trajectory(
            TrajectoryType::Cartesian,
            [data]() {
                return data;
            }
        );
    }

    array<double, 3> readSelectionVector(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return selection_vector;
    }
    void writeSelectionVector(array<double, 3> data){

        boost::lock_guard<boost::mutex> guard(mutex);
        selection_vector = data;
    }
    void writeJointAngles(vector<double> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        trajectory = Trajectory(
            TrajectoryType::Joint,
            [data]() {
                return data;
            }
        );
    }
    
    Eigen::Matrix4d getEETransform() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return EEFromDHA(current_state.q, ee_chain);
    }

    Eigen::VectorXd getEEVector() {
        Eigen::Affine3d transform(getEETransform());
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        EulerAngles current_a = quaternionToEuler(orientation);
        Eigen::VectorXd ee(6);
        ee << position[0], position[1], position[2], current_a.roll, current_a.pitch, current_a.yaw;
        return ee;
    }

    franka::RobotState readRobotState(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return current_state;
    }
    void writeRobotState(franka::RobotState data){
        boost::lock_guard<boost::mutex> guard(mutex);
        jacobian = jacobianFromDHA(data.q, ee_chain);//pandaJacobian(data.q);
        current_state = data;
    }
    array<double, 42> readJacobian() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return jacobian;
    }
    double readMaxForce() {
        boost::lock_guard<boost::mutex> guard(mutex);
        return maxForce;
    }
    void writeMaxForce(double val) {
        boost::lock_guard<boost::mutex> guard(mutex);
        maxForce = val;
    }
}