#include "PandaController.h"
#include "Kinematics.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <array>
#include <deque>

using namespace std;

namespace PandaController {
    
    namespace {
        boost::mutex mutex;
        
        deque<array<double, 7>> commanded_position = deque<array<double, 7>>();
        long pathStartTime_ms = 0;
        long pathEndTime_ms = 0;
        array<double, 36> interpolationCoefficients{};
        franka::RobotState current_state;

        array<double, 7> pose_goal{}; //<x, y, z, x, y, z, w>
        array<double, 7> joint_angles{0.0,-0.4,0.0,-2.0,0.0,1.6,0.8};

        array<double, 42> jacobian{};

        array<double, 3> selection_vector{};
        array<double, 6> FT_command{};
        
	    double maxForce = 15;
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
        double now_ms = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
        // Fit to poly of the form w0 + w1 t + w2 t^2 + ...
        // This has 3 parameters from the position, velocity and acceleration at the beginning.
        // + 1 parameter for each positional waypoint in the middle.
        // + 3 more parameters for the end goal position, end velocity (0) and and acceleration (0);
        // Only interpolate between the next few points.
        int m = 6;
        Eigen::MatrixXd A(m,m); A.fill(0);
        Eigen::MatrixXd B(m, 6); B.fill(0);
        // t goes from 0 (now) to 1 (last point)
        array<double, 7> command = commanded_position.front();
        double timeWidth = command[0] - now_ms;
        // First 3 rows look like:
        // 1 0 0 0 ...
        // 0 1 0 0 ...
        // 0 0 2 0 ...
        A(0,0) = 1;
        A(1,1) = 1;
        A(2,2) = 2;

        Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
        Eigen::Vector3d positionVector(transformMatrix.translation());
        Eigen::Quaterniond orientationVector(transformMatrix.linear());
        EulerAngles euler = quaternionToEuler(orientationVector);
        for (size_t i = 0; i < 3; i++) {
            B(0,i) = positionVector[i];
        }
        B(0,3) = euler.roll - M_PI;
        if (B(0,3) < -M_PI) B(0,3) += 2* M_PI;
        else if (B(0,3) > M_PI) B(0,3) -= 2 * M_PI;
        B(0,4) = euler.pitch;
        B(0,5) = euler.yaw;
        //B.row(1) = Eigen::Map<const Eigen::VectorXd>(currentVelocity.data(), 6);
        //B.row(2) = Eigen::Map<const Eigen::VectorXd>(currentAcceleration.data(), 6); // currentAcceleration is currently not set.

        A.row(3) << 1, 1, 1, 1, 1, 1;
        B.row(3) << command[1], command[2], command[3], command[4], command[5], command[6];
        //Huh?
        A.row(4) << 0, 1, 2, 3, 4, 5;
        A.row(4) << 0, 0, 6, 12, 20, 30;
        Eigen::MatrixXd W = A.completeOrthogonalDecomposition().solve(B);
        for (size_t i = 0; i < (m * 6); i++) {
            interpolationCoefficients[i] = W.data()[i];
        }
        pathStartTime_ms = (long)now_ms;
        pathEndTime_ms = (long)command[0];
    }

    array<double, 6> readCommandedPosition(){
        boost::lock_guard<boost::mutex> guard(mutex);

        auto command = commanded_position.front();
        bool pathChanged = false;
        while (commanded_position.size() > 1 && isMotionDone(command)) {
            commanded_position.pop_front();
            command = commanded_position.front();
            pathChanged = true;
        }
        if (pathChanged) updateInterpolationCoefficients();

        double now_ms = chrono::system_clock::now().time_since_epoch() / chrono::milliseconds(1);
        double deltaT = command[0] - now_ms;
        // If we are told to go there *now*, then forget interpolation.
        if (deltaT < 0){
            array<double, 6> positionCommand;
            for (size_t i = 0; i < 6; i++) {
                positionCommand[i] = command[i+1];
            }
            return positionCommand;
        }

        // If we have a little time, interpolate between all of the remaining points
        // to find a reasonable trajectory.
        // Now compute the position for 1ms in the future.

        double timeWidth = pathEndTime_ms - pathStartTime_ms;
        double t = (now_ms - pathStartTime_ms) / timeWidth;
        Eigen::MatrixXd W = Eigen::Map<Eigen::MatrixXd>(interpolationCoefficients.data(), 6, 6);
        double coeff = 1;
        array<double, 6> positionCommand{};
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 6; j++) {
                positionCommand[j] += coeff * W(i, j);
            }
            coeff *= t;
        }

        return positionCommand;
    }

    array<double, 6> readCommandedFT(){
        boost::lock_guard<boost::mutex> guard(mutex);

        return FT_command;
    }

    void writeCommandedFT(array<double, 6> data){
        boost::lock_guard<boost::mutex> guard(mutex);

        FT_command = data;
    }

    void writeCommandedPosition(array<double, 6> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        array<double, 7> timeStamped;
        timeStamped[0] = -1;
        for (size_t i = 0; i < 6; i++) {
            timeStamped[i+1] = data[i];
        }

        commanded_position.clear();
        commanded_position.push_back(timeStamped);
    }

    void writeCommandedPath(const array<double, 7>* data, const int & length) {
        boost::lock_guard<boost::mutex> guard(mutex);
        commanded_position.clear();
        for (int i = 0; i < length; i++ ) {
            commanded_position.push_back(data[i]);
        }
        updateInterpolationCoefficients();
    }

    array<double, 7> readPoseGoal(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return pose_goal;
    }
    array<double, 3> readSelectionVector(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return selection_vector;
    }
    void writeSelectionVector(array<double, 3> data){

        boost::lock_guard<boost::mutex> guard(mutex);
        selection_vector = data;
    }
    void writePoseGoals(array<double, 7> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        pose_goal = data;
    }
    array<double, 7> readJointAngles(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return joint_angles;
    }
    void writeJointAngles(array<double, 7> data){
        boost::lock_guard<boost::mutex> guard(mutex);
        joint_angles = data;
    }
    
    franka::RobotState readRobotState(){
        boost::lock_guard<boost::mutex> guard(mutex);
        return current_state;
    }
    void writeRobotState(franka::RobotState data){
        boost::lock_guard<boost::mutex> guard(mutex);
        jacobian = calculatePandaJacobian(data.q);
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