#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <franka/log.h>
#include "PandaController.h"
#include "Common.h"
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <thread>
#include <cmath>
#include <future>
#include <functional>
#include <deque>

using namespace boost::interprocess;
using namespace std;

namespace PandaController {
    namespace {
        shared_data* SharedData = NULL;
        mapped_region* memoryRegion;
        franka::Gripper *p_gripper;
        double maxGripperWidth;
    } 

    //Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    Eigen::Quaterniond eulerToQuaternion(EulerAngles angle) // roll (X), pitch (Y), yaw (Z)
    {
        // Abbreviations for the various angular functions
        double cy = cos(angle.yaw * 0.5);
        double sy = sin(angle.yaw * 0.5);
        double cp = cos(angle.pitch * 0.5);
        double sp = sin(angle.pitch * 0.5);
        double cr = cos(angle.roll * 0.5);
        double sr = sin(angle.roll * 0.5);

        Eigen::Quaterniond q(cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr, sy * cp * sr + cy * sp * cr, sy * cp * cr - cy * sp * sr);

        return q;
    }
    
    //Adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    EulerAngles quaternionToEuler(Eigen::Quaterniond q){
        EulerAngles angle;
        double q0=q.coeffs()[3];
        double q1=q.coeffs()[0];
        double q2=q.coeffs()[1];
        double q3=q.coeffs()[2];

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
        double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        angle.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q0 * q2 - q3 * q1);
        
        if (std::abs(sinp) >= 1)
            angle.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angle.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q0 * q3 + q1 * q2);
        double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        angle.yaw = std::atan2(siny_cosp, cosy_cosp);
        
        return angle;
    }

    // See: https://github.com/frankaemika/franka_ros/issues/35
    // This adds a tiny bit of random noise to each component to ensure they aren't 0.
    // Only works with column vectors, which seems to be the normal kind.
    void addNoise(Eigen::VectorXd & v) {
        double epsilon = 0.00001;
        for(int i = 0; i < v.rows(); i++) {
            v[i] += rand() % 2 == 0 ? epsilon : - epsilon;
        }
    }

    void printJointValues(const vector<franka::Record> log, int derivative) {
        vector<Eigen::VectorXd> values;
        for (int i = 0; i < log.size(); i++) {
            franka::RobotState state = log.at(i).state;
            values.push_back(Eigen::Map<Eigen::VectorXd>(state.q_d.data(), 6));
            cout << "q_d " << values.at(i).transpose() << endl;
        }
        for (int d = 0; d < derivative; d++) {
            for(int i = 0; i < values.size() - 1; i++){
                values.at(i) =  (values.at(i + 1) - values.at(i)) * 1000;
            }
            values.pop_back();
        }
        for (int i = 0; i < values.size(); i++) {
            cout << values.at(i).transpose() << endl;
        }
    }

    void printForces(const vector<franka::Record> log) {
        for (int i = 0; i < log.size(); i++) {
            franka::RobotState state = log.at(i).state;
            cout << Eigen::Map<Eigen::VectorXd>(state.O_F_ext_hat_K.data(), 6).transpose() << endl;
        }
    }

    void printControlError(const franka::ControlException& e) {
        string message = e.what();
        cout << message << endl;
        if (message.find("joint_velocity_discontinuity") != string::npos) {
            cout << "Printing joint accelerations" << endl;
            printJointValues(e.log, 2);
        } else if (message.find("joint_acceleration_discontinuity") != string::npos) {
            cout << "Printing joint jerks" << endl;
            printJointValues(e.log, 3);
        } else if (message.find("cartesian_reflex") != string::npos) {
            printForces(e.log);
        } else {
            cout << "No case specified for this error" << endl;
        }
    }


    array<double,42> calculatePandaJacobian(array<double, 7> q) {
        // Calculated using Modified DH-Parameters analytically
        // using joint anlges, q:
        array<double, 42> jacobian_array;

        // Store the results in a column-wise matrix that is put in the
        // shared memory object

        // Column 0
        //(0,0)
        jacobian_array[0]=0.384*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.31599*std::sin(q[0])*
        std::sin(q[1]) - 0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.08249*std::cos(q[0])*
        std::sin(q[2]) + 0.08249*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*
        std::cos(q[2])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) + 0.08249*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) - 0.0879*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) +
         0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*std::cos(q[0])*
         std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
         std::sin(q[5]) - 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) +
         0.0879*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
         std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*
         std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*
         std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) -
         0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
         std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[4])*
         std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(1,0)
        jacobian_array[1]=0.31599*std::cos(q[0])*std::sin(q[1]) - 0.08249*std::sin(q[0])*std::sin(q[2]) - 0.08249*
        std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) + 0.384*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2]) + 0.384*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) -
        0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.0879*std::cos(q[0])*
        std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) +
        0.1069*std::cos(q[0])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[5]);

        //(2,0)
        jacobian_array[2]=0.0;

        //(3,0)
        jacobian_array[3]=0.0;

        //(4,0)
        jacobian_array[4]=0.0;

        //(5,0)
        jacobian_array[5]=1.0;

        // Column 1
        //(0,1)
        jacobian_array[6]=0.31599*std::cos(q[0])*std::cos(q[1]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[3]) -
        0.08249*std::cos(q[0])*std::cos(q[2])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::sin(q[3]) -
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.08249*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.384*
        std::cos(q[0])*std::cos(q[2])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) +
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*
        std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) - 0.1069*
        std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);

        //(1,1)
        jacobian_array[7]=0.31599*std::cos(q[1])*std::sin(q[0]) - 0.08249*std::cos(q[2])*std::sin(q[0])*std::sin(q[1]) -
        0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[3]) + 0.384*std::cos(q[1])*std::cos(q[3])*std::sin(q[0]) -
        0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) + 0.08249*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[0])*std::sin(q[1]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.384*
        std::cos(q[2])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) +
        0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
        std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]);

        //(2,1)
        jacobian_array[8]=0.08249*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*std::cos(q[2]) - 0.384*std::cos(q[3])*
        std::sin(q[1]) - 0.31599*std::sin(q[1]) - 0.0879*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.08249*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[5])*
        std::sin(q[1]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[2])*
        std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(3,1)
        jacobian_array[9] = -1.0*std::sin(q[0]);

        //(4,1)
        jacobian_array[10]=std::cos(q[0]);

        //(5,1)
        jacobian_array[11]=0.0;

        // Column 2
        
        //(0,2)
        jacobian_array[12]=0.384*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) - 0.08249*std::cos(q[2])*std::sin(q[0]) - 0.08249*std::cos(q[0])*
        std::cos(q[1])*std::sin(q[2]) + 0.08249*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[1])*
        std::cos(q[3])*std::sin(q[2]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*
        std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[2])*std::sin(q[5]);

        //(1,2)
        jacobian_array[13]=0.08249*std::cos(q[0])*std::cos(q[2]) - 0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 0.08249*std::cos(q[0])*
        std::cos(q[2])*std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*
        std::sin(q[3]) + 0.08249*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[3])*
        std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]);

        //(2,2)
        jacobian_array[14]=0.08249*std::sin(q[1])*std::sin(q[2]) - 0.08249*std::cos(q[3])*std::sin(q[1])*std::sin(q[2]) - 0.384*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::sin(q[1])*std::sin(q[2])*
        std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) + 0.1069*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]);

        //(3,2)
        jacobian_array[15]=std::cos(q[0])*std::sin(q[1]);

        //(4,2)
        jacobian_array[16]=std::sin(q[0])*std::sin(q[1]);

        //(5,2)
        jacobian_array[17]=std::cos(q[1]);


        // Column 3

        //(0,3)
        jacobian_array[18]=0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.384*std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*
        std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) -
        0.0879*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) +
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[0])*
        std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) + 0.1069*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*
        std::sin(q[3])*std::sin(q[5]);

        //(1,3)
        jacobian_array[19]=0.08249*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.384*
        std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.384*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) - 0.384*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[0]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[2]) + 0.08249*std::cos(q[1])*std::cos(q[2])*
        std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[5]) + 0.1069*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1])*std::sin(q[3]) - 0.0879*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.0879*
        std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]);

        //(2,3)
        jacobian_array[20]=0.384*std::cos(q[2])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[2])*
        std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.08249*std::cos(q[1])*std::cos(q[3]) + 0.1069*
        std::cos(q[1])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
        std::cos(q[2])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]);

        //(3,3)
        jacobian_array[21]=std::cos(q[2])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::sin(q[2]);

        //(4,3)
        jacobian_array[22]=std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2]);

        //(5,3)
        jacobian_array[23]=-1.0*std::sin(q[1])*std::sin(q[2]);

        // Column 4

        //(0,4)
        jacobian_array[24]=0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[2])*
        std::cos(q[4])*std::sin(q[0])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0]) - 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) + 0.1069*
        std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[4]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4])*std::sin(q[5]);

        //(1,4)
        jacobian_array[25]=0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*
        std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[3])*
        std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
        std::sin(q[3])*std::sin(q[4]) - 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[0])*std::sin(q[4])*std::sin(q[5]);

        //(2,4)
        jacobian_array[26]=0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[3])*
        std::sin(q[4]) + 0.1069*std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::sin(q[3])*std::sin(q[4])*
        std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[1])*std::sin(q[4])*std::sin(q[5]);

        //(3,4)
        jacobian_array[27]=std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*
        std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]);

        //(4,4)
        jacobian_array[28]=std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 1.0*std::cos(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]);

        //(5,4)
        jacobian_array[29]=std::cos(q[1])*std::cos(q[3]) + std::cos(q[2])*std::sin(q[1])*std::sin(q[3]);

        // Column 5

        //(0,5)
        jacobian_array[30]=0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[4]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*
        std::sin(q[4])*std::sin(q[5]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*
        std::sin(q[5]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*
        std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[2]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.0879*
        std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(1,5)
        jacobian_array[31]=0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) + 0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
        std::sin(q[5]) + 0.1069*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*
        std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
        std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(2,5)
        jacobian_array[32]=0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.1069*
        std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*
        std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
        std::sin(q[1])*std::sin(q[5]);

        //(3,5)
        jacobian_array[33]=std::cos(q[2])*std::cos(q[4])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2]) + 1.0*
        std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 1.0*
        std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4]);

        //(4,5)
        jacobian_array[34]=std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[4]) + std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4]) + std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[4]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[4]);

        //(5,5)
        jacobian_array[35]=std::cos(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[4])*
        std::sin(q[1])*std::sin(q[2]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::sin(q[1])*std::sin(q[4]);

        // Column 6

        //(0,6)
        jacobian_array[36]=0;

        //(1,6)
        jacobian_array[37]=0;

        //(2,6)
        jacobian_array[38]=0;

        //(3,6)
        jacobian_array[39]=1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 1.0*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) - 1.0*std::cos(q[0])*std::cos(q[3])*
        std::cos(q[5])*std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[5]) + 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(4,6)
        jacobian_array[40]=1.0*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1]) + 1.0*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 1.0*std::cos(q[1])*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[5]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(5,6)
        jacobian_array[41]=std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 1.0*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*
        std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);

        SharedData->jacobian = jacobian_array;
        return jacobian_array;
    }

    double constrainEEJerk(Eigen::VectorXd & desiredJerk, const franka::RobotState & robot_state){
        double max_j = 6500.0;
        double length = desiredJerk.norm();
        double j = desiredJerk.cwiseAbs().maxCoeff();
        if (j > max_j){
            desiredJerk *= max_j / j;
        }
        if (length == 0) return 1;
        return desiredJerk.norm() / length;
    }

    double constrainEEAcceleration(Eigen::VectorXd & desiredAcceleration, const franka::RobotState & robot_state){
        double max_a = 13.0;
        double length = desiredAcceleration.norm();

        Eigen::Map<const Eigen::VectorXd> lastAcceleration(robot_state.O_ddP_EE_c.data(), 6);
        Eigen::VectorXd desiredJerk = (desiredAcceleration - lastAcceleration) * 1000;
        constrainEEJerk(desiredJerk, robot_state);

        desiredAcceleration = lastAcceleration + desiredJerk / 1000;
        double a = desiredAcceleration.cwiseAbs().maxCoeff();
        if (a > max_a){
            desiredAcceleration *= max_a / a;
        }
        if (length == 0) return 1;
        return desiredAcceleration.norm() / length;
    }

    double constrainEEVelocity(Eigen::VectorXd & desiredVelocity, const franka::RobotState & robot_state) {
        double max_v = 1.7;
        double length = desiredVelocity.norm();
        Eigen::Map<const Eigen::VectorXd> lastVelocity(robot_state.O_dP_EE_c.data(), 6); 
        Eigen::VectorXd desiredAcceleration = (desiredVelocity - lastVelocity) * 1000;
        constrainEEAcceleration(desiredAcceleration, robot_state);

        desiredVelocity = lastVelocity + desiredAcceleration / 1000;
        double v = desiredVelocity.cwiseAbs().maxCoeff();
        if (v > max_v){
            desiredVelocity *= max_v / v; 
        }
        if (length == 0) return 1;
        return desiredVelocity.norm() / length;
    }

    double constrainEEPosition(Eigen::VectorXd & desiredPosition, const franka::RobotState & robot_state) {
        Eigen::Map<const Eigen::VectorXd> lastPosition(robot_state.O_T_EE.data(), 6);
        Eigen::VectorXd desiredVelocity = (desiredPosition - lastPosition) * 1000;
        constrainEEVelocity(desiredVelocity, robot_state);
        desiredPosition = lastPosition + desiredVelocity / 1000;
        return 1;
    }

    double constrainJointJerk(Eigen::VectorXd & desiredJointJerk, const franka::RobotState & robot_state) {
        double softThreshold = 0.9;
        array<double, 7> max_dddq = {7500*softThreshold, 3750*softThreshold, 5000*softThreshold, 6250*softThreshold, 7500*softThreshold, 10000*softThreshold, 10000*softThreshold};
        double length = desiredJointJerk.norm();

        double dddq;
        for (int i = 0; i < 7; i++){
            dddq = abs(desiredJointJerk[i]);
            if (dddq > max_dddq[i]){
                desiredJointJerk *= max_dddq[i] / dddq;
            }
        }
        
        if (length == 0) return 1;
        return desiredJointJerk.norm() / length;
    }

    double constrainJointAcceleration(Eigen::VectorXd & desiredJointAcceleration, const franka::RobotState & robot_state) {
        double softThreshold = 0.9;
        array<double, 7> max_ddq = {15*softThreshold, 7.5*softThreshold, 10*softThreshold, 12.5*softThreshold, 15*softThreshold, 20*softThreshold, 20*softThreshold};
        double length = desiredJointAcceleration.norm();
        Eigen::Map<const Eigen::VectorXd> lastJointAcceleration(SharedData->lastJointAcceleration.data(), 7);
        Eigen::VectorXd desiredJointJerk = (desiredJointAcceleration - lastJointAcceleration) * 1000;
        
        constrainJointJerk(desiredJointJerk, robot_state);
        
        desiredJointAcceleration = lastJointAcceleration + desiredJointJerk / 1000;
        
        double ddq;
        for (int i = 0; i < 7; i++) {
            ddq = abs(desiredJointAcceleration[i]);
            if (ddq > max_ddq[i]){
                desiredJointAcceleration *= max_ddq[i] / ddq;
            }
        }
        if (length == 0) return 1;
        return desiredJointAcceleration.norm() / length;
    }

    double constrainJointVelocity(Eigen::VectorXd & desiredJointVelocity, const franka::RobotState & robot_state) {
        double softThreshold = 0.9;
        array<double, 7> max_dq = {2.1750*softThreshold, 2.1750*softThreshold, 2.1750*softThreshold, 2.1750*softThreshold, 2.6100*softThreshold, 2.6100*softThreshold, 2.6100*softThreshold};
        double length = desiredJointVelocity.norm();
        Eigen::Map<const Eigen::VectorXd> lastJointVelocity(robot_state.dq_d.data(), 7);
        
        Eigen::VectorXd desiredJointAcceleration = (desiredJointVelocity - lastJointVelocity) * 1000;
        constrainJointAcceleration(desiredJointAcceleration, robot_state);

        desiredJointVelocity = lastJointVelocity + desiredJointAcceleration / 1000;

        double dq;
        for (int i = 0; i < 7; i++){
            dq = abs(desiredJointVelocity[i]);
            if (dq > max_dq[i]){
                desiredJointVelocity *= max_dq[i] / dq;
            }
        }
        if (length == 0) return 1;
        return desiredJointVelocity.norm() / length;
    }

    void constrainJointPosition(Eigen::VectorXd & desiredJointPosition, const franka::RobotState & robot_state) {
        Eigen::Map<const Eigen::VectorXd> lastJointPosition(robot_state.q_d.data(), 7);
        Eigen::VectorXd desiredJointVelocity = (desiredJointPosition - lastJointPosition) * 1000;
        constrainJointVelocity(desiredJointVelocity, robot_state);
        desiredJointPosition = lastJointPosition + desiredJointVelocity / 1000;
    }

    void constrainForces(Eigen::VectorXd & velocity, const franka::RobotState & robot_state) {
        double maxForce = 15;
        for (int i = 0; i < 3; i++){
            // If the force is too high, and the velocity would increase the force,
            // Then remove that velocity.
            // Doesn't account for RPY forces - 
            // I.E. velocity in x direction could increase twist on the EE.
            if (abs(robot_state.O_F_ext_hat_K[i]) > maxForce
                && velocity[i] * robot_state.O_F_ext_hat_K[i] >= 0){
                if (abs(velocity[i]) >= 0.0001){
                    velocity[i] = 0;//- 0.05 * velocity[i] / abs(velocity[i]);
                }
            } 
        }
    }

    //Input Cartesian position (6D) - default orientation 0,0,0 - control with joint velocities
    void runPositionController(char* ip = NULL){
        try {
            cout << "In runPositionController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            double lastJointHome = 0.8;
            std::array<double,7> q_goal = {{0.0,-0.4,0.0,-2.0,0.0,1.6, lastJointHome}};
            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;

            int count = 0;
            cout << "Model loaded" << endl;

            writeRobotState(robot.readOnce());
            std::array<double, 6> positionArray;
            Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(SharedData->current_state.O_T_EE.data()));
            Eigen::Vector3d positionVector(transformMatrix.translation());
            Eigen::Quaterniond orientationVector(transformMatrix.linear());
            EulerAngles euler = quaternionToEuler(orientationVector);
            for (size_t i = 0; i < 3; i++) {
                positionArray[i] = positionVector[i];
            }
            positionArray[3] = euler.roll - M_PI;
            positionArray[4] = euler.pitch;
            positionArray[5] = euler.yaw;
            writeCommandedPosition(positionArray);
    
            cout << "About to start" << endl;

            robot.control([=, &count, &lastJointHome](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::JointVelocities {
                writeRobotState(robot_state);
                array<double, 6> commandedPosition = readCommandedPosition();

                Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.linear());
                
                // Eigen initialise quaternions as w, x, y, z
                EulerAngles desired_a;
                //Adding pi as panda control consider pi, 0, 0 to be the vertical orientation
                desired_a.roll = commandedPosition[3]+M_PI;
                desired_a.pitch = commandedPosition[4];
                desired_a.yaw = commandedPosition[5];

                Eigen::Quaterniond desired_q=eulerToQuaternion(desired_a);
                
                Eigen::Quaterniond difference(desired_q*orientation.inverse());
                
                // Not working for all configuration 
                //https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
                //auto euler = difference.toRotationMatrix().eulerAngles(0, 1, 2);

                EulerAngles difference_a = quaternionToEuler(difference);

                double scaling_factor = 3;
                double v_x = (commandedPosition[0] - position[0]) * scaling_factor;
                double v_y = (commandedPosition[1] - position[1]) * scaling_factor;
                double v_z = (commandedPosition[2] - position[2]) * scaling_factor;
                double v_roll = difference_a.roll * scaling_factor;
                double v_pitch = difference_a.pitch * scaling_factor;
                double v_yaw = difference_a.yaw * scaling_factor;

                

                Eigen::VectorXd v(6);
                v << v_x, v_y, v_z, v_roll, v_pitch, v_yaw;
                //v.fill(0);
                constrainForces(v, robot_state);

                franka::JointVelocities output {0,0,0,0,0,0,0};
                if (SharedData->controlCamera) {
                    // If we are supposed to control the camera, (second to last link of the panda)
                    // Then we should use a truncate jacobian to solve the inverse kinematics.
                    // Then we try to home the last joint so you can see with the camera.
                    Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7).topLeftCorner(6,6).completeOrthogonalDecomposition().solve(v);
                    output = {{
                        jointVelocities[0], 
                        jointVelocities[1], 
                        jointVelocities[2], 
                        jointVelocities[3], 
                        jointVelocities[4], 
                        jointVelocities[5], 
                        lastJointHome - robot_state.q[6]
                    }};
                } else {
                    Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7).completeOrthogonalDecomposition().solve(v);
                    output = {{
                        jointVelocities[0], 
                        jointVelocities[1], 
                        jointVelocities[2], 
                        jointVelocities[3], 
                        jointVelocities[4], 
                        jointVelocities[5], 
                        jointVelocities[6]
                    }};
                }
                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;

                    return franka::MotionFinished(output);
                }
                count =  count + 1;
                if (count < 5) return {{0,0,0,0,0,0,0}};
                return output;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
        } catch(const exception& e) {
            cout << e.what() << endl;
        }
    }


    void runHybridController(char* ip = NULL){
        try {
            cout << "In runPositionController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0.0754606,-0.337453,0.150729,-2.46194,0.0587094,2.12597,0.972193}};

            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;

            int count = 0;
            cout << "Model loaded" << endl;

            writeRobotState(robot.readOnce());
            std::array<double, 6> positionArray;
            Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(SharedData->current_state.O_T_EE.data()));
            Eigen::Vector3d positionVector(transformMatrix.translation());
            for (size_t i = 0; i < 3; i++) {
                positionArray[i] = positionVector[i];
            }
            for (size_t i = 0; i < 3; i++) {
                positionArray[3 + i] = 0;
            }
            writeCommandedPosition(positionArray);
    
            cout << "About to start" << endl;

            robot.control([=, &count](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::JointVelocities {
                writeRobotState(robot_state);

                // Selection vector represents directions for position (admittance) control
                array<double, 3> currentSelectionMatrix = readSelectionVector();
                Eigen::VectorXd selection_vector(3);
                selection_vector << currentSelectionMatrix[0], currentSelectionMatrix[1], currentSelectionMatrix[2]; // Currently just cartesian directions and assumed no rotation
                Eigen::Matrix< double, 3, 3> position_selection_matrix = selection_vector.array().matrix().asDiagonal();
                
                Eigen::Matrix< double, 3, 3> force_selection_matrix;
                Eigen::MatrixXd eye3 = Eigen::MatrixXd::Identity(3, 3);
                force_selection_matrix =  eye3 - position_selection_matrix;
                
                // COME BACK TO THIS!!!
                array<double, 6> commandedPosition = readCommandedPosition();
                array<double, 6> commandedWrench = readCommandedFT();

                // TEMP FORCE THE COMMANDED POSITION TO BE THE SAME!!!
                //commandedPosition = {0.42, 0.1, 0.25, 0.0, 0.0, 0.0};

                Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.linear());

                array<double, 6> currentWrench = readFTForces();
                //cout << "Current Z Force: " << currentWrench[2] << endl;
                
                // Eigen initialise quaternions as w, x, y, z
                EulerAngles desired_a;
                //Adding pi as panda control consider pi, 0, 0 to be the vertical orientation
                desired_a.roll = commandedPosition[3]+M_PI;
                desired_a.pitch = commandedPosition[4];
                desired_a.yaw = commandedPosition[5];

                Eigen::Quaterniond desired_q=eulerToQuaternion(desired_a);
                
                Eigen::Quaterniond difference(desired_q*orientation.inverse());
                
                // Not working for all configuration 
                //https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
                //auto euler = difference.toRotationMatrix().eulerAngles(0, 1, 2);

                EulerAngles difference_a = quaternionToEuler(difference);

                double scaling_factor = 5;
                double v_x = (commandedPosition[0] - position[0]) * scaling_factor;
                double v_y = (commandedPosition[1] - position[1]) * scaling_factor;
                double v_z = (commandedPosition[2] - position[2]) * scaling_factor;
                double v_roll = difference_a.roll * scaling_factor;
                double v_pitch = difference_a.pitch * scaling_factor;
                double v_yaw = difference_a.yaw * scaling_factor;


                // Force Control Law - P controller w/ very low gain
                double Kfp = 0.0005;
                double v_x_f = Kfp*(commandedWrench[0]-currentWrench[0]);
                double v_y_f = Kfp*(commandedWrench[1]-currentWrench[1]);
                double v_z_f = Kfp*(commandedWrench[2]-currentWrench[2]);
                

                Eigen::VectorXd v_position(3);
                v_position << v_x, v_y, v_z;
                Eigen::VectorXd v_force(3);
                v_force << v_x_f, v_y_f, v_z_f;

                Eigen::VectorXd v_hybrid(3);
                v_hybrid = position_selection_matrix * v_position + force_selection_matrix * v_force;

                //cout << "HYBRID V: " << v_hybrid << endl;

                Eigen::VectorXd v_hybrid_expanded(6);
                v_hybrid_expanded << v_hybrid[0], v_hybrid[1], v_hybrid[2], v_roll, v_pitch, v_yaw;


                constrainForces(v_hybrid_expanded, robot_state);
                Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7).completeOrthogonalDecomposition().solve(v_hybrid_expanded);
                
                //constrainJointVelocity(jointVelocities, robot_state);
                //v = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7) * jointVelocities;
                
                franka::JointVelocities output = {{
                    jointVelocities[0], 
                    jointVelocities[1], 
                    jointVelocities[2], 
                    jointVelocities[3], 
                    jointVelocities[4], 
                    jointVelocities[5], 
                    jointVelocities[6]}};
                Eigen::VectorXd lastJointAcceleration = (jointVelocities - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                
                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;

                    return franka::MotionFinished(output);
                }
                count =  count + 1;
                if (count < 5) return {{0,0,0,0,0,0,0}};
                return output;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
        } catch(const exception& e) {
            cout << e.what() << endl;
        }
    }

    //Input Cartesian velocity, control with joint velocities
    void runVelocityController(char* ip = NULL){
        try {
            cout << "In runVelocityController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0.0,-0.4,0.0,-2.0,0.0,1.6,0.8}};
            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;

            int count = 0;
            cout << "Model loaded" << endl;

            writeRobotState(robot.readOnce());
            cout << "About to start" << endl;

            robot.control([=, &count](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::JointVelocities {
                writeRobotState(robot_state);

                Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(readCommandedVelocity().data(), 6);
                addNoise(v);
                constrainForces(v, robot_state);
                Eigen::MatrixXd jacobian = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7);
                Eigen::VectorXd jointVelocities = jacobian.completeOrthogonalDecomposition().solve(v);
                constrainJointVelocity(jointVelocities, robot_state);
                franka::JointVelocities output = {{
                    jointVelocities[0], 
                    jointVelocities[1], 
                    jointVelocities[2], 
                    jointVelocities[3], 
                    jointVelocities[4], 
                    jointVelocities[5],
                    jointVelocities[6]}};
                Eigen::VectorXd lastJointAcceleration = (jointVelocities - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                
                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;
                    return franka::MotionFinished(output);
                }
                count =  count + 1;
                if (count < 5) return {{0,0,0,0,0,0,0}};
                return output;
            });
        } catch (const franka::ControlException& e) {
            printControlError(e);
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
        } catch(const exception& e) {
            cout << e.what() << endl;
        }
    }
    // Input joint positions (angles), control with joint velocities
    void runJointPositionController(char* ip = NULL){
        try {
            cout << "In runJointPositionController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0.0,-0.4,0.0,-2.0,0.0,1.6,0.8}};
            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;
            
            writeRobotState(robot.readOnce());
            
            cout << "About to start" << endl;

            int count = 0;
            robot.control([=, &count](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                PandaController::writeRobotState(robot_state);
                array<double, 7> joint_angles = readJointAngles();

                double scale = 5;
                Eigen::VectorXd joint_velocity(7);
                joint_velocity << 
                    (joint_angles[0] - robot_state.q[0]) * scale,
                    (joint_angles[1] - robot_state.q[1]) * scale,
                    (joint_angles[2] - robot_state.q[2]) * scale,
                    (joint_angles[3] - robot_state.q[3]) * scale,
                    (joint_angles[4] - robot_state.q[4]) * scale,
                    (joint_angles[5] - robot_state.q[5]) * scale,
                    (joint_angles[6] - robot_state.q[6]) * scale;
                addNoise(joint_velocity);

                // Take the desired joint velocities. Convert to cartesian space.
                // Attempt to shift the target cartesian velocity to avoid a collision.
                // Go back to joint space using a simplified IK via the Jacobian. <-- This part might be a little bit sketchy.
                // Then double check that we aren't exceeding joint limits.
                Eigen::VectorXd v = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7) * joint_velocity;
                constrainForces(v, robot_state);
                joint_velocity = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7).completeOrthogonalDecomposition().solve(v);
                constrainJointVelocity(joint_velocity, robot_state);
                Eigen::VectorXd lastJointAcceleration = (joint_velocity - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                franka::JointVelocities velocities = {
                    joint_velocity[0],
                    joint_velocity[1],
                    joint_velocity[2],
                    joint_velocity[3],
                    joint_velocity[4],
                    joint_velocity[5],
                    joint_velocity[6]};

                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;
                    return franka::MotionFinished(velocities);
                }
                count += 1;
                if (count < 5) return {{0,0,0,0,0,0,0}};
                return velocities;
            });
        } catch (const franka::ControlException& e) {
            printControlError(e);
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
        } catch(const exception& e) {
            cout << e.what() << endl;
        }
    }

    // Input joint velocity, controls joint velocities
    // TODO: currently takes position input instead of velocity
    void runJointVelocityController(char* ip = NULL){
        try {
            cout << "In runJointVelocityController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0.0,-0.4,0.0,-2.0,0.0,1.6,0.8}};
            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;

            double time_max = 180.0;
            double omega_max = 1.0;
            double time = 0.0;
            double delta_t = 0.001;
            std::array<double, 7> joint_pos;
            std::array<double, 7> joint_vel;
            std::array<double, 7> joint_acc;
            cout << "Model loaded" << endl;
            
            writeRobotState(robot.readOnce());
            cout << "About to start" << endl;

            robot.control([=, &time, &joint_pos, &joint_vel, &joint_acc](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                time += period.toSec();
                PandaController::writeRobotState(robot_state);
                array<double, 7> joint_angles = readJointAngles();

                double scale = 5;
                Eigen::VectorXd joint_velocity(7);
                joint_velocity << 
                    (joint_angles[0] - robot_state.q[0]) * scale,
                    (joint_angles[1] - robot_state.q[1]) * scale,
                    (joint_angles[2] - robot_state.q[2]) * scale,
                    (joint_angles[3] - robot_state.q[3]) * scale,
                    (joint_angles[4] - robot_state.q[4]) * scale,
                    (joint_angles[5] - robot_state.q[5]) * scale,
                    (joint_angles[6] - robot_state.q[6]) * scale;

                //cout<<constrainJointVelocity(joint_velocity, robot_state)<<endl;
                constrainJointVelocity(joint_velocity, robot_state);
                Eigen::VectorXd lastJointAcceleration = (joint_velocity - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                franka::JointVelocities velocities = {
                    joint_velocity[0],
                    joint_velocity[1],
                    joint_velocity[2],
                    joint_velocity[3],
                    joint_velocity[4],
                    joint_velocity[5],
                    joint_velocity[6]};

                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;
                    return franka::MotionFinished(velocities);
                }
                return velocities;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            //cout << franka::logToCSV(e.log) << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
    }

    //Does not control the panda
    void noController(char* ip = NULL){
        try {
            cout << "In noController" << endl;
            franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
            cout << "Robot connected" << endl;
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0.0,-0.4,0.0,-2.0,0.0,1.6,0.8}};
            MotionGenerator motion_generator(0.5, q_goal);
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default behavior set" << endl;

            double time_max = 30.0;
            double time = 0.0;
            double delta_t = 0.001;
            cout << "Model loaded" << endl;
            
            writeRobotState(robot.readOnce());
            cout << "About to start" << endl;

            robot.control([=, &time](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                time += period.toSec();
                writeRobotState(robot_state);

                franka::JointVelocities velocities = {0,0,0,0,0,0,0};

                if (!isRunning()) {
                    cout << endl << "Finished motion, shutting down example" << endl;
                    return franka::MotionFinished(velocities);
                }
                return velocities;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            //cout << franka::logToCSV(e.log) << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
    }

    void homeGripper() {
        try {
            p_gripper->stop();
            p_gripper->homing();
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
        }
    }

    void graspObj(std::function<void ()> onGrasp) {
        try {
            p_gripper->stop();
            SharedData->isGripperMoving = true;
            p_gripper->grasp(maxGripperWidth/2,0.2,20,0.5,0.5);
            SharedData->isGripperMoving = false;
            SharedData->grasped = true;
            PandaController::writeGripperState();
            if (onGrasp != NULL) onGrasp();
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            SharedData->isGripperMoving = false;
        }
    }

    void graspObject(std::function<void ()> onGrasp){
        if(SharedData->isGripperMoving || SharedData->grasped) {
            return;
        }
        thread(graspObj, onGrasp).detach();
    }
    void releaseObj(std::function<void ()> onRelease) {
        try {
            p_gripper->stop();
            SharedData->isGripperMoving = true;
            p_gripper->move(maxGripperWidth,0.2);
            SharedData->isGripperMoving = false;
            SharedData->grasped = false;
            PandaController::writeGripperState();
            if (onRelease != NULL) onRelease();
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
        }
    }

    void releaseObject(std::function<void ()> onRelease) {
        if(SharedData->isGripperMoving || !SharedData->grasped) {
            return;
        }
        thread(releaseObj, onRelease).detach();
    }

    void toggleGrip(std::function<void ()> onToggle) {
        if (SharedData->isGripperMoving){
            return;
        }
        if (SharedData->grasped) {
            releaseObject(onToggle);
        } else {
            graspObject(onToggle);
        }
    }

    void initializeMemory() {
        shared_memory_object shm(open_or_create, "Panda Controller", read_write);
        shm.truncate(sizeof(shared_data));
        memoryRegion = new mapped_region(shm, read_write);
        new (memoryRegion->get_address()) shared_data;
        SharedData = (shared_data*)memoryRegion->get_address();
        
        //Reset velocity commands
        std::array<double, 6> velocity;
        velocity[0] = 0;
        velocity[1] = 0;
        velocity[2] = 0;
        velocity[3] = 0;
        velocity[4] = 0;
        velocity[5] = 0;
        PandaController::writeCommandedVelocity(velocity);
        std::array<double, 7> default_angles;
        default_angles[0] = 0.0;
        default_angles[1] = -0.4;
        default_angles[2] = 0.0;
        default_angles[3] = -2.0;
        default_angles[4] = 0.0;
        default_angles[5] = 1.6;
        default_angles[6] = 0.8;
        PandaController::writeJointAngles(default_angles);
    }

    void setup_ft(){
        double cpt = 1000000;
        struct sockaddr_in addr;	/* Address of Net F/T. */
        struct hostent *he;			/* Host entry for Net F/T. */
        int err;					/* Error status of operations. */

        /* Calculate number of samples, command code, and open socket here. */
        socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketHandle == -1) {
            cout << "Can't Get Socket Handle. Exiting." << endl;
            exit(1);
        }
        
        *(uint16*)&request[0] = htons(0x1234); /* standard header. */
        *(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
        *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
        
        /* Sending the request. */
        he = gethostbyname("192.168.2.2");
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);
        
        err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
        if (err == -1) {
            cout << "Can't Connect to Socket. Exiting." << endl;
            exit(2);
        }
	
    }

    void bias_ft(){
        // Read once
         double cpf = 1000000;
            int i;						/* Generic loop/array index. */
            RESPONSE resp;				/* The structured response received from the Net F/T. */
            byte response[36];			/* The raw response data received from the Net F/T. */
            
            // Transform into the correct frame based on Panda Pose
            franka::RobotState state = PandaController::readRobotState();

            // Get feedback from FT sensor - THIS NEEDS TO BE MOVED TO SHARED MEMORY
            send(socketHandle, request, 8, 0 );

            /* Receiving the response. */
            recv( socketHandle, response, 36, 0 );
            resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
            resp.ft_sequence = ntohl(*(uint32*)&response[4]);
            resp.status = ntohl(*(uint32*)&response[8]);
            for( i = 0; i < 6; i++ ) {
                resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
            }

            std::array<double, 6> ft_sensor = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            ft_sensor[0]=resp.FTData[0]/cpf;
            ft_sensor[1]=resp.FTData[1]/cpf;
            ft_sensor[2]=resp.FTData[2]/cpf;
            ft_sensor[3]=resp.FTData[3]/cpf;
            ft_sensor[4]=resp.FTData[4]/cpf;
            ft_sensor[5]=resp.FTData[5]/cpf;
            PandaController::SharedData->ft_bias = ft_sensor;
            cout << "Force Torque Sensor Bias: " << endl <<
             ft_sensor[0] << endl <<
             ft_sensor[1] << endl <<
             ft_sensor[2] << endl <<
             ft_sensor[3] << endl <<
             ft_sensor[4] << endl <<
             ft_sensor[5] << endl;


    }

    void read_ft(){
        while(PandaController::isRunning()){
            // Get the current FT reading from the sensor
            double cpf = 1000000;
            int i;						/* Generic loop/array index. */
            RESPONSE resp;				/* The structured response received from the Net F/T. */
            byte response[36];			/* The raw response data received from the Net F/T. */
            
            // Transform into the correct frame based on Panda Pose
            franka::RobotState state = PandaController::readRobotState();

            // Get feedback from FT sensor - THIS NEEDS TO BE MOVED TO SHARED MEMORY
            send(socketHandle, request, 8, 0 );

            /* Receiving the response. */
            recv( socketHandle, response, 36, 0 );
            resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
            resp.ft_sequence = ntohl(*(uint32*)&response[4]);
            resp.status = ntohl(*(uint32*)&response[8]);
            for( i = 0; i < 6; i++ ) {
                resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
            }

            std::array<double, 6> ft_sensor = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::array<double, 6> ft_bias = PandaController::SharedData->ft_bias;
            ft_sensor[0]=resp.FTData[0]/cpf-ft_bias[0];
            ft_sensor[1]=resp.FTData[1]/cpf-ft_bias[1];
            ft_sensor[2]=resp.FTData[2]/cpf-ft_bias[2];
            ft_sensor[3]=resp.FTData[3]/cpf-ft_bias[3];
            ft_sensor[4]=resp.FTData[4]/cpf-ft_bias[4];
            ft_sensor[5]=resp.FTData[5]/cpf-ft_bias[5];
            writeFTForces(ft_sensor);
        }
    
    }

    pid_t initPandaController(ControlMode mode, char* ip) {
        initializeMemory();

        //Setting the Panda IP
        if (ip == NULL) {
            //ip = "10.134.71.22";
            ip = std::getenv("PANDA_IP");
            cout<<"Panda ip is "<<ip<<endl;
                
        }

        //Initializing the gripper/ setting the max width
        p_gripper = new franka::Gripper(ip);
        // if (!homeGripper()){
        //     cout << "Could not home gripper\n";
        // }
        franka::GripperState state = p_gripper->readOnce();
        maxGripperWidth = state.max_width;
        p_gripper->move(maxGripperWidth, 0.2);
        PandaController::writeGripperState();
        std::cout << "Starting" << std::endl;
        SharedData->running = true;
        SharedData->start_time = std::chrono::system_clock::now();

        // Fork process for Panda Controller
        pid_t pid = fork();
        if (pid == 0) {
            switch(mode){
                case PandaController::ControlMode::CartesianVelocity:
                    runVelocityController(ip);
                    break;
                case PandaController::ControlMode::CartesianPosition:
                    runPositionController(ip);
                    break;
                case PandaController::ControlMode::JointVelocity:
                    runJointVelocityController(ip);
                    break;
                case PandaController::ControlMode::JointPosition:
                    runJointPositionController(ip);
                    break;
                case PandaController::ControlMode::HybridControl:
                    runHybridController(ip);
                    break;
                case PandaController::ControlMode::None:
                    noController(ip);
                    break;
             }
            stopControl();
            exit(0);
        }

        // Fork process for the force torque sensor
        pid_t pid_child_2 = fork();
        if (pid_child_2 == 0) {
            setup_ft();
            bias_ft();
            read_ft();
            exit(0);
        }

        return pid;
    }


    /** 
     *  Shared Memory Accessors
     * */

    void stopControl() {
        cout << "Control stopping" << endl;
        SharedData->running = false;
    }

    void startLogging() {
        if (SharedData == NULL) throw "Must initialize shared memory space first";
        SharedData->logging = true;
    }
    void stopLogging() {
        if (SharedData == NULL) return;
        SharedData->logging = false;
    }

    bool isRunning() {
        if (SharedData == NULL) return false;
        return SharedData->running;
    }

    bool isMotionDone(std::array<double, 7> command) {
        long timePoint = (long)command[0];
        long deltaT = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::time_point<std::chrono::system_clock>(std::chrono::milliseconds(timePoint)) - std::chrono::system_clock::now()
        ).count();
        // If the waypoint is in the future. 
        // Then we aren't done yet
        if (deltaT > 0) {
            return false;
        }
        // If it is sufficiently far in the past
        // Then lets forget about it.
        if (deltaT < 0){
            return true;
        }

        Eigen::Affine3d transform(Eigen::Matrix4d::Map(SharedData->current_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        double distance = (position[0] - command[1]) * (position[0] - command[1]) +
                            (position[1] - command[2]) * (position[1] - command[2]) + 
                            (position[2] - command[3]) * (position[2] - command[3]);

        // If distance^2 is less than 1mm^2, then were good. 
        if (distance < 0.00001){
            return true;
        }

        return false;
    }

    void updateInterpolationCoefficients() {
        double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        // Fit to poly of the form w0 + w1 t + w2 t^2 + ...
        // This has 3 parameters from the position, velocity and acceleration at the beginning.
        // + 1 parameter for each positional waypoint in the middle.
        // + 3 more parameters for the end goal position, end velocity (0) and and acceleration (0);
        // Only interpolate between the next few points.
        int maxPositionConstraints = 0;
        int lastCommand = min(SharedData->lastCommand, SharedData->currentCommand + maxPositionConstraints);
        int nPositionConstraints = lastCommand - SharedData->currentCommand;
        int m = maxPositionConstraints + 6;
        // We will chop out some rows and columns of the matrix if we don't 
        // have the maximum number of intermediate points. This is so we can store the 
        // coefficients matrix in a static size;
        int variableM = nPositionConstraints + 6;
        Eigen::MatrixXd A(m,m); A.fill(0);
        Eigen::MatrixXd B(m, 6); B.fill(0);
        // t goes from 0 (now) to 1 (last point)
        double timeWidth = SharedData->commanded_position[lastCommand][0] - now_ms;
        // First 3 rows look like:
        // 1 0 0 0 ...
        // 0 1 0 0 ...
        // 0 0 2 0 ...
        A(0,0) = 1;
        A(1,1) = 1;
        A(2,2) = 2;

        Eigen::Affine3d transformMatrix(Eigen::Matrix4d::Map(SharedData->current_state.O_T_EE.data()));
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
        //B.row(1) = Eigen::Map<const Eigen::VectorXd>(SharedData->currentVelocity.data(), 6);
        //B.row(2) = Eigen::Map<const Eigen::VectorXd>(SharedData->currentAcceleration.data(), 6); // currentAcceleration is currently not set.

        // These ones look like
        // 1 t t^2 t^3 ...
        // for each time point
        for (size_t i = 3; i <= 3 + nPositionConstraints; i++) {
            int commandIdx = i - 3 + SharedData->currentCommand;
            if (commandIdx > lastCommand) break;
            double t = (SharedData->commanded_position[commandIdx][0] - now_ms) / timeWidth;
            double coeff = 1;
            for (size_t j = 0; j < variableM; j++) {
                A(i, j) = coeff;
                coeff *= t;
            }
            array<double, 7> command = SharedData->commanded_position[commandIdx];
            B.row(i) << command[1], command[2], command[3], command[4], command[5], command[6];
        }
        // Last two lines look like
        // 0 1 2 3 4 5 ...
        // 0 0 6 12 20 ...
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 1; j < variableM; j++)
            {
                double coeff = j;
                for (size_t k = 1; k <= i; k++) {
                    coeff *= j-k;
                }
                A(nPositionConstraints + 4 + i, j) = coeff;
            }
            // Assume that the last position in the path is commanding 0 velocity and acceleration.
            // B.row(nPositionConstraints + 4 + i) = <0...>
        }

        Eigen::MatrixXd W = A.completeOrthogonalDecomposition().solve(B);
        for (size_t i = 0; i < (m * 6); i++) {
            SharedData->interpolationCoefficients[i] = W.data()[i];
        }
        SharedData->pathStartTime_ms = (long)now_ms;
        SharedData->pathEndTime_ms = (long)SharedData->commanded_position[lastCommand][0];
    }

    std::array<double, 6> readCommandedPosition(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);

        auto command = SharedData->commanded_position[SharedData->currentCommand];
        bool pathChanged = false;
        while (SharedData->currentCommand < SharedData->lastCommand && isMotionDone(command)) {
            SharedData->currentCommand ++;
            command = SharedData->commanded_position[SharedData->currentCommand];
            pathChanged = true;
            cout << "path changed" << endl;
        }
        if (pathChanged) updateInterpolationCoefficients();

        double now_ms = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        double deltaT = command[0] - now_ms;
        // If we are told to go there *now*, then forget interpolation.
        if (deltaT < 0){
            std::array<double, 6> positionCommand;
            for (size_t i = 0; i < 6; i++) {
                positionCommand[i] = command[i+1];
            }
            return positionCommand;
        }

        // If we have a little time, interpolate between all of the remaining points
        // to find a reasonable trajectory.
        // Now compute the position for 1ms in the future.

        double timeWidth = SharedData->pathEndTime_ms - SharedData->pathStartTime_ms;
        double t = (now_ms - SharedData->pathStartTime_ms) / timeWidth;
        Eigen::MatrixXd W = Eigen::Map<Eigen::MatrixXd>(SharedData->interpolationCoefficients.data(), 6, 6);
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


    std::array<double, 6> readCommandedFT(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);

       return SharedData->FT_command;
    }

     void writeCommandedFT(std::array<double, 6> data){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);

        SharedData->FT_command = data;
    }

    void writeCommandedPosition(std::array<double, 6> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        std::array<double, 7> timeStamped;
        timeStamped[0] = -1;
        for (size_t i = 0; i < 6; i++) {
            timeStamped[i+1] = data[i];
        }

        SharedData->commanded_position[0] = timeStamped;
        SharedData->lastCommand = 0;
    }

    void writeCommandedPath(const std::array<double, 7>* data, const int & length) {
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        for (int i = 0; i < length; i++ ) {
            SharedData->commanded_position[i] = data[i];
        }
        SharedData->currentCommand = 0;
        SharedData->lastCommand = length - 1;
        updateInterpolationCoefficients();

    }

    void setControlCamera(const bool & controlCamera) {
        SharedData->controlCamera = controlCamera;
    }

    std::array<double, 6> readCommandedVelocity() {
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->commanded_velocity;
    }
    void writeCommandedVelocity(std::array<double, 6> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->commanded_velocity = data;
    }

    std::array<double, 6> readFTForces() {
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->ft_sensor;
    }
    void writeFTForces(std::array<double, 6> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->ft_sensor = data;
    }
    
    std::array<double, 7> readPoseGoal(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->pose_goal;
    }

    std::array<double, 3> readSelectionVector(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->selection_vector;
    }

    void writeSelectionVector(std::array<double, 3> data){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->selection_vector = data;
    }
    

    void writePoseGoals(std::array<double, 7> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->pose_goal = data;
    }
    
    std::array<double, 7> readJointAngles(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->joint_angles;
    }
    void writeJointAngles(std::array<double, 7> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->joint_angles = data;
    }
    
    franka::RobotState readRobotState(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->current_state;
    }

    void writeRobotState(franka::RobotState data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->jacobian = calculatePandaJacobian(data.q);
        SharedData->iteration++;
        if (SharedData->logging) {
            SharedData->buffer_end++;
            if (SharedData->buffer_end > 1000){
                SharedData->buffer_end = 1;
                SharedData->buffer_start = 0;
            }
            SharedData->buffer[SharedData->buffer_end-1] = data;
            SharedData->timestamps[SharedData->buffer_end-1] = (std::chrono::system_clock::now() - SharedData->start_time).count();
        }
        SharedData->current_state = data;
    }
    franka::GripperState readGripperState() {
        if (SharedData == NULL) throw "Must initialize shared memory space first";
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->gripper_state;
    }

    void writeGripperState() {
        if (SharedData == NULL) return;
        franka::GripperState state = p_gripper->readOnce(); 
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->gripper_state = state;
    }

    void consumeBuffer(int &count, franka::RobotState* result, long* times){
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        count = 0;
        for (int i = SharedData->buffer_start; i < SharedData->buffer_end; i++){
            times[count] = SharedData->timestamps[i];
            result[(count)++] = SharedData->buffer[i];
        }
        SharedData->buffer_start = 0;
        SharedData->buffer_end = 0;
    }


} //end namespace PandaController
