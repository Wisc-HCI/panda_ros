#include "Kinematics.h"
#include <array>
#include <cmath>
#include <Eigen/Core>

using namespace std;

namespace PandaController {

    Eigen::Matrix<double, 4, 4> transformFromDH(double a, double d, double alpha, double theta) {
        Eigen::MatrixXd trans(4,4);
        // Column major modified DH parameters
        trans << cos(theta), sin(theta) * cos(alpha), sin(theta) * sin(alpha), 0,
                -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                          0,             -sin(alpha),              cos(alpha), 0,
                          a,         -d * sin(alpha),          d * cos(alpha), 1;
        return trans.transpose();
    }

    Eigen::Matrix<double, 4, 4> transformDerivativeFromDH(double a, double d, double alpha, double theta) {
        Eigen::MatrixXd trans(4,4);
        // Column major modified DH parameters
        trans << -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                 -cos(theta),-sin(theta) * cos(alpha),-sin(theta) * sin(alpha), 0,
                          0,                        0,                       0, 0,
                          0,                        0,                       0, 0;
        return trans.transpose();
    }

    Eigen::Matrix<double, 4, 4> EEtransform(array<double, 7> q) {
        // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        return  transformFromDH(       0,   0.333,       0, q[0]) *
                transformFromDH(       0,       0, -M_PI/2, q[1]) *
                transformFromDH(       0, 0.31599,  M_PI/2, q[2]) *
                transformFromDH( 0.08249,       0,  M_PI/2, q[3]) *
                transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]) *
                transformFromDH(       0,       0,  M_PI/2, q[5]) *
                transformFromDH(  0.0879,       0,  M_PI/2, q[6]) *
                transformFromDH(       0,  0.1069,       0,    0);
    }

    array<double, 16> calculatePandaEE(array<double, 7> q){
        array<double, 16> transform{};
        // Pointer magic
        Eigen::Map<Eigen::MatrixXd>(transform.data(), 4, 4) = EEtransform(q);
        return transform;
    }

    array<double, 42> pandaJacobian(array<double, 7> q){
        array<Eigen::Matrix<double, 4, 4>, 8> transforms {
            transformFromDH(       0,   0.333,       0, q[0]),
            transformFromDH(       0,       0, -M_PI/2, q[1]),
            transformFromDH(       0, 0.31599,  M_PI/2, q[2]),
            transformFromDH( 0.08249,       0,  M_PI/2, q[3]),
            transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
            transformFromDH(       0,       0,  M_PI/2, q[5]),
            transformFromDH(  0.0879,       0,  M_PI/2, q[6]),
            transformFromDH(       0,  0.1069,       0,    0)
        };
        array<Eigen::Matrix<double, 4, 4>, 7> transforms_derivative {
            transformDerivativeFromDH(       0,   0.333,       0, q[0]),
            transformDerivativeFromDH(       0,       0, -M_PI/2, q[1]),
            transformDerivativeFromDH(       0, 0.31599,  M_PI/2, q[2]),
            transformDerivativeFromDH( 0.08249,       0,  M_PI/2, q[3]),
            transformDerivativeFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
            transformDerivativeFromDH(       0,       0,  M_PI/2, q[5]),
            transformDerivativeFromDH(  0.0879,       0,  M_PI/2, q[6])
        };

        auto trans = EEtransform(q);
        auto R = trans.topLeftCorner(3, 3);

        Eigen::Matrix<double, 6, 7> jacobian = Eigen::MatrixXd::Zero(6, 7);
        for (int j = 0; j < 7; j++) {
            Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
            for (int i = 0; i < transforms.size(); i++) {
                if (i == j) {
                    jac = jac * transforms_derivative[i];
                } else {
                    jac = jac * transforms[i];
                }
            }
            jacobian(0, j) = jac(0, 3);
            jacobian(1, j) = jac(1, 3);
            jacobian(2, j) = jac(2, 3);

            auto W = jac.topLeftCorner(3,3) * R.transpose();
            jacobian(3, j) = W(2,1); // w_x
            jacobian(4, j) = W(0,2); // w_y
            jacobian(5, j) = W(1,0); // w_z
        }
        
        array<double, 42> jacobian_array{};
        // Pointer magic
        Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7) = jacobian;
        return jacobian_array;
    }

    array<double, 42> calculatePandaJacobian(array<double, 7> q) {
        // Calculated using Modified DH-Parameters analytically
        // using joint anlges, q:
        array<double, 42> jacobian;
        // Column 0
        //(0,0)
        jacobian[0]=0.384*cos(q[0])*sin(q[2])*sin(q[3]) - 0.31599*sin(q[0])*
        sin(q[1]) - 0.384*cos(q[3])*sin(q[0])*sin(q[1]) - 0.08249*cos(q[0])*
        sin(q[2]) + 0.08249*sin(q[0])*sin(q[1])*sin(q[3]) - 0.08249*cos(q[1])*
        cos(q[2])*sin(q[0]) + 0.08249*cos(q[0])*cos(q[3])*sin(q[2]) + 0.08249*
        cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) - 0.0879*cos(q[0])*cos(q[2])*
        cos(q[5])*sin(q[4]) + 0.384*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3]) +
            0.1069*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[1]) - 0.1069*cos(q[0])*
            cos(q[5])*sin(q[2])*sin(q[3]) - 0.1069*cos(q[0])*cos(q[2])*sin(q[4])*
            sin(q[5]) - 0.0879*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[5]) +
            0.0879*cos(q[0])*sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[0])*
            cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) - 0.1069*cos(q[1])*cos(q[2])*
            cos(q[5])*sin(q[0])*sin(q[3]) - 0.1069*cos(q[0])*cos(q[3])*cos(q[4])*
            sin(q[2])*sin(q[5]) + 0.0879*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3])*
            sin(q[5]) + 0.0879*cos(q[1])*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[4]) -
            0.0879*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3]) + 0.1069*
            cos(q[1])*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.1069*cos(q[4])*
            sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[1])*cos(q[2])*
            cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0]) - 0.1069*cos(q[1])*cos(q[2])*
            cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5]);

        //(1,0)
        jacobian[1]=0.31599*cos(q[0])*sin(q[1]) - 0.08249*sin(q[0])*sin(q[2]) - 0.08249*
        cos(q[0])*sin(q[1])*sin(q[3]) + 0.08249*cos(q[3])*sin(q[0])*sin(q[2]) + 0.384*
        sin(q[0])*sin(q[2])*sin(q[3]) + 0.08249*cos(q[0])*cos(q[1])*cos(q[2]) + 0.384*
        cos(q[0])*cos(q[3])*sin(q[1]) - 0.08249*cos(q[0])*cos(q[1])*cos(q[2])*
        cos(q[3]) - 0.384*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3]) - 0.1069*cos(q[0])*
        cos(q[3])*cos(q[5])*sin(q[1]) + 0.0879*cos(q[0])*cos(q[3])*sin(q[1])*sin(q[5]) -
        0.0879*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[4]) - 0.1069*cos(q[5])*sin(q[0])*
        sin(q[2])*sin(q[3]) - 0.1069*cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5]) + 0.0879*
        sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5]) + 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*
        cos(q[5])*sin(q[3]) - 0.0879*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3])*sin(q[5]) -
        0.0879*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4]) + 0.0879*cos(q[0])*
        cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3]) - 0.0879*cos(q[3])*cos(q[4])*cos(q[5])*
        sin(q[0])*sin(q[2]) - 0.1069*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) +
        0.1069*cos(q[0])*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) - 0.1069*cos(q[3])*
        cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5]) + 0.0879*cos(q[0])*cos(q[1])*cos(q[2])*
        cos(q[3])*cos(q[4])*cos(q[5]) + 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*
        cos(q[4])*sin(q[5]);

        //(2,0)
        jacobian[2]=0.0;

        //(3,0)
        jacobian[3]=0.0;

        //(4,0)
        jacobian[4]=0.0;

        //(5,0)
        jacobian[5]=1.0;

        // Column 1
        //(0,1)
        jacobian[6]=0.31599*cos(q[0])*cos(q[1]) + 0.384*cos(q[0])*cos(q[1])*cos(q[3]) -
        0.08249*cos(q[0])*cos(q[2])*sin(q[1]) - 0.08249*cos(q[0])*cos(q[1])*sin(q[3]) -
        0.1069*cos(q[0])*cos(q[1])*cos(q[3])*cos(q[5]) + 0.08249*cos(q[0])*cos(q[2])*
        cos(q[3])*sin(q[1]) + 0.0879*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[5]) + 0.384*
        cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3]) + 0.0879*cos(q[0])*cos(q[1])*cos(q[4])*
        cos(q[5])*sin(q[3]) - 0.1069*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3]) +
        0.1069*cos(q[0])*cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[0])*
        cos(q[2])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[0])*cos(q[5])*sin(q[1])*
        sin(q[2])*sin(q[4]) + 0.1069*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) -
        0.0879*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]) - 0.1069*
        cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5]);

        //(1,1)
        jacobian[7]=0.31599*cos(q[1])*sin(q[0]) - 0.08249*cos(q[2])*sin(q[0])*sin(q[1]) -
        0.08249*cos(q[1])*sin(q[0])*sin(q[3]) + 0.384*cos(q[1])*cos(q[3])*sin(q[0]) -
        0.1069*cos(q[1])*cos(q[3])*cos(q[5])*sin(q[0]) + 0.08249*cos(q[2])*cos(q[3])*
        sin(q[0])*sin(q[1]) + 0.0879*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[5]) + 0.384*
        cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3]) + 0.0879*cos(q[1])*cos(q[4])*cos(q[5])*
        sin(q[0])*sin(q[3]) - 0.1069*cos(q[2])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3]) +
        0.1069*cos(q[1])*cos(q[4])*sin(q[0])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[2])*
        sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[5])*sin(q[0])*sin(q[1])*
        sin(q[2])*sin(q[4]) + 0.1069*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) -
        0.0879*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1]) - 0.1069*
        cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5]);

        //(2,1)
        jacobian[8]=0.08249*sin(q[1])*sin(q[3]) - 0.08249*cos(q[1])*cos(q[2]) - 0.384*cos(q[3])*
        sin(q[1]) - 0.31599*sin(q[1]) - 0.0879*cos(q[3])*sin(q[1])*sin(q[5]) + 0.08249*cos(q[1])*
        cos(q[2])*cos(q[3]) + 0.384*cos(q[1])*cos(q[2])*sin(q[3]) + 0.1069*cos(q[3])*cos(q[5])*
        sin(q[1]) - 0.1069*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3]) + 0.0879*cos(q[1])*cos(q[2])*
        sin(q[3])*sin(q[5]) + 0.0879*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4]) - 0.0879*cos(q[4])*
        cos(q[5])*sin(q[1])*sin(q[3]) + 0.1069*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.1069*
        cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[1])*cos(q[2])*cos(q[3])*
        cos(q[4])*cos(q[5]) - 0.1069*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]);

        //(3,1)
        jacobian[9] = -1.0*sin(q[0]);

        //(4,1)
        jacobian[10]=cos(q[0]);

        //(5,1)
        jacobian[11]=0.0;

        // Column 2
        
        //(0,2)
        jacobian[12]=0.384*cos(q[2])*sin(q[0])*sin(q[3]) - 0.08249*cos(q[2])*sin(q[0]) - 0.08249*cos(q[0])*
        cos(q[1])*sin(q[2]) + 0.08249*cos(q[2])*cos(q[3])*sin(q[0]) + 0.08249*cos(q[0])*cos(q[1])*
        cos(q[3])*sin(q[2]) + 0.384*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[3]) - 0.1069*cos(q[2])*
        cos(q[5])*sin(q[0])*sin(q[3]) + 0.0879*cos(q[2])*sin(q[0])*sin(q[3])*sin(q[5]) + 0.0879*
        cos(q[5])*sin(q[0])*sin(q[2])*sin(q[4]) + 0.1069*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) -
        0.0879*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[4]) - 0.0879*cos(q[2])*cos(q[3])*cos(q[4])*
        cos(q[5])*sin(q[0]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3]) - 0.1069*cos(q[0])*
        cos(q[1])*cos(q[2])*sin(q[4])*sin(q[5]) - 0.1069*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*
        sin(q[5]) + 0.0879*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[0])*
        cos(q[1])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[3])*
        cos(q[4])*sin(q[2])*sin(q[5]);

        //(1,2)
        jacobian[13]=0.08249*cos(q[0])*cos(q[2]) - 0.08249*cos(q[1])*sin(q[0])*sin(q[2]) - 0.08249*cos(q[0])*
        cos(q[2])*cos(q[3]) - 0.384*cos(q[0])*cos(q[2])*sin(q[3]) + 0.1069*cos(q[0])*cos(q[2])*cos(q[5])*
        sin(q[3]) + 0.08249*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2]) - 0.0879*cos(q[0])*cos(q[2])*sin(q[3])*
        sin(q[5]) - 0.0879*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[4]) + 0.384*cos(q[1])*sin(q[0])*sin(q[2])*
        sin(q[3]) - 0.1069*cos(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) + 0.0879*cos(q[0])*cos(q[2])*cos(q[3])*
        cos(q[4])*cos(q[5]) + 0.1069*cos(q[0])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]) - 0.0879*cos(q[1])*
        cos(q[2])*cos(q[5])*sin(q[0])*sin(q[4]) - 0.1069*cos(q[1])*cos(q[5])*sin(q[0])*sin(q[2])*
        sin(q[3]) - 0.1069*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5]) + 0.0879*cos(q[1])*sin(q[0])*
        sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[1])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*
        sin(q[2]) - 0.1069*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5]);

        //(2,2)
        jacobian[14]=0.08249*sin(q[1])*sin(q[2]) - 0.08249*cos(q[3])*sin(q[1])*sin(q[2]) - 0.384*sin(q[1])*
        sin(q[2])*sin(q[3]) + 0.0879*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[4]) + 0.1069*cos(q[5])*sin(q[1])*
        sin(q[2])*sin(q[3]) + 0.1069*cos(q[2])*sin(q[1])*sin(q[4])*sin(q[5]) - 0.0879*sin(q[1])*sin(q[2])*
        sin(q[3])*sin(q[5]) + 0.0879*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[2]) + 0.1069*cos(q[3])*
        cos(q[4])*sin(q[1])*sin(q[2])*sin(q[5]);

        //(3,2)
        jacobian[15]=cos(q[0])*sin(q[1]);

        //(4,2)
        jacobian[16]=sin(q[0])*sin(q[1]);

        //(5,2)
        jacobian[17]=cos(q[1]);


        // Column 3

        //(0,3)
        jacobian[18]=0.384*cos(q[3])*sin(q[0])*sin(q[2]) - 0.384*cos(q[0])*sin(q[1])*sin(q[3]) - 0.08249*
        sin(q[0])*sin(q[2])*sin(q[3]) - 0.08249*cos(q[0])*cos(q[3])*sin(q[1]) - 0.384*cos(q[0])*
        cos(q[1])*cos(q[2])*cos(q[3]) + 0.08249*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3]) + 0.1069*
        cos(q[0])*cos(q[5])*sin(q[1])*sin(q[3]) - 0.1069*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[2]) -
        0.0879*cos(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[5]) +
        0.1069*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[5]) - 0.0879*cos(q[0])*cos(q[1])*cos(q[2])*
        cos(q[3])*sin(q[5]) + 0.0879*cos(q[0])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]) + 0.1069*cos(q[0])*
        cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5]) + 0.0879*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[2])*
        sin(q[3]) + 0.1069*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[0])*cos(q[1])*
        cos(q[2])*cos(q[4])*cos(q[5])*sin(q[3]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[4])*
        sin(q[3])*sin(q[5]);

        //(1,3)
        jacobian[19]=0.08249*cos(q[0])*sin(q[2])*sin(q[3]) - 0.08249*cos(q[3])*sin(q[0])*sin(q[1]) - 0.384*
        sin(q[0])*sin(q[1])*sin(q[3]) - 0.384*cos(q[0])*cos(q[3])*sin(q[2]) - 0.384*cos(q[1])*cos(q[2])*
        cos(q[3])*sin(q[0]) + 0.1069*cos(q[0])*cos(q[3])*cos(q[5])*sin(q[2]) + 0.08249*cos(q[1])*cos(q[2])*
        sin(q[0])*sin(q[3]) - 0.0879*cos(q[0])*cos(q[3])*sin(q[2])*sin(q[5]) + 0.1069*cos(q[5])*sin(q[0])*
        sin(q[1])*sin(q[3]) - 0.0879*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.1069*cos(q[1])*cos(q[2])*
        cos(q[3])*cos(q[5])*sin(q[0]) - 0.0879*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[5]) + 0.0879*
        cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1]) - 0.0879*cos(q[0])*cos(q[4])*cos(q[5])*
        sin(q[2])*sin(q[3]) + 0.1069*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5]) - 0.1069*cos(q[0])*
        cos(q[4])*sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[1])*cos(q[2])*cos(q[4])*cos(q[5])*
        sin(q[0])*sin(q[3]) - 0.1069*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[0])*sin(q[3])*sin(q[5]);

        //(2,3)
        jacobian[20]=0.384*cos(q[2])*cos(q[3])*sin(q[1]) - 0.384*cos(q[1])*sin(q[3]) - 0.08249*cos(q[2])*
        sin(q[1])*sin(q[3]) - 0.0879*cos(q[1])*sin(q[3])*sin(q[5]) - 0.08249*cos(q[1])*cos(q[3]) + 0.1069*
        cos(q[1])*cos(q[5])*sin(q[3]) + 0.0879*cos(q[1])*cos(q[3])*cos(q[4])*cos(q[5]) - 0.1069*cos(q[2])*
        cos(q[3])*cos(q[5])*sin(q[1]) + 0.1069*cos(q[1])*cos(q[3])*cos(q[4])*sin(q[5]) + 0.0879*cos(q[2])*
        cos(q[3])*sin(q[1])*sin(q[5]) + 0.0879*cos(q[2])*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3]) + 0.1069*
        cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]);

        //(3,3)
        jacobian[21]=cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]);

        //(4,3)
        jacobian[22]=cos(q[1])*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[2]);

        //(5,3)
        jacobian[23]=-1.0*sin(q[1])*sin(q[2]);

        // Column 4

        //(0,4)
        jacobian[24]=0.0879*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[4]) - 0.1069*cos(q[2])*
        cos(q[4])*sin(q[0])*sin(q[5]) - 0.0879*cos(q[0])*cos(q[1])*cos(q[4])*cos(q[5])*
        sin(q[2]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[4])*sin(q[2])*sin(q[5]) - 0.0879*cos(q[0])*
        cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4]) - 0.0879*cos(q[2])*cos(q[4])*cos(q[5])*
        sin(q[0]) - 0.1069*cos(q[0])*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5]) + 0.1069*
        cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.0879*cos(q[0])*cos(q[1])*cos(q[2])*
        cos(q[3])*cos(q[5])*sin(q[4]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4])*sin(q[5]);

        //(1,4)
        jacobian[25]=0.0879*cos(q[0])*cos(q[2])*cos(q[4])*cos(q[5]) + 0.1069*cos(q[0])*cos(q[2])*cos(q[4])*
        sin(q[5]) - 0.0879*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[2]) - 0.0879*cos(q[0])*cos(q[3])*
        cos(q[5])*sin(q[2])*sin(q[4]) - 0.1069*cos(q[1])*cos(q[4])*sin(q[0])*sin(q[2])*sin(q[5]) - 0.1069*
        cos(q[0])*cos(q[3])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.0879*cos(q[5])*sin(q[0])*sin(q[1])*
        sin(q[3])*sin(q[4]) - 0.1069*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5]) - 0.0879*cos(q[1])*
        cos(q[2])*cos(q[3])*cos(q[5])*sin(q[0])*sin(q[4]) - 0.1069*cos(q[1])*cos(q[2])*cos(q[3])*
        sin(q[0])*sin(q[4])*sin(q[5]);

        //(2,4)
        jacobian[26]=0.0879*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[2]) - 0.0879*cos(q[1])*cos(q[5])*sin(q[3])*
        sin(q[4]) + 0.1069*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[5]) - 0.1069*cos(q[1])*sin(q[3])*sin(q[4])*
        sin(q[5]) + 0.0879*cos(q[2])*cos(q[3])*cos(q[5])*sin(q[1])*sin(q[4]) + 0.1069*cos(q[2])*cos(q[3])*
        sin(q[1])*sin(q[4])*sin(q[5]);

        //(3,4)
        jacobian[27]=sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*cos(q[3])*
        sin(q[1]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3]);

        //(4,4)
        jacobian[28]=cos(q[3])*sin(q[0])*sin(q[1]) - 1.0*cos(q[0])*
        sin(q[2])*sin(q[3]) - 1.0*cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3]);

        //(5,4)
        jacobian[29]=cos(q[1])*cos(q[3]) + cos(q[2])*sin(q[1])*sin(q[3]);

        // Column 5

        //(0,5)
        jacobian[30]=0.0879*cos(q[0])*cos(q[3])*cos(q[5])*sin(q[1]) + 0.1069*
        cos(q[0])*cos(q[3])*sin(q[1])*sin(q[5]) - 0.1069*cos(q[2])*cos(q[5])*sin(q[0])*
        sin(q[4]) + 0.0879*cos(q[5])*sin(q[0])*sin(q[2])*sin(q[3]) + 0.0879*cos(q[2])*sin(q[0])*
        sin(q[4])*sin(q[5]) + 0.1069*sin(q[0])*sin(q[2])*sin(q[3])*sin(q[5]) - 0.0879*cos(q[0])*
        cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*sin(q[3])*
        sin(q[5]) - 0.1069*cos(q[0])*cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4]) + 0.1069*cos(q[0])*
        cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3]) - 0.1069*cos(q[3])*cos(q[4])*cos(q[5])*
        sin(q[0])*sin(q[2]) + 0.0879*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.0879*cos(q[0])*
        cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.0879*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*
        sin(q[5]) + 0.1069*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5]) - 0.0879*
        cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]);

        //(1,5)
        jacobian[31]=0.1069*cos(q[0])*cos(q[2])*cos(q[5])*sin(q[4]) + 0.0879*cos(q[3])*cos(q[5])*sin(q[0])*
        sin(q[1]) - 0.0879*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3]) - 0.0879*cos(q[0])*cos(q[2])*sin(q[4])*
        sin(q[5]) + 0.1069*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[5]) - 0.1069*cos(q[0])*sin(q[2])*sin(q[3])*
        sin(q[5]) + 0.1069*cos(q[0])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) - 0.0879*cos(q[1])*cos(q[2])*
        cos(q[5])*sin(q[0])*sin(q[3]) - 0.0879*cos(q[0])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]) - 0.1069*
        cos(q[1])*cos(q[2])*sin(q[0])*sin(q[3])*sin(q[5]) - 0.1069*cos(q[1])*cos(q[5])*sin(q[0])*
        sin(q[2])*sin(q[4]) + 0.1069*cos(q[4])*cos(q[5])*sin(q[0])*sin(q[1])*sin(q[3]) + 0.0879*cos(q[1])*
        sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.0879*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*
        sin(q[5]) + 0.1069*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[0]) - 0.0879*cos(q[1])*
        cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5]);

        //(2,5)
        jacobian[32]=0.0879*cos(q[1])*cos(q[3])*cos(q[5]) + 0.1069*cos(q[1])*cos(q[3])*sin(q[5]) + 0.1069*
        cos(q[1])*cos(q[4])*cos(q[5])*sin(q[3]) + 0.0879*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3]) - 0.0879*
        cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5]) + 0.1069*cos(q[2])*sin(q[1])*sin(q[3])*sin(q[5]) + 0.1069*
        cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) - 0.0879*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) - 0.1069*
        cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]) + 0.0879*cos(q[2])*cos(q[3])*cos(q[4])*
        sin(q[1])*sin(q[5]);

        //(3,5)
        jacobian[33]=cos(q[2])*cos(q[4])*sin(q[0]) + cos(q[0])*cos(q[1])*cos(q[4])*sin(q[2]) + 1.0*
        cos(q[0])*sin(q[1])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) + 1.0*
        cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4]);

        //(4,5)
        jacobian[34]=cos(q[1])*cos(q[4])*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[2])*
        cos(q[4]) + cos(q[0])*cos(q[3])*sin(q[2])*sin(q[4]) + sin(q[0])*sin(q[1])*sin(q[3])*
        sin(q[4]) + cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[4]);

        //(5,5)
        jacobian[35]=cos(q[1])*sin(q[3])*sin(q[4]) - 1.0*cos(q[4])*
        sin(q[1])*sin(q[2]) - 1.0*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]);

        // Column 6

        //(0,6)
        jacobian[36]=0;

        //(1,6)
        jacobian[37]=0;

        //(2,6)
        jacobian[38]=0;

        //(3,6)
        jacobian[39]=1.0*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[5])*sin(q[3]) - 1.0*cos(q[5])*sin(q[0])*
        sin(q[2])*sin(q[3]) - 1.0*cos(q[2])*sin(q[0])*sin(q[4])*sin(q[5]) - 1.0*cos(q[0])*cos(q[3])*
        cos(q[5])*sin(q[1]) - 1.0*cos(q[0])*cos(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) + 1.0*cos(q[0])*
        cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) - 1.0*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[2])*
        sin(q[5]) + 1.0*cos(q[0])*cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]);

        //(4,6)
        jacobian[40]=1.0*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3]) - 1.0*cos(q[3])*cos(q[5])*sin(q[0])*
        sin(q[1]) + 1.0*cos(q[0])*cos(q[2])*sin(q[4])*sin(q[5]) + 1.0*cos(q[1])*cos(q[2])*cos(q[5])*
        sin(q[0])*sin(q[3]) + cos(q[0])*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]) - 1.0*cos(q[1])*
        sin(q[0])*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*
        sin(q[5]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[0])*sin(q[5]);

        //(5,6)
        jacobian[41]=cos(q[1])*cos(q[4])*sin(q[3])*sin(q[5]) - 1.0*cos(q[2])*cos(q[5])*sin(q[1])*
        sin(q[3]) - 1.0*cos(q[1])*cos(q[3])*cos(q[5]) + 1.0*sin(q[1])*sin(q[2])*sin(q[4])*
        sin(q[5]) - 1.0*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[5]);

        return jacobian;
    }
}