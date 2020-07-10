#include "Kinematics.h"
#include <array>
#include <vector>
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

    Eigen::Matrix<double, 4, 4> EEFromDHA(array<double, 7> q, vector<array<double, 3>> dha) {
        // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        
        Eigen::Matrix<double, 4, 4> ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            double angle = 0;
            if (i < q.size()){
                angle = q[i];
            }
            auto trans = transformFromDH(dha[i][0], dha[i][1], dha[i][2], angle);
        }
        return ee_trans;
    }

    array<double, 42> jacobianFromDHA(array<double, 7> q, vector<array<double, 3>> dha) {
        vector<Eigen::Matrix4d> transforms = vector<Eigen::Matrix4d>();
        vector<Eigen::Matrix4d> transforms_derivative = vector<Eigen::Matrix4d>();
        Eigen::Matrix4d ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            double angle = 0;
            if (i < q.size()){
                angle = q[i];
            }
            auto trans = transformFromDH(dha[i][0], dha[i][1], dha[i][2], angle);
            ee_trans = ee_trans * trans;
            transforms.push_back(trans);
            transforms_derivative.push_back(transformDerivativeFromDH(dha[i][0], dha[i][1], dha[i][2], angle));
        }
        auto R = ee_trans.topLeftCorner(3, 3);

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

    // array<double, 42> pandaJacobian(array<double, 7> q){
    //     array<Eigen::Matrix<double, 4, 4>, 8> transforms {
    //         transformFromDH(       0,   0.333,       0, q[0]),
    //         transformFromDH(       0,       0, -M_PI/2, q[1]),
    //         transformFromDH(       0, 0.31599,  M_PI/2, q[2]),
    //         transformFromDH( 0.08249,       0,  M_PI/2, q[3]),
    //         transformFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
    //         transformFromDH(       0,       0,  M_PI/2, q[5]),
    //         transformFromDH(  0.0879,       0,  M_PI/2, q[6]),
    //         transformFromDH(       0,  0.1069,       0,    0)
    //     };
    //     array<Eigen::Matrix<double, 4, 4>, 7> transforms_derivative {
    //         transformDerivativeFromDH(       0,   0.333,       0, q[0]),
    //         transformDerivativeFromDH(       0,       0, -M_PI/2, q[1]),
    //         transformDerivativeFromDH(       0, 0.31599,  M_PI/2, q[2]),
    //         transformDerivativeFromDH( 0.08249,       0,  M_PI/2, q[3]),
    //         transformDerivativeFromDH(-0.08249,   0.384, -M_PI/2, q[4]),
    //         transformDerivativeFromDH(       0,       0,  M_PI/2, q[5]),
    //         transformDerivativeFromDH(  0.0879,       0,  M_PI/2, q[6])
    //     };

    //     auto trans = EEtransform(q);
    //     auto R = trans.topLeftCorner(3, 3);

    //     Eigen::Matrix<double, 6, 7> jacobian = Eigen::MatrixXd::Zero(6, 7);
    //     for (int j = 0; j < 7; j++) {
    //         Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
    //         for (int i = 0; i < transforms.size(); i++) {
    //             if (i == j) {
    //                 jac = jac * transforms_derivative[i];
    //             } else {
    //                 jac = jac * transforms[i];
    //             }
    //         }
    //         jacobian(0, j) = jac(0, 3);
    //         jacobian(1, j) = jac(1, 3);
    //         jacobian(2, j) = jac(2, 3);

    //         auto W = jac.topLeftCorner(3,3) * R.transpose();
    //         jacobian(3, j) = W(2,1); // w_x
    //         jacobian(4, j) = W(0,2); // w_y
    //         jacobian(5, j) = W(1,0); // w_z
    //     }
        
    //     array<double, 42> jacobian_array{};
    //     // Pointer magic
    //     Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7) = jacobian;
    //     return jacobian_array;
    // }
}