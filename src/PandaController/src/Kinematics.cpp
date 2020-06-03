#include "Kinematics.h"
#include <array>
#include <cmath>
#include <Eigen/Core>
#include <adept.h>
#include <adept_arrays.h>

using namespace std;

namespace PandaController {
    
    Eigen::Matrix<double, 4, 4> transformFromDH(double a, double d, double alpha, double theta) {
        Eigen::MatrixXd trans(4,4);
        // Column major modified DH parameters
        trans << cos(theta), sin(theta) * cos(alpha), sin(theta) * sin(alpha), 0,
                -sin(theta), cos(theta) * cos(alpha), cos(theta) * sin(alpha), 0,
                          0,             -sin(alpha),              cos(alpha), 0,
                          a,         -d * sin(alpha),          d * cos(alpha), 1;          
        return trans;
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

    adept::aMatrix44 transformFromDHA(double a, double d, double alpha, adept::adouble theta) {
        adept::aMatrix44 trans;
        trans(0,0) = cos(theta);
        trans(1,0) = sin(theta) * cos(alpha);
        trans(2,0) = sin(theta) * sin(alpha);
        trans(3,0) = 0;

        trans(0,1) = -sin(theta);
        trans(1,1) = cos(theta) * cos(alpha);
        trans(2,1) = cos(theta) * sin(alpha);
        trans(3,1) = 0;
        
        trans(0,2) = 0;
        trans(1,2) = -sin(alpha);
        trans(2,2) = cos(alpha);
        trans(3,2) = 0;
        
        trans(0,3) = a;
        trans(1,3) = -d * sin(alpha);
        trans(2,3) = d * cos(alpha);
        trans(3,3) = 1;
        return trans;
    }

    adept::aMatrix44 EEtransform(array<adept::adouble, 7> q) {
        // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        return  transformFromDHA(       0,   0.333,       0, q[0]) **
                transformFromDHA(       0,       0, -M_PI/2, q[1]) **
                transformFromDHA(       0, 0.31599,  M_PI/2, q[2]) **
                transformFromDHA( 0.08249,       0,  M_PI/2, q[3]) **
                transformFromDHA(-0.08249,   0.384, -M_PI/2, q[4]) **
                transformFromDHA(       0,       0,  M_PI/2, q[5]) **
                transformFromDHA(  0.0879,       0,  M_PI/2, q[6]) **
                transformFromDHA(       0,  0.1069,       0,    0);
    }

    array<double, 42> pandaAutoJacobian(array<double, 7> q) {
        adept::Stack stack;
        array<adept::adouble, 7> q_ad;
        for (int i = 0; i < q.size(); i++) q_ad[i] = q[i];
        stack.new_recording();
        adept::aMatrix44 trans = EEtransform(q_ad);
        stack.independent(q_ad.data(), 7);

        stack.dependent(trans(0, 3)); // x_dot
        stack.dependent(trans(1, 3)); // y_dot
        stack.dependent(trans(2, 3)); // z_dot
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                stack.dependent(trans(i,j));
                R(i,j) = trans(i,j).value();
            }
        }
        Eigen::Matrix<double, 12, 7> jacobian_T;
        // This is not the jacobian WRT euler angles yet.
        // This is the jacobian WRT the transformation matrix
        stack.jacobian(jacobian_T.data());

        Eigen::Matrix<double, 6, 7> jacobian_X;
        jacobian_X.topRows(3) = jacobian_T.topRows(3);
        for (int i = 0; i < 7; i++) {
            Eigen::Matrix3d dRdq;
            for (int j = 0; j < 9; j++) {
                dRdq((int)(j / 3), j % 3) = jacobian_T(3+j,i);
            }
            auto W = dRdq * R.transpose();
            jacobian_X(3, i) = W(2,1); // w_x
            jacobian_X(4, i) = W(0,2); // w_y
            jacobian_X(5, i) = W(1,0); // w_z
        }
        array<double, 42> jacobian_array;
        Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data()) = jacobian_X;
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