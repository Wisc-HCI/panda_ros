#include "Kinematics.h"
#include <array>
#include <Eigen/Core>
#include <iostream>

using namespace std;


int main() {
    array<double, 7> q{0.9,0.1,-0.5,0.1,0.4,0.1,0};
    cout << Eigen::Map<Eigen::MatrixXd>(PandaController::calculatePandaJacobian(q).data(), 6, 7) << endl;
    cout << "New jacobian" << endl;
    cout << Eigen::Map<Eigen::MatrixXd>(PandaController::pandaAutoJacobian(q).data(), 6, 7) << endl;

    cout << "Diff" << endl;
    cout << Eigen::Map<Eigen::MatrixXd>(PandaController::calculatePandaJacobian(q).data(), 6, 7) - 
            Eigen::Map<Eigen::MatrixXd>(PandaController::pandaAutoJacobian(q).data(), 6, 7) << endl;

    return 0;
}