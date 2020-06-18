#include "Kinematics.h"
#include <array>
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include <vector>
#include <stdlib.h>

using namespace std;


int main() {
    array<double, 7> q{0.9,0.1,-0.5,0.1,0.4,0.1,0};
    vector<Eigen::MatrixXd> list = vector<Eigen::MatrixXd>();
    //vector<std::array<double, 16>> list = vector<std::array<double, 16>>();

    // cout << Eigen::Map<Eigen::MatrixXd>(PandaController::pandaJacobian(q).data(), 6, 7) -
    //         Eigen::Map<Eigen::MatrixXd>(PandaController::calculatePandaJacobian(q).data(), 6, 7) << endl;
    // cout << "values" << endl;

    // cout << Eigen::Map<Eigen::MatrixXd>(PandaController::pandaJacobian(q).data(), 6, 7) <<endl;
    // cout << "old" << endl;
    // cout << Eigen::Map<Eigen::MatrixXd>(PandaController::calculatePandaJacobian(q).data(), 6, 7) << endl;

    for (int i = 0; i < 1; i ++) {
        for (int j = 0; j < 7; j++) {
            q[j] = rand() * (3.14/2) - 3.14;
        }
        auto start = chrono::system_clock::now();
        list.push_back(Eigen::Map<Eigen::MatrixXd>(PandaController::pandaJacobian(q).data(), 6, 7));
        //list.push_back(PandaController::calculatePandaEE(q));
        cout << chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now() - start).count() << endl;
    }



    return 0;
}