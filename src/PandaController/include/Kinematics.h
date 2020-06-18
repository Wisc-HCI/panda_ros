#ifndef PANDA_KINEMATICS_H
#define PANDA_KINEMATICS_H
#include <array>
#include <Eigen/Core>

namespace PandaController {
    std::array<double, 16> calculatePandaEE(std::array<double, 7> q);
    std::array<double, 42> calculatePandaJacobian(std::array<double, 7> q);
    std::array<double, 42> pandaJacobian(std::array<double, 7> q);
}
#endif