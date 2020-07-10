#include <functional>
#include <vector>
#include "Trajectory.h"

namespace PandaController {
    Trajectory::Trajectory(TrajectoryType t, std::function<std::vector<double>()> trajectory_generator) {
        this->type = t;
        this->trajectory_generator = trajectory_generator;
    }
    std::vector<double> Trajectory::operator()() {
        return this->trajectory_generator();
    }
}