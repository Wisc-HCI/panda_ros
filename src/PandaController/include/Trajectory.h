#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <functional>
#include <vector>

namespace PandaController {
    enum TrajectoryType {Cartesian, Joint};
    class Trajectory {
    private:
        std::function<std::vector<double>()> trajectory_generator;
    public:
        TrajectoryType type;
        Trajectory(TrajectoryType t, std::function<std::vector<double>()> trajectory_generator);

        std::vector<double> operator()();
    };
}
#endif