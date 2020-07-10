#include <vector>
#include <array>


namespace PandaController {
    namespace Kinematics {
        std::vector<std::array<double, 3>> PandaGripperDHA{
            {       0,   0.333,       0},
            {       0,       0, -M_PI/2},
            {       0, 0.31599,  M_PI/2},
            { 0.08249,       0,  M_PI/2},
            {-0.08249,   0.384, -M_PI/2},
            {       0,       0,  M_PI/2},
            {  0.0879,       0,  M_PI/2},
            {       0,  0.1069,       0}
        };
    }
}