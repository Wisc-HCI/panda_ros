#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <eigen3/Eigen/Core>

namespace PandaController {

    pid_t initPandaController(char* = NULL);

    std::array<double, 3> readCommandedPosition();
    void writeCommandedPosition(std::array<double, 3> data);

    std::array<double, 7> readPoseGoal();
    void writePoseGoal(std::array<double, 7> data);

    std::array<double, 7> readJointAngles();
    void writeJointAngles(std::array<double, 7> data);

    franka::RobotState readRobotState();
    void writeRobotState(franka::RobotState data, franka::Model * robotModel = NULL);
    void consumeBuffer(int &, franka::RobotState*, long *);

    void startLogging();
    bool isRunning();

    struct shared_data {
    public:
        std::array<double, 3> commanded_position;
        franka::RobotState current_state;
        franka::RobotState buffer[1000];
        std::chrono::system_clock::time_point start_time;
        long timestamps[1000];
        int buffer_start = 0;
        int buffer_end = 0;

        std::array<double, 7> pose_goal; //<x, y, z, x, y, z, w>
        std::array<double, 7> joint_angles;
        //uint32_t sequence_num;
	    //bool quit;
        //bool reset_solver;
        
        mutable boost::interprocess::interprocess_mutex mutex;

        long iteration;
        bool running = false;
        bool logging = false;
        std::array<double, 42> jacobian;
        std::array<double, 42> jacobian_pinv;
        std::array<double, 7> lastJointAcceleration;
        

        shared_data() {
        }
    }; //end struct shared_data
} //end namespace PandaController

