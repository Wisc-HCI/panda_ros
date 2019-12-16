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
#include <franka/gripper.h>

namespace PandaController {

    enum ControlMode {CartesianVelocity, CartesianPosition, JointVelocity, JointPosition, None};

    void stopControl();
    pid_t initPandaController(ControlMode, char* = NULL);

    std::array<double, 6> readCommandedPosition();
    void writeCommandedPosition(std::array<double, 6> data);

    std::array<double, 6> readCommandedVelocity();
    void writeCommandedVelocity(std::array<double, 6> data);

    std::array<double, 7> readPoseGoal();
    void writePoseGoal(std::array<double, 7> data);

    std::array<double, 7> readJointAngles();
    void writeJointAngles(std::array<double, 7> data);

    franka::RobotState readRobotState();
    void writeRobotState(franka::RobotState data);
    void printRobotJointAngles(franka::RobotState data);
    void consumeBuffer(int &, franka::RobotState*, long *);

    void startLogging();
    bool isRunning();

    void writeGripperState();
    void homeGripper();
    
    void toggleGrip(std::function<void ()> onToggle = NULL);
    void graspObject(std::function<void ()> onGrasp = NULL);
    void releaseObject(std::function<void ()> onRelease = NULL);

    struct shared_data {
    public:

        std::array<double, 6> commanded_position;
        std::array<double, 6> commanded_velocity;
        franka::RobotState current_state;
        franka::RobotState buffer[1000];
        std::chrono::system_clock::time_point start_time;
        long timestamps[1000];
        int buffer_start = 0;
        int buffer_end = 0;

        std::array<double, 7> pose_goal{}; //<x, y, z, x, y, z, w>
        std::array<double, 7> joint_angles{};
        
        mutable boost::interprocess::interprocess_mutex mutex;

        long iteration;
        bool running = false;
        bool logging = false;
        std::array<double, 42> jacobian{};
        std::array<double, 42> jacobian_pinv{};
        std::array<double, 7> lastJointAcceleration{};

        bool isGripperMoving = false;
        bool grasped = false;
        

        shared_data() {
            writeCommandedVelocity({0,0,0,0,0,0});
        }
    }; //end struct shared_data

} //end namespace PandaController

