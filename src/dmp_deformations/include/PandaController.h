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
#include <deque>

// Socket Libraries for FT sensor readings
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h> 

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

namespace PandaController {

    enum ControlMode {CartesianVelocity, CartesianPosition, JointVelocity, JointPosition, HybridControl, None};

    void stopControl();
    pid_t initPandaController(ControlMode, char* = NULL);

    std::array<double, 6> readCommandedPosition(double & scaleFactor);
    void writeCommandedPosition(std::array<double, 6> data);
    void writeCommandedPath(const std::array<double, 7>* data, const int & length);

    std::array<double, 6> readCommandedVelocity();
    void writeCommandedVelocity(std::array<double, 6> data);

    std::array<double, 6> readFTForces();
    void writeFTForces(std::array<double, 6> data);

    std::array<double, 7> readPoseGoal();
    void writePoseGoal(std::array<double, 7> data);

    std::array<double, 7> readJointAngles();
    void writeJointAngles(std::array<double, 7> data);

    std::array<double, 3> readSelectionVector();
    void writeSelectionVector(std::array<double, 3> data);

    std::array<double, 6> readCommandedFT();
    void writeCommandedFT(std::array<double, 6> data);

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

    /* FT Sensor - Typedefs used so integer sizes are more explicit */
    typedef unsigned int uint32;
    typedef int int32;
    typedef unsigned short uint16;
    typedef short int16;
    typedef unsigned char byte;
    typedef struct response_struct {
        uint32 rdt_sequence;
        uint32 ft_sequence;
        uint32 status;
        int32 FTData[6];
    } RESPONSE;
    // End FT Sensor Socket

    byte request[8]; /* The request data sent to the Net F/T. */
    int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */

    struct shared_data {
    public:

        std::array<double, 7> commanded_position[1000]; // time, pose
        int currentCommand = 0;
        int lastCommand = 0;
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

        std::array<double, 3> selection_vector{};
        std::array<double, 6> FT_command{};
        
        // Terms for storing filtered derivative
        std::array<double, 7> last_qe_dot{}; //<x, y, z, x, y, z, w>
        std::array<double, 7> last_qe{}; 


        bool isGripperMoving = false;
        bool grasped = false;

        std::array<double, 6> ft_sensor;
        

        shared_data() {
            writeCommandedVelocity({0,0,0,0,0,0});
            writeCommandedPosition({0,0,0,0,0,0});
        }
    }; //end struct shared_data

} //end namespace PandaController

