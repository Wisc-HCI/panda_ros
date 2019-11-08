#include "PandaController.h"
#include <franka/robot_state.h>

extern "C" void start_logging(){
    PandaController::startLogging();
}

extern "C" bool isRunning(){
    return PandaController::isRunning();
}

extern "C" void get_state_history(long* times, double** cartesianPositions, double** jointPositions, double** forces){
    franka::RobotState buffer[1000];
    int count;
    PandaController::consumeBuffer(count, buffer, times);

    for (int i = 0; i < count; i++){
        franka::RobotState state = buffer[i];
        for(int j = 0; j < 3; j++)
            cartesianPositions[i][j] = state.O_T_EE[12 + j];
        for(int j = 0; j < 7; j++)
            jointPositions[i][j] = state.q[j];
        for(int j = 0; j < 3; j++)
            forces[i][j] = state.O_F_ext_hat_K[j];
    }
}
