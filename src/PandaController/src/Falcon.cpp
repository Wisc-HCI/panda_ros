#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "PandaController.h"
#include <franka/robot_state.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include <csignal>

using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;

FalconDevice m_falconDevice;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in Falcon.cpp\n";
    PandaController::stopControl();

    exit(sig);
}

bool init_falcon() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
    //m_falconDevice.setFalconGrip<FalconGripFourButton>();
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware

    if(!m_falconDevice.open(0)) //Open falcon @ index
    {
        cout << "Failed to find Falcon\n";
        return false;
    }
    else
    {
        cout << "Falcon Found\n";
    }

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    
    // MH: forcing the firmware to reload seems to solve the issue where
    // we had to keep re-plugging it in

    if(!firmware_loaded)
    {
        cout << "Loading firmware\n";

        uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
        long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

        for(int i = 0; i < 50; ++i)	//Attempt to load firmware 50 times
        {
            if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, firmware_size, firmware_block))
            {
                cout << "Firmware loading try failed\n";
            }
            else
            {
                firmware_loaded = true;
                break;
            }
        }
    }

    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue\n";
        return false;
    }
    cout << "Firmware loaded\n";

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set\n";

    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    usleep(100000);
    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) {
            continue;
        }
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in.\n";
            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
            cout << "Falcon homed.\n";
            stop = true;
        }
    }

    m_falconDevice.runIOLoop();
    return true;
}

void pollFalcon() {
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();
    array<double, 3> panda_pos = {0.0, 0.0, 0.0};
    panda_pos[0] = 0.26 + 0.26 * ((falconPos[2] - 0.074) / 0.099);
    panda_pos[1] = -0.12 + 0.47 * ((falconPos[0] + 0.047) / 0.067);
    panda_pos[2] = 0.29 + 0.3 * ((falconPos[1] + 0.05) / 0.098);
    PandaController::writeCommandedPosition(panda_pos);
    //TODO: write the pose instead of the position
}

void feedbackFalcon() {
    franka::RobotState state = PandaController::readRobotState();
    double scale = -0.1;
    m_falconDevice.setForce({
                state.O_F_ext_hat_K[1] * scale, 
                state.O_F_ext_hat_K[2] * scale, 
                state.O_F_ext_hat_K[0] * scale});
}

int main() {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    if (!init_falcon()) {
        cout << "Failed to init falcon" << endl;
        return -1;
    }
    pollFalcon();
    pid_t pid = PandaController::initPandaController(PandaController::ControlMode::CartesianPosition);
    if (pid < 0) {
        cout << "Failed to start panda process" << endl;
    }
    while (PandaController::isRunning()) {
        pollFalcon();
        feedbackFalcon();
    }
    m_falconDevice.close();
    return 0;
}
