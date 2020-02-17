#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>

// Panda
#include "PandaController.h"
#include <franka/robot_state.h>

// Force Dimension
#include "dhdc.h"

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

/* Typedefs used so integer sizes are more explicit */
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

using namespace std;
//using namespace libnifalcon;
//using namespace StamperKinematicImpl;

//FalconDevice m_falconDevice;
byte request[8]; /* The request data sent to the Net F/T. */
int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
    dhdEnableForce (DHD_OFF);
    cout << "Closing Panda" << endl;
    PandaController::stopControl();
    exit(sig);
}

bool init_forcedimension() {
    cout <<"Setting up Force Dimension\n";
    if (dhdOpen () < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (2.0);
        return false;
    }
    cout << "Force Dimension Setup Complete" << endl;


    bool buttonPressed = false;

    cout << endl << endl << "(Press Force Dimension Button to Start Control)" << endl;

    // maybe add timeout?
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
}

void poll_forcedimension() {

    // Scaling Values
    array<double,3> offsets = {0.42, 0., 0.25};
    array<double,3> scaling_factors = {-2.0, -2.0, 2.0};

    array<double, 6> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    array<double, 3> force_dimension = {0.0, 0.0, 0.0};

    dhdGetPosition (&force_dimension[0], &force_dimension[1], &force_dimension[2]);

    panda_pos[0] = scaling_factors[0] * force_dimension[0] + offsets[0];
    panda_pos[1] = scaling_factors[1] * force_dimension[1] + offsets[1];
    panda_pos[2] = scaling_factors[2] * force_dimension[2] + offsets[2];

    //cout << "FD: " << panda_pos[0]  << "," << panda_pos[1] << "," << panda_pos[2] << endl;
    PandaController::writeCommandedPosition(panda_pos);

}

void feedback_forcedimension(){
    franka::RobotState state = PandaController::readRobotState();
    double scale = -0.0;
    //cout << "Forces: " << state.O_F_ext_hat_K[0]  << "," << state.O_F_ext_hat_K[1] << "," << state.O_F_ext_hat_K[2] << endl;
    dhdSetForceAndTorque(-state.O_F_ext_hat_K[0] * scale,-state.O_F_ext_hat_K[1] * scale,state.O_F_ext_hat_K[2] * scale,0.0,0.0,0.0);
}


void setup_ft(){
    
	double cpt = 1000000;
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	int err;					/* Error status of operations. */

	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
        cout << "Can't Get Socket Handle. Exiting." << endl;
		exit(1);
	}
	
	*(uint16*)&request[0] = htons(0x1234); /* standard header. */
	*(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	he = gethostbyname("192.168.2.2");
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		cout << "Can't Connect to Socket. Exiting." << endl;
        exit(2);
	}
	
}

void feedback_ft_forcedimension(){
    
    // Get the current FT reading from the sensor
    double cpf = 1000000;
    int i;						/* Generic loop/array index. */
    RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
    
    // Get feedback from FT sensor
    send(socketHandle, request, 8, 0 );

	/* Receiving the response. */
	recv( socketHandle, response, 36, 0 );
	resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
	resp.ft_sequence = ntohl(*(uint32*)&response[4]);
	resp.status = ntohl(*(uint32*)&response[8]);
	for( i = 0; i < 6; i++ ) {
		resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
	}

	/* Output the response data. */
	//printf( "Status: 0x%08x\n", resp.status );
	printf("%s: %f\n", "Fx:", (double)resp.FTData[0]/cpf);
    printf("%s: %f\n", "Fx:", (double)resp.FTData[1]/cpf);
    printf("%s: %f\n", "Fx:", (double)resp.FTData[2]/cpf);

    // Transform into the correct frame based on Panda Pose
    franka::RobotState state = PandaController::readRobotState();
    cout << state.O_T_EE[0] << endl;

    // Subtract the tool weight in the global z direction

    // Send to Force Dimension
    double scale = -1.0;
    dhdSetForceAndTorque(-(double)resp.FTData[0]/cpf * scale,-(double)resp.FTData[1]/cpf * scale,(double)resp.FTData[2]/cpf * scale,0.0,0.0,0.0);

}

int main() {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    setup_ft();

    if (!init_forcedimension()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }


    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button again to enable Forces and Panda Control" << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    pid_t pid = PandaController::initPandaController(PandaController::ControlMode::CartesianPosition);
    if (pid < 0) {
       cout << "Failed to start panda process" << endl;
    }
    poll_forcedimension();

    bool exit=false;

    while (PandaController::isRunning() && (exit == false)) {
    //while (1) {
        poll_forcedimension();
    //    feedback_forcedimension();
        feedback_ft_forcedimension();

        if(dhdGetButton(0)==1) // button pressed
        {
            exit = true;
        }
    }

    dhdEnableForce (DHD_OFF);
    cout << "Closing Panda" << endl;
    PandaController::stopControl();
    
    return 0;
}
