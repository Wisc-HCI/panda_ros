#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>
#include "PandaController.h"
#include "Common.h"
#include <eigen3/Eigen/Dense>


using namespace boost::interprocess;
using namespace std;

namespace PandaController {
    namespace {
        shared_data* SharedData = NULL;
        mapped_region* memoryRegion;
        franka::Gripper *p_gripper;
    }

    void stopControl() {
        cout << "Control stopping" << endl;
        SharedData->running = false;
    }

    array<double,42> calculatePandaJacobian(array<double, 7> q) {
        // Calculated using Modified DH-Parameters analytically
        // using joint anlges, q:
        array<double, 42> jacobian_array;

        // Store the results in a column-wise matrix that is put in the
        // shared memory object

        // Column 0
        //(0,0)
        jacobian_array[0]=0.384*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.31599*std::sin(q[0])*
        std::sin(q[1]) - 0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.08249*std::cos(q[0])*
        std::sin(q[2]) + 0.08249*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*
        std::cos(q[2])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) + 0.08249*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) - 0.0879*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) +
         0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*std::cos(q[0])*
         std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
         std::sin(q[5]) - 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) +
         0.0879*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
         std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*
         std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*
         std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) -
         0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
         std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[4])*
         std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[1])*std::cos(q[2])*
         std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(1,0)
        jacobian_array[1]=0.31599*std::cos(q[0])*std::sin(q[1]) - 0.08249*std::sin(q[0])*std::sin(q[2]) - 0.08249*
        std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) + 0.384*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2]) + 0.384*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) -
        0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.0879*std::cos(q[0])*
        std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) +
        0.1069*std::cos(q[0])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[5]);

        //(2,0)
        jacobian_array[2]=0.0;

        //(3,0)
        jacobian_array[3]=0.0;

        //(4,0)
        jacobian_array[4]=0.0;

        //(5,0)
        jacobian_array[5]=1.0;

        // Column 1
        //(0,1)
        jacobian_array[6]=0.31599*std::cos(q[0])*std::cos(q[1]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::cos(q[3]) -
        0.08249*std::cos(q[0])*std::cos(q[2])*std::sin(q[1]) - 0.08249*std::cos(q[0])*std::cos(q[1])*std::sin(q[3]) -
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.08249*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[1]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.384*
        std::cos(q[0])*std::cos(q[2])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) +
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*
        std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) - 0.1069*
        std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);

        //(1,1)
        jacobian_array[7]=0.31599*std::cos(q[1])*std::sin(q[0]) - 0.08249*std::cos(q[2])*std::sin(q[0])*std::sin(q[1]) -
        0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[3]) + 0.384*std::cos(q[1])*std::cos(q[3])*std::sin(q[0]) -
        0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) + 0.08249*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[0])*std::sin(q[1]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.384*
        std::cos(q[2])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) +
        0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
        std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.1069*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]);

        //(2,1)
        jacobian_array[8]=0.08249*std::sin(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[1])*std::cos(q[2]) - 0.384*std::cos(q[3])*
        std::sin(q[1]) - 0.31599*std::sin(q[1]) - 0.0879*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.08249*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3]) + 0.384*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[5])*
        std::sin(q[1]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[2])*
        std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(3,1)
        jacobian_array[9] = -1.0*std::sin(q[0]);

        //(4,1)
        jacobian_array[10]=std::cos(q[0]);

        //(5,1)
        jacobian_array[11]=0.0;

        // Column 2
        
        //(0,2)
        jacobian_array[12]=0.384*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]) - 0.08249*std::cos(q[2])*std::sin(q[0]) - 0.08249*std::cos(q[0])*
        std::cos(q[1])*std::sin(q[2]) + 0.08249*std::cos(q[2])*std::cos(q[3])*std::sin(q[0]) + 0.08249*std::cos(q[0])*std::cos(q[1])*
        std::cos(q[3])*std::sin(q[2]) + 0.384*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) + 0.0879*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) -
        0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
        std::cos(q[5])*std::sin(q[0]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.1069*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*
        std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[2])*std::sin(q[5]);

        //(1,2)
        jacobian_array[13]=0.08249*std::cos(q[0])*std::cos(q[2]) - 0.08249*std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 0.08249*std::cos(q[0])*
        std::cos(q[2])*std::cos(q[3]) - 0.384*std::cos(q[0])*std::cos(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*
        std::sin(q[3]) + 0.08249*std::cos(q[1])*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[3])*
        std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.384*std::cos(q[1])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*
        std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[1])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2]) - 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]);

        //(2,2)
        jacobian_array[14]=0.08249*std::sin(q[1])*std::sin(q[2]) - 0.08249*std::cos(q[3])*std::sin(q[1])*std::sin(q[2]) - 0.384*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::sin(q[1])*std::sin(q[2])*
        std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) + 0.1069*std::cos(q[3])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]);

        //(3,2)
        jacobian_array[15]=std::cos(q[0])*std::sin(q[1]);

        //(4,2)
        jacobian_array[16]=std::sin(q[0])*std::sin(q[1]);

        //(5,2)
        jacobian_array[17]=std::cos(q[1]);


        // Column 3

        //(0,3)
        jacobian_array[18]=0.384*std::cos(q[3])*std::sin(q[0])*std::sin(q[2]) - 0.384*std::cos(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.08249*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[0])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[3]) + 0.08249*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]) + 0.1069*
        std::cos(q[0])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) -
        0.0879*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) +
        0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[0])*
        std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[3]) + 0.1069*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*
        std::sin(q[3])*std::sin(q[5]);

        //(1,3)
        jacobian_array[19]=0.08249*std::cos(q[0])*std::sin(q[2])*std::sin(q[3]) - 0.08249*std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 0.384*
        std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) - 0.384*std::cos(q[0])*std::cos(q[3])*std::sin(q[2]) - 0.384*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[0]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[2]) + 0.08249*std::cos(q[1])*std::cos(q[2])*
        std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[5]) + 0.1069*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1])*std::sin(q[3]) - 0.0879*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[5]) + 0.0879*
        std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[2])*std::sin(q[3]) + 0.1069*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[4])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]);

        //(2,3)
        jacobian_array[20]=0.384*std::cos(q[2])*std::cos(q[3])*std::sin(q[1]) - 0.384*std::cos(q[1])*std::sin(q[3]) - 0.08249*std::cos(q[2])*
        std::sin(q[1])*std::sin(q[3]) - 0.0879*std::cos(q[1])*std::sin(q[3])*std::sin(q[5]) - 0.08249*std::cos(q[1])*std::cos(q[3]) + 0.1069*
        std::cos(q[1])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.1069*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]) + 0.0879*std::cos(q[2])*
        std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) + 0.1069*
        std::cos(q[2])*std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]);

        //(3,3)
        jacobian_array[21]=std::cos(q[2])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::sin(q[2]);

        //(4,3)
        jacobian_array[22]=std::cos(q[1])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2]);

        //(5,3)
        jacobian_array[23]=-1.0*std::sin(q[1])*std::sin(q[2]);

        // Column 4

        //(0,4)
        jacobian_array[24]=0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[2])*
        std::cos(q[4])*std::sin(q[0])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[2]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 0.0879*std::cos(q[2])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0]) - 0.1069*std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) + 0.1069*
        std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[3])*std::cos(q[5])*std::sin(q[4]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4])*std::sin(q[5]);

        //(1,4)
        jacobian_array[25]=0.0879*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*std::cos(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[4])*
        std::sin(q[5]) - 0.0879*std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[2]) - 0.0879*std::cos(q[0])*std::cos(q[3])*
        std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*
        std::sin(q[3])*std::sin(q[4]) - 0.1069*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*std::sin(q[4]) - 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[0])*std::sin(q[4])*std::sin(q[5]);

        //(2,4)
        jacobian_array[26]=0.0879*std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[5])*std::sin(q[3])*
        std::sin(q[4]) + 0.1069*std::cos(q[4])*std::sin(q[1])*std::sin(q[2])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::sin(q[3])*std::sin(q[4])*
        std::sin(q[5]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1])*std::sin(q[4]) + 0.1069*std::cos(q[2])*std::cos(q[3])*
        std::sin(q[1])*std::sin(q[4])*std::sin(q[5]);

        //(3,4)
        jacobian_array[27]=std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*
        std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3]);

        //(4,4)
        jacobian_array[28]=std::cos(q[3])*std::sin(q[0])*std::sin(q[1]) - 1.0*std::cos(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3]);

        //(5,4)
        jacobian_array[29]=std::cos(q[1])*std::cos(q[3]) + std::cos(q[2])*std::sin(q[1])*std::sin(q[3]);

        // Column 5

        //(0,5)
        jacobian_array[30]=0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[5])*std::sin(q[1]) + 0.1069*
        std::cos(q[0])*std::cos(q[3])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[2])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[4]) + 0.0879*std::cos(q[5])*std::sin(q[0])*std::sin(q[2])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::sin(q[0])*
        std::sin(q[4])*std::sin(q[5]) + 0.1069*std::sin(q[0])*std::sin(q[2])*std::sin(q[3])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::sin(q[3])*
        std::sin(q[5]) - 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[5])*std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[0])*
        std::cos(q[4])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.1069*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[2]) + 0.0879*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.0879*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5]) - 0.0879*
        std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(1,5)
        jacobian_array[31]=0.1069*std::cos(q[0])*std::cos(q[2])*std::cos(q[5])*std::sin(q[4]) + 0.0879*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1]) - 0.0879*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*
        std::sin(q[5]) + 0.1069*std::cos(q[3])*std::sin(q[0])*std::sin(q[1])*std::sin(q[5]) - 0.1069*std::cos(q[0])*std::sin(q[2])*std::sin(q[3])*
        std::sin(q[5]) + 0.1069*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[2]) - 0.0879*std::cos(q[1])*std::cos(q[2])*
        std::cos(q[5])*std::sin(q[0])*std::sin(q[3]) - 0.0879*std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 0.1069*
        std::cos(q[1])*std::cos(q[2])*std::sin(q[0])*std::sin(q[3])*std::sin(q[5]) - 0.1069*std::cos(q[1])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[4]) + 0.1069*std::cos(q[4])*std::cos(q[5])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3]) + 0.0879*std::cos(q[1])*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.0879*std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[0]) - 0.0879*std::cos(q[1])*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(2,5)
        jacobian_array[32]=0.0879*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 0.1069*std::cos(q[1])*std::cos(q[3])*std::sin(q[5]) + 0.1069*
        std::cos(q[1])*std::cos(q[4])*std::cos(q[5])*std::sin(q[3]) + 0.0879*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*std::sin(q[3]) - 0.0879*
        std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) + 0.1069*std::cos(q[2])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) + 0.1069*
        std::cos(q[5])*std::sin(q[1])*std::sin(q[2])*std::sin(q[4]) - 0.0879*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) - 0.1069*
        std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::cos(q[5])*std::sin(q[1]) + 0.0879*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*
        std::sin(q[1])*std::sin(q[5]);

        //(3,5)
        jacobian_array[33]=std::cos(q[2])*std::cos(q[4])*std::sin(q[0]) + std::cos(q[0])*std::cos(q[1])*std::cos(q[4])*std::sin(q[2]) + 1.0*
        std::cos(q[0])*std::sin(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[3])*std::sin(q[0])*std::sin(q[2])*std::sin(q[4]) + 1.0*
        std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[4]);

        //(4,5)
        jacobian_array[34]=std::cos(q[1])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2]) - 1.0*std::cos(q[0])*std::cos(q[2])*
        std::cos(q[4]) + std::cos(q[0])*std::cos(q[3])*std::sin(q[2])*std::sin(q[4]) + std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[4]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::sin(q[0])*std::sin(q[4]);

        //(5,5)
        jacobian_array[35]=std::cos(q[1])*std::sin(q[3])*std::sin(q[4]) - 1.0*std::cos(q[4])*
        std::sin(q[1])*std::sin(q[2]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::sin(q[1])*std::sin(q[4]);

        // Column 6

        //(0,6)
        jacobian_array[36]=0;

        //(1,6)
        jacobian_array[37]=0;

        //(2,6)
        jacobian_array[38]=0;

        //(3,6)
        jacobian_array[39]=1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*std::sin(q[3]) - 1.0*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[2])*std::sin(q[0])*std::sin(q[4])*std::sin(q[5]) - 1.0*std::cos(q[0])*std::cos(q[3])*
        std::cos(q[5])*std::sin(q[1]) - 1.0*std::cos(q[0])*std::cos(q[1])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[0])*
        std::cos(q[4])*std::sin(q[1])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[2])*
        std::sin(q[5]) + 1.0*std::cos(q[0])*std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[5]);

        //(4,6)
        jacobian_array[40]=1.0*std::cos(q[0])*std::cos(q[5])*std::sin(q[2])*std::sin(q[3]) - 1.0*std::cos(q[3])*std::cos(q[5])*std::sin(q[0])*
        std::sin(q[1]) + 1.0*std::cos(q[0])*std::cos(q[2])*std::sin(q[4])*std::sin(q[5]) + 1.0*std::cos(q[1])*std::cos(q[2])*std::cos(q[5])*
        std::sin(q[0])*std::sin(q[3]) + std::cos(q[0])*std::cos(q[3])*std::cos(q[4])*std::sin(q[2])*std::sin(q[5]) - 1.0*std::cos(q[1])*
        std::sin(q[0])*std::sin(q[2])*std::sin(q[4])*std::sin(q[5]) + std::cos(q[4])*std::sin(q[0])*std::sin(q[1])*std::sin(q[3])*
        std::sin(q[5]) + std::cos(q[1])*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[0])*std::sin(q[5]);

        //(5,6)
        jacobian_array[41]=std::cos(q[1])*std::cos(q[4])*std::sin(q[3])*std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[5])*std::sin(q[1])*
        std::sin(q[3]) - 1.0*std::cos(q[1])*std::cos(q[3])*std::cos(q[5]) + 1.0*std::sin(q[1])*std::sin(q[2])*std::sin(q[4])*
        std::sin(q[5]) - 1.0*std::cos(q[2])*std::cos(q[3])*std::cos(q[4])*std::sin(q[1])*std::sin(q[5]);

        SharedData->jacobian = jacobian_array;
        return jacobian_array;
    }

    double absMax(Eigen::VectorXd v) {
        double max = 0;
        for(int i = 0; i < v.size(); i++){
            if (std::abs(v[i]) > max){
                max = std::abs(v[i]);
            }
        }
        return max;
    }

    double signAbsMax(Eigen::VectorXd v) {
        double max = 0;
        double act_max = 0;
        for(int i = 0; i < v.size(); i++){
            if (std::abs(v[i]) > max){
                max = std::abs(v[i]);
                act_max = v[i];
            }
        }

        if (act_max < 0){
            return -1;
        }
        else{
            return 1;
        }
    }

    int absMaxIndex(Eigen::VectorXd v) {
        double max = 0;
        int max_index = 0;
        for(int i = 0; i < v.size(); i++){
            if (std::abs(v[i]) > max){
                max = std::abs(v[i]);
                max_index = i;
            }
        }
        return max_index;
    }

    double constrainEEJerk(Eigen::VectorXd & desiredJerk, const franka::RobotState & robot_state){
        double max_j = 6500.0;
        double length = desiredJerk.norm();
        double j = absMax(desiredJerk);
        if (j > max_j){
            desiredJerk *= max_j / j;
        }
        if (length == 0) return 1;
        return desiredJerk.norm() / length;
    }


    double constrainEEAcceleration(Eigen::VectorXd & desiredAcceleration, const franka::RobotState & robot_state){
        double max_a = 13.0;
        double length = desiredAcceleration.norm();

        Eigen::Map<const Eigen::VectorXd> lastAcceleration(robot_state.O_ddP_EE_c.data(), 6);
        Eigen::VectorXd desiredJerk = (desiredAcceleration - lastAcceleration) * 1000;
        constrainEEJerk(desiredJerk, robot_state);

        desiredAcceleration = lastAcceleration + desiredJerk / 1000;
        double a = absMax(desiredAcceleration);
        if (a > max_a){
            desiredAcceleration *= max_a / a;
        }
        if (length == 0) return 1;
        return desiredAcceleration.norm() / length;
    }

    double constrainEEVelocity(Eigen::VectorXd & desiredVelocity, const franka::RobotState & robot_state) {
        double max_v = 1.7;
        double length = desiredVelocity.norm();
        Eigen::Map<const Eigen::VectorXd> lastVelocity(robot_state.O_dP_EE_c.data(), 6); 
        Eigen::VectorXd desiredAcceleration = (desiredVelocity - lastVelocity) * 1000;
        constrainEEAcceleration(desiredAcceleration, robot_state);

        desiredVelocity = lastVelocity + desiredAcceleration / 1000;
        double v = absMax(desiredVelocity);
        if (v > max_v){
            desiredVelocity *= max_v / v; 
        }
        if (length == 0) return 1;
        return desiredVelocity.norm() / length;
    }

    double constrainEEPosition(Eigen::VectorXd & desiredPosition, const franka::RobotState & robot_state) {
        Eigen::Map<const Eigen::VectorXd> lastPosition(robot_state.O_T_EE.data(), 6);
        Eigen::VectorXd desiredVelocity = (desiredPosition - lastPosition) * 1000;
        constrainEEVelocity(desiredVelocity, robot_state);
        desiredPosition = lastPosition + desiredVelocity / 1000;
        return 1;
    }

    double constrainJointJerk(Eigen::VectorXd & desiredJointJerk, const franka::RobotState & robot_state) {
        double softThreshold = 0.95;
        array<double, 7> max_dddq = {7500*softThreshold, 3750*softThreshold, 5000*softThreshold, 6250*softThreshold, 7500*softThreshold, 10000*softThreshold, 10000*softThreshold};
        double length = desiredJointJerk.norm();

        double dddq;
        for (int i = 0; i < 7; i++){
            dddq = abs(desiredJointJerk[i]);
            if (dddq > max_dddq[i]){
                desiredJointJerk *= max_dddq[i] / dddq;
            }
        }
        
        if (length == 0) return 1;
        return desiredJointJerk.norm() / length;
    }

    double constrainJointAcceleration(Eigen::VectorXd & desiredJointAcceleration, const franka::RobotState & robot_state) {
        double softThreshold = 0.95;
        array<double, 7> max_ddq = {15*softThreshold, 7.5*softThreshold, 10*softThreshold, 12.5*softThreshold, 15*softThreshold, 20*softThreshold, 20*softThreshold};
        double length = desiredJointAcceleration.norm();
        Eigen::Map<const Eigen::VectorXd> lastJointAcceleration(SharedData->lastJointAcceleration.data(), 7);
        Eigen::VectorXd desiredJointJerk = (desiredJointAcceleration - lastJointAcceleration) * 1000;
        
        constrainJointJerk(desiredJointJerk, robot_state);
        
        desiredJointAcceleration = lastJointAcceleration + desiredJointJerk / 1000;
        
        double ddq;
        for (int i = 0; i < 7; i++) {
            ddq = abs(desiredJointAcceleration[i]);
            if (ddq > max_ddq[i]){
                desiredJointAcceleration *= max_ddq[i] / ddq;
            }
        }
        if (length == 0) return 1;
        return desiredJointAcceleration.norm() / length;
    }

    double constrainJointVelocity(Eigen::VectorXd & desiredJointVelocity, const franka::RobotState & robot_state) {
        double softThreshold = 0.95;
        array<double, 7> max_dq = {2.1750*softThreshold, 2.1750*softThreshold, 2.1750*softThreshold, 2.1750*softThreshold, 2.6100*softThreshold, 2.6100*softThreshold, 2.6100*softThreshold};
        double length = desiredJointVelocity.norm();
        Eigen::Map<const Eigen::VectorXd> lastJointVelocity(robot_state.dq_d.data(), 7);
        
        Eigen::VectorXd desiredJointAcceleration = (desiredJointVelocity - lastJointVelocity) * 1000;
        constrainJointAcceleration(desiredJointAcceleration, robot_state);

        desiredJointVelocity = lastJointVelocity + desiredJointAcceleration / 1000;

        double dq;
        for (int i = 0; i < 7; i++){
            dq = abs(desiredJointVelocity[i]);
            if (dq > max_dq[i]){
                desiredJointVelocity *= max_dq[i] / dq;
            }
        }
        if (length == 0) return 1;
        return desiredJointVelocity.norm() / length;
    }

    void constrainJointPosition(Eigen::VectorXd & desiredJointPosition, const franka::RobotState & robot_state) {
        Eigen::Map<const Eigen::VectorXd> lastJointPosition(robot_state.q_d.data(), 7);
        Eigen::VectorXd desiredJointVelocity = (desiredJointPosition - lastJointPosition) * 1000;
        constrainJointVelocity(desiredJointVelocity, robot_state);
        desiredJointPosition = lastJointPosition + desiredJointVelocity / 1000;
    }

    void printOutJointThings(Eigen::VectorXd dq, const franka::RobotState & robot_state){
        cout << "Velocity: " << dq.transpose() << endl;
        Eigen::VectorXd desiredAcceleration = (dq - Eigen::Map<const Eigen::VectorXd>(robot_state.dq.data(),7)) * 1000;
        cout << "Acceleration: " << (desiredAcceleration).transpose() << endl;
        Eigen::VectorXd desiredJerk = (desiredAcceleration - Eigen::Map<Eigen::VectorXd>(SharedData->lastJointAcceleration.data(),7)) * 1000;
        cout << "Jerk: " << (desiredJerk).transpose() << endl;
    }

    void roundTo(Eigen::VectorXd & a, int precision){
        for(int i = 0; i < a.size(); i++){
            a[i] = trunc(a[i] * precision) / precision;
        }
    }


    void runVelocityController(char* ip = NULL){
        if (ip == NULL) {
            ip = "10.134.71.22";
        }
        try {
            cout << "Before connect" << endl;
            franka::Robot robot(ip);
            robot.automaticErrorRecovery();
            MotionGenerator motion_generator(0.5, {{0.626267,-0.757022,8.87913e-06,-2.46825,-3.16042e-06,1.71123,1.41167}});
            cout << "Starting homing" << endl;
            robot.control(motion_generator);
            cout << "Motion complete" << endl;
            setDefaultBehavior(robot);
            cout << "Default Behavior Set" << endl;

            double time_max = 30.0;
            double time = 0.0;
            double v_max = 0.1;
            int count = 0;
            cout << "Model loaded" << endl;

            writeRobotState(robot.readOnce());
            //startInvJacobianComputer();
            cout << "About to start" << endl;

            robot.control([=, &time, &count](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::CartesianVelocities {
                time += period.toSec();
                writeRobotState(robot_state);

                array<double, 3> commandedPosition = readCommandedPosition();
                
                double scaling_factor = 0.2;
                double v_x = (commandedPosition[0] - robot_state.O_T_EE[12]) / scaling_factor;
                double v_y = (commandedPosition[1] - robot_state.O_T_EE[13]) / scaling_factor;
                double v_z = (commandedPosition[2] - robot_state.O_T_EE[14]) / scaling_factor;

                int ramp = 1400;
                if (count < ramp)
                {
                    v_x = v_x * (1 - 2*exp(-0.005 * count) + exp(-0.01 * count));
                    v_y = v_y * (1 - 2*exp(-0.005 * count) + exp(-0.01 * count));
                    v_z = v_z * (1 - 2*exp(-0.005 * count) + exp(-0.01 * count));
                }

                Eigen::VectorXd v(6);
                v << v_x, v_y, v_z, 0, 0, 0;
                Eigen::VectorXd jointVelocities = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7).completeOrthogonalDecomposition().solve(v);
                constrainJointVelocity(jointVelocities,robot_state);
                v = Eigen::Map<Eigen::MatrixXd>(SharedData->jacobian.data(), 6, 7) * jointVelocities;
                franka::CartesianVelocities output = {{v[0], v[1], v[2], 0.0, 0.0, 0.0}};
                Eigen::VectorXd lastJointAcceleration = (jointVelocities - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                
                if (time >= time_max) {
                    cout << endl << "Finished motion, shutting down example" << endl;
                    return franka::MotionFinished(output);
                }
                count =  count + 1;
                return output;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
        stopControl();
    }

    void runJointPositionController(char* ip = NULL){
        if (ip == NULL) {
            ip = "10.134.71.22";
        }
        try {
            franka::Robot robot(ip);
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // std::array<double,7> q_goal = {{0.626267,-0.757022,8.87913e-06,-2.46825,-3.16042e-06,1.71123,1.41167}};
            MotionGenerator motion_generator(0.5, q_goal);
            robot.control(motion_generator);
            setDefaultBehavior(robot);

            std::array<double, 7> initial_position;
            std::array<double, 7> curr_position;
            double time = 0.0;
            double time_max = 5.0;
            //int count = 0;
            //franka::Model robotModel = robot.loadModel();
            robot.control([=, &initial_position, &curr_position, &time](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::JointPositions {
                time += period.toSec();
                writeRobotState(robot_state);

                if (time == 0.0) {
                    initial_position = robot_state.q_d;
                    curr_position = initial_position;
                }

                array<double, 7> joint_angles = readJointAngles();  
                array<double, 7> delta_angles;
                for (int i = 0; i < 7; i++) {
                    delta_angles[i] = joint_angles[i] - curr_position[i];
                }

                //TODO: some sort of bounds checking on the delta angles
                //TODO: relaxed IK takes care of smoothing right?

                for (int i = 0; i < 7; i++) {
                    curr_position[i] = delta_angles[i] + curr_position[i];
                }

                //TODO: actually use the joint positions here
                // franka::JointPositions output = {{curr_position[0], curr_position[1],
                //                                 curr_position[2], curr_position[3],
                //                                 curr_position[4], curr_position[5],
                //                                 curr_position[6]}};

                double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
                franka::JointPositions output = {{initial_position[0], initial_position[1],
                                                initial_position[2], initial_position[3] + delta_angle,
                                                initial_position[4] + delta_angle, initial_position[5],
                                                initial_position[6] + delta_angle}};

                if (time >= time_max) {
                    std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                    return franka::MotionFinished(output);
                }
                //count =  count + 1;
                return output;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            //cout << franka::logToCSV(e.log) << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
        stopControl();
    }

    void runJointVelocityController(char* ip = NULL){
        if (ip == NULL) {
            ip = "10.134.71.22";
        }
        try {
            franka::Robot robot(ip);
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // std::array<double,7> q_goal = {{0.626267,-0.757022,8.87913e-06,-2.46825,-3.16042e-06,1.71123,1.41167}};
            MotionGenerator motion_generator(0.5, q_goal);
            robot.control(motion_generator);
            setDefaultBehavior(robot);

            double time_max = 50.0;
            double omega_max = 1.0;
            double time = 0.0;
            double delta_t = 0.001;
            std::array<double, 7> joint_pos;
            std::array<double, 7> joint_vel;
            std::array<double, 7> joint_acc;
            cout << "In vel controller" <<endl;
            robot.control([=, &time, &joint_pos, &joint_vel, &joint_acc](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                time += period.toSec();
                array<double, 7> joint_angles = readJointAngles();

                double scale = 5;
                Eigen::VectorXd joint_velocity(7);
                joint_velocity << 
                    (joint_angles[0] - robot_state.q[0]) * scale,
                    (joint_angles[1] - robot_state.q[1]) * scale,
                    (joint_angles[2] - robot_state.q[2]) * scale,
                    (joint_angles[3] - robot_state.q[3]) * scale,
                    (joint_angles[4] - robot_state.q[4]) * scale,
                    (joint_angles[5] - robot_state.q[5]) * scale,
                    (joint_angles[6] - robot_state.q[6]) * scale;

                //cout<<constrainJointVelocity(joint_velocity, robot_state)<<endl;
                constrainJointVelocity(joint_velocity, robot_state);
                Eigen::VectorXd lastJointAcceleration = (joint_velocity - Eigen::Map<const Eigen::VectorXd>(robot_state.dq_d.data(), 7)) * 1000;
                for(int i = 0; i < 7; i++) {
                    SharedData->lastJointAcceleration[i] = lastJointAcceleration[i];
                }
                franka::JointVelocities velocities = {
                    joint_velocity[0],
                    joint_velocity[1],
                    joint_velocity[2],
                    joint_velocity[3],
                    joint_velocity[4],
                    joint_velocity[5],
                    joint_velocity[6]};

                if (time >= time_max) {
                    std::cout << std::endl << "TIME OUT!!" << std::endl;
                    return franka::MotionFinished(velocities);
                }
                return velocities;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            //cout << franka::logToCSV(e.log) << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
        // std::cout << "\n\n\n\n\n\n\n\n Outside of panda control loop\n\n\n\n\n\n\n\n\n";
        stopControl();
    }

    void noController(char* ip = NULL){
        if (ip == NULL) {
            ip = "10.134.71.22";
        }
        try {
            franka::Robot robot(ip);
            robot.automaticErrorRecovery();
            std::array<double,7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // std::array<double,7> q_goal = {{0.626267,-0.757022,8.87913e-06,-2.46825,-3.16042e-06,1.71123,1.41167}};
            MotionGenerator motion_generator(0.5, q_goal);
            robot.control(motion_generator);
            setDefaultBehavior(robot);

            double time_max = 30.0;
            double time = 0.0;
            double delta_t = 0.001;
            robot.control([=, &time](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                time += period.toSec();
                writeRobotState(robot_state);

                franka::JointVelocities velocities = {0,0,0,0,0,0,0};

                if (time >= time_max) {
                 std::cout << std::endl << "TIME OUT!!" << std::endl;
                    return franka::MotionFinished(velocities);
                }
                return velocities;
            });
        } catch (const franka::ControlException& e) {
            cout << e.what() << endl;
            //cout << franka::logToCSV(e.log) << endl;
            stopControl();
        } catch (const franka::Exception& e){  
            cout << e.what() << endl;
            stopControl();
        } catch(const exception& e) {
            cout << e.what() << endl;
            stopControl();
        }
        // std::cout << "\n\n\n\n\n\n\n\n Outside of panda control loop\n\n\n\n\n\n\n\n\n";
        stopControl();
    }
    void writeGripperState() {
        franka::GripperState state = p_gripper->readOnce();
        std::cout << "Printing the gripper state:\n" << "Current width = " << state.width <<
                     "m\nMax width = " << state.max_width << "m\nIs grasped = " << state.is_grasped << "\n";        
    }

    bool homeGripper() {
        try {
            return p_gripper->homing();
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
    }

    bool graspObject() {
        cout<<"In grasp"<<endl;
        try {
            return p_gripper->grasp(0.03994995,0.05,10,0.5,0.5);
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
    }

    bool releaseObject() {
        try {
            p_gripper->stop();
            return p_gripper->move(0.0798999,0.05);            
        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }
    }

    void gripperTest() {
        // homeGripper();
        writeGripperState();
        if(!graspObject()) {std::cout << "Could not grasp object\n";}
        writeGripperState();
        if(!releaseObject()) {std::cout << "Could not release object\n";}
        writeGripperState();
    }

    pid_t initPandaController(ControlMode mode, char* ip) {
        shared_memory_object shm(open_or_create, "Panda Controller", read_write);
        shm.truncate(sizeof(shared_data));
        memoryRegion = new mapped_region(shm, read_write);
        new (memoryRegion->get_address()) shared_data;
        SharedData = (shared_data*)memoryRegion->get_address();

        if (ip == NULL) {
            ip = "10.134.71.22";
        }    
        p_gripper = new franka::Gripper(ip);

        std::cout << "Starting" << std::endl;

        SharedData->running = true;
        SharedData->start_time = std::chrono::system_clock::now();
        pid_t pid = fork();
        if (pid == 0) {
            switch(mode){
                case PandaController::ControlMode::CartesianVelocity:
                    runVelocityController(ip);
                    break;
                case PandaController::ControlMode::JointVelocity:
                    runJointVelocityController(ip);
                    break;
                case PandaController::ControlMode::None:
                    noController(ip);
                    break;
             }
            // gripperTest();
            stopControl();
            exit(0);
        }
        return pid;
    }

    void startLogging() {
        if (SharedData == NULL) throw "Must initialize shared memory space first";
        SharedData->logging = true;
    }
    void stopLogging() {
        if (SharedData == NULL) return;
        SharedData->logging = false;
    }

    bool isRunning() {
        if (SharedData == NULL) return false;
        return SharedData->running;
    }

    std::array<double, 3> readCommandedPosition(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->commanded_position;
    }
    
    void writeCommandedPosition(std::array<double, 3> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->commanded_position = data;
    }

    std::array<double, 7> readPoseGoal(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->pose_goal;
    }

    void writePoseGoals(std::array<double, 7> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->pose_goal = data;
    }
    
    std::array<double, 7> readJointAngles(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->joint_angles;
    }

    void writeJointAngles(std::array<double, 7> data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->joint_angles = data;
    }
    
    franka::RobotState readRobotState(){
        if (SharedData == NULL) throw "Must initialize shared memory space first";
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        return SharedData->current_state;
    }
    
    void writeRobotState(franka::RobotState data){
        if (SharedData == NULL) return;
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        SharedData->jacobian = calculatePandaJacobian(data.q);
        SharedData->iteration++;
        if (SharedData->logging) {
            SharedData->buffer_end++;
            if (SharedData->buffer_end > 1000){
                SharedData->buffer_end = 1;
                SharedData->buffer_start = 0;
            }
            SharedData->buffer[SharedData->buffer_end-1] = data;
            SharedData->timestamps[SharedData->buffer_end-1] = (std::chrono::system_clock::now() - SharedData->start_time).count();
        }
        SharedData->current_state = data;
    }

    void consumeBuffer(int &count, franka::RobotState* result, long* times){
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(SharedData->mutex);
        count = 0;
        for (int i = SharedData->buffer_start; i < SharedData->buffer_end; i++){
            times[count] = SharedData->timestamps[i];
            result[(count)++] = SharedData->buffer[i];
        }
        SharedData->buffer_start = 0;
        SharedData->buffer_end = 0;
    }


} //end namespace PandaController
