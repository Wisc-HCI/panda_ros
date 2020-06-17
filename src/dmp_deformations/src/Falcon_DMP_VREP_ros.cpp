#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <relaxed_ik/EEPoseGoals.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

#include "BSplineSurface.h"
#include "nlopt.hpp"
#include <iomanip>
#include <thread>

using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;

FalconDevice m_falconDevice;

// For calculating the falcon velocity
array<double, 3> falcon_vel = {0.0, 0.0, 0.0};
std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
std::array<double, 3> last_falcon = {0.0, 0.0, 0.0};

array<double, 3> actual_pos;

bool last_falcon_updated = false;

// For recording
std::ofstream outputfile;
double x, y, z, fx, fy, fz;
double dmp_fx, dmp_fy, dmp_fz;

bool replay_active = false;


////////////////////////////////
// General robot utility fxns //
////////////////////////////////

/**
 * Calculates the cross product between 2 3-D vectors
 */
array<double,3> crossProduct(array<double,3> x, array<double,3> y){
    array<double,3> return_vec;
    return_vec[0]=x[1]*y[2]-x[2]*y[1];
    return_vec[1]=x[2]*y[0]-x[0]*y[2];
    return_vec[2]=x[0]*y[1]-x[1]*y[0];
    return return_vec;
}

/**
 * Rotates a vector into a frame (represented as a quaternion)
 */
array<double,3> vectorIntoConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw){
    // Make sure the quaternion is normalized (otherwise, there will be skew)
    double mag = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

    // Calculate the rotation matrix
    double r11=1.0-2*qy*qy-2*qz*qz;
    double r12=2*qx*qy-2*qz*qw;
    double r13=2*qx*qz+2*qy*qw;
    double r21=2*qx*qy+2*qz*qw;
    double r22=1.0-2*qx*qx-2*qz*qz;
    double r23=2*qy*qz-2*qx*qw;
    double r31=2*qx*qz-2*qy*qw;
    double r32=2*qy*qz+2*qx*qw;
    double r33=1.0-2*qx*qx-2*qy*qy;

    // Rotate the vector via the rotation matrix
    array<double,3> rotated_vector;
    rotated_vector[0] = r11*x+r12*y+r13*z;
    rotated_vector[1] = r21*x+r22*y+r23*z;
    rotated_vector[2] = r31*x+r32*y+r33*z;
    return rotated_vector;
}

/**
 * Rotates a vector OUT OF a frame (represented as a quaternion)
 */
array<double,3> vectorOutOfConstraintFrame(double x, double y, double z, double qx, double qy, double qz, double qw){
    // Make sure the quaternion is normalized (otherwise, there will be skew)
    double mag = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

    // Calculate the rotation matrix
    double r11=1.0-2*qy*qy-2*qz*qz;
    double r12=2*qx*qy-2*qz*qw;
    double r13=2*qx*qz+2*qy*qw;
    double r21=2*qx*qy+2*qz*qw;
    double r22=1.0-2*qx*qx-2*qz*qz;
    double r23=2*qy*qz-2*qx*qw;
    double r31=2*qx*qz-2*qy*qw;
    double r32=2*qy*qz+2*qx*qw;
    double r33=1.0-2*qx*qx-2*qy*qy;

    // Rotate the vector out of the frame
    // by multiplying by the transpose of
    // the rotation matrix
    array<double,3> rotated_vector;
    rotated_vector[0] = r11*x+r21*y+r31*z;
    rotated_vector[1] = r12*x+r22*y+r32*z;
    rotated_vector[2] = r13*x+r23*y+r33*z;
    return rotated_vector;
}

/**
 * This function takes a rotation matrix expressed as 3 column vectors and creates
 * a quaternion of the form x,y,z,w
 */
void rotationToQuaternion(array<double,3> x_hat, array<double,3> y_hat, array<double,3>z_hat, array<double,4> &q_out){
    // Using simple (non-optimal) formulation found here: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    // page 2

    double trace = x_hat[0] + y_hat[1] + z_hat[2];
    double k;

    // Positive trace - W is largest
    if(trace > 0.0){
        k = 0.5/(sqrt(1.0+trace));
        q_out = {k*(z_hat[1]-y_hat[2]), k*(x_hat[2]-z_hat[0]), k*(y_hat[0]-x_hat[1]), 0.25/k};
    }

    // X is largest
    else if ((x_hat[0]>y_hat[1]) && (x_hat[0]>z_hat[2])){
        k = 0.5/(sqrt(1.0+x_hat[0]-y_hat[1]-z_hat[2]));
        q_out = {0.25/k, k*(x_hat[1]+y_hat[0]), k*(x_hat[2]+z_hat[0]), k*(z_hat[1]-y_hat[2])};
    }

    // Y is largest
    else if (y_hat[1]>z_hat[2]){
        k = 0.5/(sqrt(1.0+y_hat[1]-x_hat[0]-z_hat[2]));
        q_out = {k*(x_hat[1]+y_hat[0]), 0.25/k, k*(y_hat[2]+z_hat[1]), k*(x_hat[2]-z_hat[0])};
    }

    // Z is largest
    else{
        k = 0.5/(sqrt(1.0+z_hat[2]-x_hat[0]-y_hat[1]));
        q_out = {k*(x_hat[2]+z_hat[0]), k*(y_hat[2]+z_hat[1]), 0.25/k, k*(y_hat[0]-x_hat[1])};
    }

    // Make sure it is normalized
    double mag = sqrt(q_out[0]*q_out[0]+q_out[1]*q_out[1]+q_out[2]*q_out[2]+q_out[3]*q_out[3]);
    q_out[0] = q_out[0]/mag; q_out[1] = q_out[1]/mag; q_out[2] = q_out[2]/mag; q_out[3] = q_out[3]/mag;
}

///////////////////////////////
// FALCON-SPECIFIC FUNCTIONS //
///////////////////////////////
void falconVelocity() {
    double delta_T = 1000;
    // Compute the velocity to add some viscous friction to help with stability
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    // Provide there is a last position to compute the velocity, calculate with backwards differencing
    if(last_falcon_updated)
    {
        // Velocity calculated as basic differencing
        falcon_vel = {(falconPos[0]-last_falcon[0])/delta_T,(falconPos[1]-last_falcon[1])/delta_T,(falconPos[2]-last_falcon[2])/delta_T};
    }

    //Update last falcon for use in the velocity calculation
    last_falcon[0] = falconPos[0];
    last_falcon[1] = falconPos[1];
    last_falcon[2] = falconPos[2];

    last_falcon_updated = true;
}

bool haptic_cue_increased = false;
bool haptic_cue_decreased = false;
int haptic_cue_count = 0;

void run_falcon_deformation_controller(){
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    array<double, 3> falconPos = {0,0,0};

    long count = 0;   

    falconVelocity();
    
    while(1){
        m_falconDevice.runIOLoop();
        falconPos = m_falconDevice.getPosition();

        count++;

        // Haptic Cues
        if (haptic_cue_increased){
            if(haptic_cue_count>500){
                haptic_cue_increased=false;
                haptic_cue_count = 0;
            }
            else{
                // goes up by 20 after half a second then back
                stiffness = 200.0-(100.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0);
                viscous_replay = 3.0*sqrt(stiffness);
                haptic_cue_count++;
                //cout << "HCI" << endl;
            }
        }

        if (haptic_cue_decreased){
            if(haptic_cue_count>500){
                haptic_cue_decreased=false;
                haptic_cue_count = 0;
            }
            else{
                // goes up by 20 after half a second then back
                stiffness = 50.0+(50.0/62500.0)*(haptic_cue_count-250.0)*(haptic_cue_count-250.0);
                viscous_replay = 3.0*sqrt(stiffness);
                haptic_cue_count++;
                //cout << "HCD" << endl;
            }
        }

        //cout << "FP:" << falconPos[0] << " " << falconPos[1] << " " << falconPos[2] << endl;
        // zero displacement mode
        // falcon has offset in z
        m_falconDevice.setForce({
                -stiffness*falconPos[0]-viscous_replay*falcon_vel[0], 
                -stiffness*falconPos[1]-viscous_replay*falcon_vel[1], 
                -stiffness*(falconPos[2]-0.125)-viscous_replay*falcon_vel[2]});

        // Store forcing from falcon for deformations
        // Note: these should be unit-normalized (i.e., span from -1 to 1)
        dmp_fx = -(falconPos[0])/(0.06);
        dmp_fy = (falconPos[2]-0.125)/(0.05);
        dmp_fz = falconPos[1]/(0.055);
        usleep(1000);
    }
}

/**
 * Returns a boolean array for the current state of the four buttons on the falcon
 * 1 is pressed, 0 is unpressed
 */
array<bool,4> getButtons(){
    unsigned int my_buttons = m_falconDevice.getFalconGrip()->getDigitalInputs();
    array<bool,4> buttons;
    buttons[0] = (my_buttons & libnifalcon::FalconGripFourButton::CENTER_BUTTON)  ? 1 : 0;
    buttons[1] = (my_buttons & libnifalcon::FalconGripFourButton::PLUS_BUTTON)    ? 1 : 0;
    buttons[2] = (my_buttons & libnifalcon::FalconGripFourButton::MINUS_BUTTON)   ? 1 : 0;
    buttons[3] = (my_buttons & libnifalcon::FalconGripFourButton::FORWARD_BUTTON) ? 1 : 0;
    return buttons;
}

////////////////////////////
// DEFORMATION FUNCTIONS  //
////////////////////////////

/**
 * Checks whether a haptic cue should be issues based on a changing variance
 */
double prev_var_x = 0.0; double prev_var_y = 0.0; double prev_var_z = 0.0;
bool var_x_changing = false; bool var_y_changing = false; bool var_z_changing = false;
void check_for_haptic_cue(double var_x, double var_y, double var_z){
    if(var_x != prev_var_x && !var_x_changing)
    {
        var_x_changing = true;
        if(var_x-prev_var_x>0.0){
            // increasing range -> push user in
            haptic_cue_increased = true;
        }

        else{
            // decreasing range -> push user out
            haptic_cue_decreased = true;
        }
    }

    else if(var_x==prev_var_x){
        var_x_changing=false;
    }

    prev_var_x = var_x; prev_var_y = var_y; prev_var_z = var_z;

}

// Next 3 pieces are for surface optimization
typedef struct {
    double x, y, z;
    BSplineSurface surface;
} opt_data;

double obj_closest_surface(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    opt_data *d = (opt_data *) data;
    array<double,3> r = {0.0, 0.0, 0.0};
    array<double,3> n_hat = {0.0, 0.0, 0.0};
    array<double,3> r_u = {0.0, 0.0, 0.0};
    array<double,3> r_v = {0.0, 0.0, 0.0};
    d->surface.calculateSurfacePoint(x[0],x[1],r,n_hat,r_u,r_v);
    
    return (r[0]-d->x)*(r[0]-d->x)+(r[1]-d->y)*(r[1]-d->y)+(r[2]-d->z)*(r[2]-d->z);
}

void getClosestParams(double x, double y, double z, double &u, double&v, BSplineSurface surface){
    // Find the closest point on the surface using NL-opt
    nlopt::opt opt(nlopt::LN_COBYLA, 2);
    
    // Bounds of surface are 0 to 1 in U,V directions
    std::vector<double> lb(2);
    lb[0] = 0.0; lb[1] = 0.0;
    std::vector<double> ub(2);
    ub[0] = 1.0; ub[1] = 1.0;
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // Initial guess for parameters U,V
    std::vector<double> params(2);
    params[0] = u; params[1] = v;
    double minf;

    opt_data data;
    data.x=x; data.y=y; data.z=z;
    data.surface = surface;

    opt.set_min_objective(obj_closest_surface,&data);
    opt.set_xtol_rel(1e-4);

    try{
        nlopt::result result = opt.optimize(params, minf);
        
        // Output new values if found
        u = params[0]; v=params[1];
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}

/**
 * Streams robot state data to an output file (csv). This is used for when
 * the demonstration DMPs are learned from demonstration
 */
void log_demonstration(double x, double y, double z, double fx, double fy, double fz){
    outputfile << x << "," << y << "," << z << "," << fx << "," << fy << "," << fz << "\n";
}

/**
 * TODO: fill this out
 */
void bidirectional_checker(array<double,3> &vec, array<double,3> &prev_vec){
    // Check if velocity should be flipped (tool goes both ways)
    // TODO: change to a function
    double sign = prev_vec[0]*vec[0] +  prev_vec[1]*vec[1] +  prev_vec[2]*vec[2];
    cout << "SIGN:" << sign << endl;
    if(sign<0.0){
        vec[0] = -vec[0]; vec[1] = -vec[1]; vec[2] = -vec[2]; 
    }
}

/**
 * Determines whether a transition should occur to the next DMP based on residual "Error"
  from the deformations

  transition_x,transition_y, transition_z are outputs representing the change to be applied
  to the next starting point
 */
void calculateDMPTransition(double ii, double &transition_x, double &transition_y, double &transition_z, double &transition_def_x, double &transition_def_y, double &transition_def_z, double x_imp, double y_imp, double z_imp, double x_def, double y_def, double z_def, vector<array<double,3>> selections,vector<array<double,7>> starting_points, vector<string> surfaces){
    if((ii+1)<selections.size()){
        // TWO TRANSITIONS -> need 6 transition variables, need the stiffness, need the deformation scaling

        // Situations - same requires direct addition
        // position to force -> nothing
        // force to position -> nothing
        // position to manifold
        // manifold to position

        if (selections[ii][2]==0.0 && selections[ii+1][2]==1.0){
            // On the manifold currently and coming off it!
            BSplineSurface surface;
            surface.loadSurface(surfaces[ii]);
            array<double,3> r; array<double,3> n_hat; array<double,3> r_u; array<double,3> r_v;

            // transition point based on current surface value (state + deformation)
            surface.calculateSurfacePoint(x_imp+x_def,y_imp+y_def,r,n_hat,r_u,r_v);

            transition_x = (r[0]-starting_points[ii+1][0]);
            transition_y = (r[1]-starting_points[ii+1][1]);
            transition_z = (r[2]-starting_points[ii+1][2]);

            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }

        else if (selections[ii][2]==1.0 && selections[ii+1][2]==0.0){
            
            // Need the surface that is being approached
            BSplineSurface surface;
            surface.loadSurface(surfaces[ii+1]);

            // Approaching the manifold, find the nearest manifold point to transition
            double u = starting_points[ii+1][0];
            double v = starting_points[ii+1][1];
            getClosestParams(actual_pos[0], actual_pos[1], actual_pos[2],u,v,surface);

            transition_x = u-starting_points[ii+1][0];
            transition_y = v-starting_points[ii+1][1];
            transition_z = 0.0;

            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }

        else if (selections[ii][2]==0.0 && selections[ii+1][2]==0.0){
            // Staying on the surface - continuity of all local variables
            transition_x = (x_imp-starting_points[ii+1][0]);
            transition_y = (y_imp-starting_points[ii+1][1]);
            transition_z = (z_imp-starting_points[ii+1][2]);

            transition_def_x = x_def;
            transition_def_y = y_def;
            transition_def_z = z_def;
        }

        else{
            transition_x = 0.0; transition_y = 0.0; transition_z = 0.0;
            transition_def_x = 0.0; transition_def_y = 0.0; transition_def_z = 0.0; 
        }
    }
}

/**
 * TODO: Calculate whether there is a chance for collision or something...?
 */
void calculateNearestContact(int ii,vector<array<double,3>> selections,vector<array<double,7>> starting_points,vector<array<double,7>> attractor_points,bool &dmp_x_limiting,bool &dmp_y_limiting,bool &dmp_z_limiting,double &dmp_x_collision, double &dmp_y_collision, double &dmp_z_collision){
    
    // This is used to contact the "Nearest contact condition" to consider
    // minimizing the deformations
    dmp_x_limiting = false;
    dmp_y_limiting = false;
    dmp_z_limiting = false;

    // TODO: check over all this collision stuff WRT to the new rotations
    // There is a next DMP, check for pos to force transition
    if (ii+1<selections.size()){
        if(selections[ii][2] ==1 && selections[ii+1][2]==0){
            dmp_x_collision = attractor_points[ii][0];
            dmp_y_collision = attractor_points[ii][1];
            dmp_z_collision = attractor_points[ii][2];
            dmp_x_limiting=true;
            dmp_y_limiting=true;
            dmp_z_limiting=true;
        }
    }

    // There is a previous DMP, check for force to pos transition
    if (ii>0){
        if(selections[ii-1][0] ==0 && selections[ii][0]==1){
            dmp_x_collision = starting_points[ii][0];
            dmp_y_collision = starting_points[ii][1];
            dmp_z_collision = starting_points[ii][2];
            dmp_x_limiting=true;
            dmp_y_limiting=true;
            dmp_z_limiting=true;
        }
    }
}

/**
 * Calculate the final deformation based on adaptive gains, collisions, workspace limits, etc.
 */
array<double,3> deformationScaling(array<double,3> &rotated_deformation, double var_x, double var_y, double var_z, geometry_msgs::Vector3 selection, double x, double y, double collision_scaling){
    
    array<double,3> final_deformation;

    double k= 50;

    // First scale based on the type of input
    final_deformation[0] = var_x*rotated_deformation[0];
    final_deformation[1] = var_y*rotated_deformation[1];
    final_deformation[2] = var_z*rotated_deformation[2];

    // Parameterized surfaces have a range of 0<=x<=1
    // Make sure it is not over the bounds (for hybrid control)
    double minEdgePercent = 0.05;
    if (selection.z == 0){
        // Direction 1
        if((final_deformation[0]/k+x)>(1.0-minEdgePercent)){
            final_deformation[0] = k*((1.0-minEdgePercent)-x);
        }
        else if((final_deformation[0]/k+x)<(minEdgePercent)){
            final_deformation[0] = k*((minEdgePercent)-x);
        }

        // Direction 2
        if((final_deformation[1]/k+y)>(1.0-minEdgePercent)){
            final_deformation[1] = k*((1.0-minEdgePercent)-y);
        }
        else if((final_deformation[1]/k+y)<(minEdgePercent)){
            final_deformation[1] = k*((minEdgePercent)-y);
        }

    }

    // Scale back based on potential collision
    final_deformation[0] = final_deformation[0]/collision_scaling;
    final_deformation[1] = final_deformation[1]/collision_scaling;
    final_deformation[2] = final_deformation[2]/collision_scaling;

    return final_deformation;
}

/**
 * This function takes in all of the relevant parameters for a particular DMP and will figure out the
 * starting velocity. This is used for setting the tool orientation as it approaches the surface.
 */
array<double,3> getFirstSurfaceVelocity(array<double,7> attractor_point, array<double,7> starting_point,double dmp_u,double dmp_v, array<double,3> r_u, array<double,3> r_v ){
    
    // TODO: can this be zero?
    double k=50;
    array<double,3> vel_hat;

    // These next 2 are equivalent to calculating the first velocity in the DMP ignoring the deformation and damping (since the velocity is initially 0)

    // Calculate vel-U direction
    double ddu = k*(attractor_point[0]-starting_point[0])+dmp_u;

    // Calculate vel-V direction
    double ddv = k*(attractor_point[1]-starting_point[1])+dmp_v;

    vel_hat[0] = r_u[0]*ddu+r_v[0]*ddv;
    vel_hat[1] = r_u[1]*ddu+r_v[1]*ddv;
    vel_hat[2] = r_u[2]*ddu+r_v[2]*ddv;
    double vel_mag = sqrt(vel_hat[0]*vel_hat[0]+vel_hat[1]*vel_hat[1]+vel_hat[2]*vel_hat[2]);
    vel_hat[0]=vel_hat[0]/vel_mag; vel_hat[1]=vel_hat[1]/vel_mag; vel_hat[2]=vel_hat[2]/vel_mag;
    return vel_hat;
}

array<double,3> map_deformation_input(int method, double dmp_fx,double dmp_fy,double dmp_fz,double dx,double dy, double dz){
    // TODO: add the selection vector stuff

    // Methods:
    // 1. Task oriented frame - define in terms of a principal axis for the task
    // 2. Velocity-oriented frame - deformation is a relative direction based on the current velocity

    // Rotate the deformation force into the relevant frame
    array<double,3> deformation_rotation;

    if (method==1){ 
        /////////////////
        // Task-Oriented Frame
        /////////////////
        // This quaternion rotates it to be aligned with the general camera/x-y directions
        deformation_rotation = vectorIntoConstraintFrame(dmp_fx, dmp_fy, dmp_fz, 0.0, 0.0, -1.0, 0.0);
        return deformation_rotation;
    }

    else if (method==2){
        /////////////////
        // Velocity-oriented
        /////////////////
        array<double,3> manifold_vel_dir = {1.0, 0.0, 0.0}; // z is always zero as these are functionally
        // homogeneous coordinates since we are talking about a 2D projection + force

        if(dx!=0.0 || dy != 0.0)
        {
            manifold_vel_dir = {dx, dy, 0.0};
            double mag = sqrt(dx*dx+dy*dy);
            manifold_vel_dir[0] = manifold_vel_dir[0]/mag; manifold_vel_dir[1] = manifold_vel_dir[1]/mag;
        }

        //cout << "MD:" << manifold_vel_dir[0]  << " " << manifold_vel_dir[1] << " " << manifold_vel_dir[2] << endl;

        array<double,3> y_temp;
        array<double,3> z_hat = {0.0, 0.0, 1.0};
        y_temp = crossProduct(z_hat,manifold_vel_dir);

        array<double,4> q_out_rc;
        rotationToQuaternion(manifold_vel_dir, y_temp,z_hat, q_out_rc);
        array<double,3> falconRotation = vectorIntoConstraintFrame(dmp_fx,dmp_fy,dmp_fz,0.0, 0.0, 0.7071, 0.7071);
        deformation_rotation = vectorOutOfConstraintFrame(falconRotation[0], falconRotation[1], falconRotation[2], q_out_rc[0], q_out_rc[1], q_out_rc[2], q_out_rc[3]);
        return deformation_rotation;
    }

    else{ // Invalid method - No rotation
        deformation_rotation = {dmp_fx, dmp_fy, dmp_fz};
        cout << "INVALID METHOD FOR INPUT MAPPING" << endl;
        return deformation_rotation;
    }
}

/**
 * TODO: fill this out
 */
void forceOnloading(int ii, geometry_msgs::Vector3 selection, vector<array<double,7>> starting_points, vector<array<double,7>> attractor_points, vector<vector<array<double,7>>> dmps, BSplineSurface curr_surface, ros::Publisher &selection_vector_pub, ros::Publisher &constraint_frame_pub, ros::Publisher pose_goal_pub, ros::Publisher wrench_goal_pub){
    // Do force onloading with the first sample
    cout << "Force Onloading Started..." << endl;
    geometry_msgs::Wrench ft;
    geometry_msgs::Pose pose;

    ft.force.x = starting_points[ii][0];
    ft.force.y = starting_points[ii][1];
    ft.force.z = starting_points[ii][2];
    pose.position.x = starting_points[ii][0];
    pose.position.y = starting_points[ii][1];
    pose.position.z = starting_points[ii][2];
    
    // TODO: FIX THIS!
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    // TODO - FIX THIS FIX THIS FIX THIS!!!!!
    array<double,3> r, n_hat, y_hat, x_hat;
    curr_surface.calculateSurfacePoint(starting_points[ii][0],starting_points[ii][1],r,n_hat,x_hat,y_hat);

    // Calculate the starting point for orientation based on the first velocity of the DMP
    array<double,3> vel_hat = getFirstSurfaceVelocity(attractor_points[ii],starting_points[ii],dmps[ii][0][0],dmps[ii][0][1],x_hat,y_hat);

    // Z (normal) x X (vel) = +Y
    array<double,3> y_new = crossProduct(n_hat,vel_hat);

    geometry_msgs::Quaternion constraint_frame; 
    array<double,4> q_out;               
    rotationToQuaternion(vel_hat,y_new,n_hat,q_out);
    // rotationToQuaternion(x_hat,y_hat,n_hat,q_out);
    constraint_frame.x = q_out[0]; constraint_frame.y = q_out[1]; constraint_frame.z = q_out[2]; constraint_frame.w = q_out[3];

    // Convert the XYZ into the constraint frame
    array<double,3> temp_vec = vectorIntoConstraintFrame(r[0],r[1],r[2],q_out[0],q_out[1],q_out[2],q_out[3]);
    pose.position.x = temp_vec[0]; pose.position.y = temp_vec[1]; pose.position.z=temp_vec[2];

    //cout << "CF: " << constraint_frame.x << " " << constraint_frame.y << " " << constraint_frame.z << " " << constraint_frame.w << endl;
    //cout << "XYZ:" << pose.position.x << " " << pose.position.y << " " << x_hat[2]*r[0]+y_hat[2]*r[1]+n_hat[2]*r[2] << endl;
    // cout << "RU " << x_hat[0] << " " << x_hat[1] << " " << x_hat[2] << endl;
    // cout << "RV " << y_hat[0] << " " << y_hat[1] << " " << y_hat[2] << endl;
    // cout << "NHAT " << n_hat[0] << " " << n_hat[1] << " " << n_hat[2] << endl;

    selection_vector_pub.publish(selection);
    constraint_frame_pub.publish(constraint_frame);
    pose_goal_pub.publish(pose);
    wrench_goal_pub.publish(ft);

    // This loop monitors the robot which has started moving in the
    // force direction until it achieves the desired force (within a tolerance)
    bool proper_contact = false;
    while(!proper_contact)
    {
        //proper_contact=true; // TODO: REMOVE/FIX THIS
        double f_z_rotated = x_hat[2]*fx+y_hat[2]*fy+n_hat[2]*fz;
        cout << "FZ: " << f_z_rotated << " " << starting_points[ii][2] << endl;
        if(f_z_rotated<0.95*starting_points[ii][2] && f_z_rotated>1.05*starting_points[ii][2])
        {
            proper_contact = true;
        }
        
        // This keeps the falcon in zero-displacement mode while this loop runs
        for (int yy=0; yy<10; yy++)
        {
            usleep(1000);
        }
        ros::spinOnce();
    }
    cout << "Force Onloading Complete..." << endl;
}


void readDemo(vector<vector<array<double,7>>> &dmps,vector<array<double,3>> &selections,vector<array<double,7>> &starting_points,vector<array<double,7>> &attractor_points, vector<string> &surfaces, vector<vector<array<double,3>>> &variance_dmp){
    std::ifstream dmpfile("learneddmp.csv");
    double junk;
    string temp;
    string surface;

    vector<array<double,7>> dmp_temp;
    vector<array<double,3>> dmp_variance_temp;

    bool loading_variances = false;

    // Read entire file into the vectors
    if(dmpfile.good())
    {
        while(getline(dmpfile,temp,',')){
            // Either a new mode definition or continued values
            if(temp=="mode")
            {
                // New mode, need to create vector
                // for trajectory, get selection and
                // starting/attractor points
                // make this surface and stiffness (damping implied from overdamped)

                loading_variances = false;

                getline(dmpfile,temp,',');
                surface = temp.c_str();
                surfaces.push_back(surface);
                getline(dmpfile,temp);
                junk = atof(temp.c_str());

                // Read Selection Vector
                array<double,3> selection_temp;

                getline(dmpfile,temp,','); selection_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); selection_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp); selection_temp[2] = atof(temp.c_str());
                selections.push_back(selection_temp);

                // Read Starting Points
                array<double,7> starting_temp;
                getline(dmpfile,temp,','); starting_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,','); starting_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp); starting_temp[6] = atof(temp.c_str());
                starting_points.push_back(starting_temp);

                // Read Attractor Points
                array<double,7> attractor_temp;
                getline(dmpfile,temp,','); attractor_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,','); attractor_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp); attractor_temp[6] = atof(temp.c_str());
                attractor_points.push_back(attractor_temp);

                // Send the previous DMP if it has something in it
                // this avoids sending an empty DMP at the beginning
                if(dmp_temp.size()>0)
                {
                    dmps.push_back(dmp_temp);
                    variance_dmp.push_back(dmp_variance_temp);
                }

                // Reinitialize the temporary vector
                // to store the trajectory
                dmp_temp.clear();
                dmp_variance_temp.clear();
            }

            else if(temp=="variance")
            {
                loading_variances = true;
                getline(dmpfile,temp);
            }

            else if(loading_variances){
                array<double,3> variance_temp;
                // temp has first value already
                variance_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                variance_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp);
                variance_temp[2] = atof(temp.c_str());
                dmp_variance_temp.push_back(variance_temp);
            }

            else{ // Add new value to dmp_vector
                array<double,7> state_vector_temp;
                // temp has first value already
                state_vector_temp[0] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[1] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[2] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[3] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[4] = atof(temp.c_str());
                getline(dmpfile,temp,',');
                state_vector_temp[5] = atof(temp.c_str());
                getline(dmpfile,temp);
                state_vector_temp[6] = atof(temp.c_str());
                dmp_temp.push_back(state_vector_temp);
            }

        }

        // Publish the final dmp
        if(dmp_temp.size()>0)
        {
            dmps.push_back(dmp_temp);
            variance_dmp.push_back(dmp_variance_temp);
        }
    }

    //close the filestream
    dmpfile.close();
}

void replay_demo(ros::Publisher pose_goal_pub, ros::NodeHandle n){

    std_msgs::String replay_str;
    
    // Publishers specifically used for replay
    ros::Publisher selection_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/selection", 5);
    ros::Publisher wrench_goal_pub = 
        n.advertise<geometry_msgs::Wrench>("/panda/ee_wrench_goals", 5);
    ros::Publisher pose_path_pub = 
        n.advertise<geometry_msgs::Pose>("/panda/ee_path_goals", 5);    
    ros::Publisher constraint_frame_pub = 
        n.advertise<geometry_msgs::Quaternion>("/panda/constraintframe", 5);
    ros::Publisher point_goal_pub = // Used to project the future path for the user TODO: actually
        n.advertise<geometry_msgs::Pose>("/panda/ee_point_goals", 5);
    ros::Publisher dmp_replay_pub = 
        n.advertise<std_msgs::String>("/dmp/replay", 5);

    // Publishers specifically for intent visualization
    ros::Publisher defscaling_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/defscaling", 5);
    ros::Publisher nominaltraj_vector_pub = 
        n.advertise<geometry_msgs::Vector3>("/panda/nominaltraj", 5);

    // Pause 0.5s to allow publishers to register
    for(int jj=0; jj<500; jj++)
    {
        ros::spinOnce();
        usleep(1000);
    }

    // All of the data from the DMPs are stored in vectors
    vector<vector<array<double,7>>> dmps;
    vector<array<double,3>> selections;
    vector<array<double,7>> starting_points;
    vector<array<double,7>> attractor_points;
    vector<string> surfaces;
    vector<vector<array<double,3>>> variance_dmp;

    // Reused ROS messages for publishing
    geometry_msgs::Pose pose;
    geometry_msgs::Wrench ft;
    geometry_msgs::Vector3 selection;
    
    // DMP Parameters - TODO: READ THESE FROM THE FILE
    double k=50;
    double b=sqrt(2*k); // ideal underdamped
    double b_def=2*sqrt(k); // overdamped
    readDemo(dmps,selections,starting_points,attractor_points,surfaces,variance_dmp);

    // Action: Tell the robot the replay is starting over
    // In case something needs to change or be reset in the simulation state
    replay_str.data = "reset";
    dmp_replay_pub.publish(replay_str);

    // Tell the robot to go to the overall starting point
    // Pose path pub will interpolate the path
    pose.position.x = starting_points[0][0];
    pose.position.y = starting_points[0][1];
    pose.position.z = starting_points[0][2];
    pose_path_pub.publish(pose);
        
    // Sleep for 2.5 seconds to allow path completion
    for(int jj=0; jj<2500; jj++)
    {
        ros::spinOnce();
        usleep(1000);
    }

    // TODO MAKE THE ABOVE CLOSED LOOP (waits until it gets to the starting point)!!!!!!!!!!

    // Action: Tell the robot the replay is starting
    cout << "Replay Starting..." << endl;
    replay_str.data = "start";
    dmp_replay_pub.publish(replay_str);   

    // Variables needed for transitions and deformation collision-limiting
    bool previous_dmp_no_contact = true;
    double transition_x=0.0, transition_y=0.0, transition_z=0.0;
    double transition_def_x=0.0, transition_def_y=0.0, transition_def_z=0.0;
    bool dmp_x_limiting;
    bool dmp_y_limiting;
    bool dmp_z_limiting;
    double dmp_x_collision = 0.0;
    double dmp_y_collision = 0.0;
    double dmp_z_collision = 0.0;
    double surface_scaling_x = 1.0; double surface_scaling_y = 1.0;

    BSplineSurface curr_surface;

    // Loop through to replay the demo
    for(int ii = 0; ii<selections.size();ii++)
    {
        if(surfaces[ii]!="")
        {
            // Load the B-spline surface
            curr_surface.loadSurface(surfaces[ii]);

            // get the surface ~ scaling
            surface_scaling_y = 0.5;

        }

        else{
            // surface scaling is 1.0
            surface_scaling_x = 1.0; surface_scaling_y = 1.0;
        }

        // First, publish selection vector
        selection.x =selections[ii][0];
        selection.y =selections[ii][1];
        selection.z =selections[ii][2];

        
        if((selection.x==0 || selection.y==0 || selection.z==0)){
            if(previous_dmp_no_contact){
                // If selection has force control and is transitioning from no-contact, need force onloading
                forceOnloading(ii,selection,starting_points,attractor_points,dmps,curr_surface,selection_vector_pub,constraint_frame_pub,pose_goal_pub,wrench_goal_pub);
            }
            previous_dmp_no_contact = false;
        }

        else{
            previous_dmp_no_contact = true;
        }

        // Check for prominent collisions that should be avoided in deformation (aka known contact points from the model)
        calculateNearestContact(ii,selections,starting_points,attractor_points,dmp_x_limiting,dmp_y_limiting,dmp_z_limiting,dmp_x_collision,dmp_y_collision,dmp_z_collision);
        
        // Set up the values for the new DMP
        double ddx, ddy, ddz;
        double dx = 0.0, dy = 0.0, dz = 0.0;
        double x=starting_points[ii][0]+transition_x, y=starting_points[ii][1]+transition_y, z=starting_points[ii][2]+transition_z;

        // For the general impedance model
        double ddx_imp, ddy_imp, ddz_imp;
        double dx_imp = 0.0, dy_imp=0.0, dz_imp=0.0;
        double x_imp = starting_points[ii][0]+transition_def_x, y_imp=starting_points[ii][1]+transition_def_y, z_imp=starting_points[ii][2]+transition_def_z;
        double x_def = 0.0, y_def=0.0, z_def=0.0;

        // Set up the values for the Quaternion TODO: Move to CDMP
        double ddqx, ddqy, ddqz, ddqw;
        double dqx = 0.0, dqy = 0.0, dqz = 0.0, dqw=0.0;
        double qx=starting_points[ii][3], qy=starting_points[ii][4], qz=starting_points[ii][5], qw=starting_points[ii][6];
        
        array<double,3> prev_v_hat = {0.0, 0.0, 0.0};
        
        double s = 0; // phase variable used for time
        double delta_s = 1.0;

        // Now replay the entirety of the demonstration
        while(s<dmps[ii].size()){
            ros::spinOnce();

            // Interpolate each of the applied forces for the non-integer value s
            // TODO: this isn't right and once fixed, will also fail for the final sample
            double dmp_x = dmps[ii][(int)floor(s)][0]+(dmps[ii][(int)ceil(s)][0]-dmps[ii][(int)floor(s)][0])*(s-floor(s));
            double dmp_y = dmps[ii][(int)floor(s)][1]+(dmps[ii][(int)ceil(s)][1]-dmps[ii][(int)floor(s)][1])*(s-floor(s));
            double dmp_z = dmps[ii][(int)floor(s)][2]+(dmps[ii][(int)ceil(s)][2]-dmps[ii][(int)floor(s)][2])*(s-floor(s));

            double var_x = variance_dmp[ii][(int)floor(s)][0]+(variance_dmp[ii][(int)ceil(s)][0]-variance_dmp[ii][(int)floor(s)][0])*(s-floor(s));
            double var_y = variance_dmp[ii][(int)floor(s)][1]+(variance_dmp[ii][(int)ceil(s)][1]-variance_dmp[ii][(int)floor(s)][1])*(s-floor(s));
            double var_z = variance_dmp[ii][(int)floor(s)][2]+(variance_dmp[ii][(int)ceil(s)][2]-variance_dmp[ii][(int)floor(s)][2])*(s-floor(s));

            // If the variance starts changing (up or down), signal a haptic cue to the user
            check_for_haptic_cue(var_x,var_y,var_z);

            // Compute a potential DMP diminishing scale based on collisions
            double collision_scaling = 1.0;

            if(dmp_x_limiting){
                double dist_to_collision = sqrt((x-dmp_x_collision)*(x-dmp_x_collision)+(y-dmp_y_collision)*(y-dmp_y_collision)+(z-dmp_z_collision)*(z-dmp_z_collision));
                collision_scaling = 1-exp(-75*dist_to_collision);

            }
            
            geometry_msgs::Quaternion constraint_frame;
            constraint_frame.x=0.0; constraint_frame.y=0.0; constraint_frame.z=0.0; constraint_frame.w=1.0;
            double x_conv, y_conv, z_conv;
            
            ////////////////////////////////////////////////////////////////
            //      Deformation input mapping                             //
            ////////////////////////////////////////////////////////////////
            // 1 - Task Frame
            // 2 - Velocity Frame
            int input_mapping_method=1;
            array<double,3> rotated_deformation = map_deformation_input(input_mapping_method,dmp_fx,dmp_fy,dmp_fz,dx,dy,dz);

            /////////////////////////////////////////////////////////////////
            //      Deformation scaling                                    //
            /////////////////////////////////////////////////////////////////
            array<double,3> final_deformation = deformationScaling(rotated_deformation,var_x,var_y,var_z,selection,x_imp,y_imp, collision_scaling);

            // Turn off deformations
            //final_deformation[0] = 0.0; final_deformation[1]=0.0; final_deformation[2]=0.0;
            
            ////////////////////////////////////////////////////////////////
            //      Calculate New DMP values                              //
            ////////////////////////////////////////////////////////////////

            // Calculate New X (State 1)
            ddx = k*(attractor_points[ii][0]-x_imp)-b*dx+dmp_x;
            dx = dx + ddx*0.01*delta_s;
            x_imp = x_imp+dx*0.01*delta_s;
            
            ddx_imp = k*(-x_def)-b_def*dx_imp+final_deformation[0];
            dx_imp = dx_imp + ddx_imp*0.01*delta_s;
            x_def = x_def + dx_imp*0.01*delta_s;

            x = x_imp+x_def; 

            // Calculate New Y (State 2)
            ddy = k*(attractor_points[ii][1]-y_imp)-b*dy+dmp_y;
            dy = dy + ddy*0.01*delta_s;
            y_imp = y_imp+dy*0.01*delta_s;
            
            ddy_imp = k*(-y_def)-b_def*dy_imp+final_deformation[1];
            dy_imp = dy_imp + ddy_imp*0.01*delta_s;
            y_def = y_def + dy_imp*0.01*delta_s;

            y = y_imp+y_def; 

            // Calculate New Z (State 3)
            ddz = k*(attractor_points[ii][2]-z_imp)-b*dz+dmp_z;
            dz = dz + ddz*0.01*delta_s;
            z_imp = z_imp+dz*0.01*delta_s;
            
            ddz_imp = k*(-z_def)-b_def*dz_imp+final_deformation[2];
            dz_imp = dz_imp + ddz_imp*0.01*delta_s;
            z_def = z_def + dz_imp*0.01*delta_s;

            z = z_imp+z_def; 

            ////////////////////////////////////////////////
            // Calculate the orientation - NO DEFORMATIONS /
            ////////////////////////////////////////////////
            ddqx = k*(attractor_points[ii][3]-qx)-b*dqx;
            dqx = dqx + ddqx*0.01*delta_s;
            qx = qx+dqx*0.01*delta_s;
            ddqy = k*(attractor_points[ii][4]-qy)-b*dqy;
            dqy = dqy + ddqy*0.01*delta_s;
            qy = qy+dqy*0.01*delta_s;
            ddqz = k*(attractor_points[ii][5]-qz)-b*dqz;
            dqz = dqz + ddqz*0.01*delta_s;
            qz = qz+dqz*0.01*delta_s;
            ddqw = k*(attractor_points[ii][6]-qw)-b*dqw;
            dqw = dqw + ddqw*0.01*delta_s;
            qw = qw+dqw*0.01*delta_s;

            //cout << "X: " << x << " Y:" << y << endl;

            ////////////////////////////////////////////
            // Hybrid Control For Arbitrary Surfaces  //
            ////////////////////////////////////////////

            // If the mode is interaction, then a few things need to be converted:
            // 1. X and Y are the U,V of the parameterized surface and need to be
            // converted back to the X,Y,Z
            // 2. The orientation should be set to be the surface normal for the
            // given U and V
            // 3. Need to set the constraint frame based on the surface normal
            // 4. Rotate the X,Y,Z and orientation into the constraint frame
            x_conv = x; y_conv = y; z_conv = z;
            if(selection.z == 0)
            {
                // Calculate the constraint frame
                array<double,3> r = {0.0, 0.0, 0.0};
                array<double,3> n_hat = {0.0, 0.0, 0.0};
                array<double,3> y_hat;
                array<double,3> x_hat;
                curr_surface.calculateSurfacePoint(x,y,r,n_hat,x_hat,y_hat);

                cout << "R: " << r[0] << " " << r[1] << " " << r[2] << endl;
                
                // Using the velocity of x and y (u and v), calculate the time
                // derivative of the surface (i.e., vector-valued function)
                array<double,3> v_hat;
                v_hat[0]=x_hat[0]*dx+y_hat[0]*dy; v_hat[1]=x_hat[1]*dx+y_hat[1]*dy; v_hat[2]=x_hat[2]*dx+y_hat[2]*dy; // old velocity
                v_hat[0]=x_hat[0]*(dx+dx_imp)+y_hat[0]*(dy+dy_imp); v_hat[1]=x_hat[1]*(dx+dx_imp)+y_hat[1]*(dy+dy_imp); v_hat[2]=x_hat[2]*(dx+dx_imp)+y_hat[2]*(dy+dy_imp); // true velocity
                //fix velocity -> transition_testing
                // v_hat[0] = 1.0; v_hat[1] = 0.0; v_hat[2]=0.0;

                double v_mag = sqrt(v_hat[0]*v_hat[0] + v_hat[1]*v_hat[1] + v_hat[2]*v_hat[2]);

                cout << "VHAT: " << v_hat[0] << " " << v_hat[1] << " " << v_hat[2] << endl;
                
                // If the velocity direction is non-zero, re-align the constraint frame
                // If it is zero, it will keep its previous value
                if(v_mag>0.0)
                {
                    // change the constraint frame
                    v_hat[0]=v_hat[0]/v_mag; v_hat[1]=v_hat[1]/v_mag; v_hat[2]=v_hat[2]/v_mag;
                    //bidirectional_checker(v_hat,prev_v_hat); //check if it should be flipped
                    //cout << "vhat: " << v_hat[0] <<  " " << v_hat[1] << " " << v_hat[2] << endl;
                    //cout << "p_vhat: " << prev_v_hat[0] <<  " " << prev_v_hat[1] << " " << prev_v_hat[2] << endl;
                    //prev_v_hat[0] = v_hat[0]; prev_v_hat[1] = v_hat[1]; prev_v_hat[2] = v_hat[2]; 

                    // Z x X = Y
                    array<double,3> y_new;
                    y_new = crossProduct(n_hat,v_hat);
                
                    array<double,4> q_out;
                    rotationToQuaternion(v_hat,y_new,n_hat,q_out);
                    constraint_frame.x = q_out[0]; constraint_frame.y = q_out[1]; constraint_frame.z = q_out[2]; constraint_frame.w = q_out[3];
                }

                // Orientation is now just the identity in the constraint frame
                // NOTE: This is overwriting the DMP for orientation above!
                qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0;

                // Convert the XYZ into the constraint frame
                array<double,3> xyz_conv = vectorIntoConstraintFrame(r[0], r[1], r[2], constraint_frame.x, constraint_frame.y, constraint_frame.z, constraint_frame.w);
                x_conv = xyz_conv[0];
                y_conv = xyz_conv[1];
                z_conv = z; // Z-direction is the force (hybrid control)
                
            }

            // Compute the new velocity factor and
            // Update the time variable
            // TODO: Maybe use a covariance to try and remove units?
            // TODO: this should be all kinematic directions at the least?
            // TODO: get to the point where the deformation is one-normalized so things are less arbitrarily scaled
            double dir_x = dx/sqrt(dx*dx+dy*dy);
            double dir_y = dy/sqrt(dx*dx+dy*dy);
            // TODO: this aint right
            double dp_in_dir = dir_x*final_deformation[0] + dir_y*final_deformation[1];
            
            if(dp_in_dir > 0)
            {
                dp_in_dir = 0.0; // only allow slowing down
            }

            // TODO: fix all this stuff
            dp_in_dir = 0.0;

            delta_s = 1.0+dp_in_dir;
            s+=delta_s;


            /////////////////////////////////////////////////////
            // INTENT AND EXPECTATION ///////////////////////////
            /////////////////////////////////////////////////////
            
            // Publish enough to show the ellipsoid of expectation

            // Need deformation amount for x,y,z
            // Need the nominal value for x,y,z
            geometry_msgs::Vector3 def_range;
            def_range.x = var_x/k; def_range.y = var_y/k; def_range.z = var_z/k;

            if(selection.z==0){
                def_range.x = def_range.x*surface_scaling_x;
                def_range.y = def_range.y*surface_scaling_y;
            }
            
            geometry_msgs::Vector3 nominal;
            nominal.x = x_def; nominal.y = y_def; nominal.z = z_def;

            defscaling_vector_pub.publish(def_range);
            nominaltraj_vector_pub.publish(nominal);


            // PUBLISH THE ROBOT CONTROL PARAMETERS
            ft.force.x = x_conv; ft.force.y = y_conv; ft.force.z = z_conv;
            pose.position.x = x_conv; pose.position.y = y_conv; pose.position.z = z_conv;
            pose.orientation.x = qx; pose.orientation.y = qy; pose.orientation.z = qz; pose.orientation.w = qw;
            selection_vector_pub.publish(selection);
            constraint_frame_pub.publish(constraint_frame);
            pose_goal_pub.publish(pose);
            wrench_goal_pub.publish(ft);
            
            // Pause for 0.01 seconds, but calculate velocities at 0.001s
            for (int yy=0; yy<10; yy++)
            {
                usleep(1000);
            }
        }

        // AFTER THE DMP IS COMPLETE, look at whether transitions are needed
        // Combat the transition discontinuities if there is still a transition left
        calculateDMPTransition(ii,transition_x, transition_y, transition_z,transition_def_x,transition_def_y,transition_def_z,x_imp,y_imp,z_imp,x_def,y_def,z_def,selections, starting_points, surfaces);

    }

    // Tells simulation that things are over if necessary
    replay_str.data = "end";
    dmp_replay_pub.publish(replay_str);
    // Pause 0.5s to allow final message
    for(int jj=0; jj<500; jj++)
    {
        ros::spinOnce();
        usleep(1000);
    }
}
 
bool init_falcon() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
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
    m_falconDevice.setFalconGrip<libnifalcon::FalconGripFourButton>();

    m_falconDevice.runIOLoop();
    
    return true;
}

void publishPose(ros::Publisher pose_goal_pub, std::array<double, 7> panda_pose) {
    geometry_msgs::Pose pose_goal;
    pose_goal.position.x = panda_pose[0];
    pose_goal.position.y = panda_pose[1];
    pose_goal.position.z = panda_pose[2];
    pose_goal.orientation.x = panda_pose[3];
    pose_goal.orientation.y = panda_pose[4];
    pose_goal.orientation.z = panda_pose[5];
    pose_goal.orientation.w = panda_pose[6];
    pose_goal_pub.publish(pose_goal);
}

void pollFalcon(ros::Publisher pose_goal_pub, ros::Publisher command_pub, double* scaling_factors, double* offsets, bool* clutch, bool* reset_center, bool* freeze) {
    static bool lastCenterButton = false;
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 3> normalized_falcon = {0.0, 0.0, 0.0};
    
    // These convert the falcon to match the respective directions on the Kinova!
    normalized_falcon[0] = falconPos[0];
    normalized_falcon[1] = falconPos[2];
    normalized_falcon[2] = falconPos[1];

    // The first time the clutch is initiated, freeze the position of the falcon
    if (*freeze)
    {
        frozen_position[0] = normalized_falcon[0];
        frozen_position[1] = normalized_falcon[1];
        frozen_position[2] = normalized_falcon[2];
        *freeze = false;
    }

    // If still clutching, keep the position consistent based on the frozen position
    if(*clutch)
    {
        panda_pos[0] = scaling_factors[0] * frozen_position[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * frozen_position[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * frozen_position[2] + offsets[2];
        panda_pos[6] = 1;
    }

    else{ // Not clutching

        // If this is the first non-clutching sample, update the center of the workspace
        // based on the movement during the clutching action
        if(*reset_center)
        {
            offsets[0] = offsets[0]-scaling_factors[0]*(normalized_falcon[0]-frozen_position[0]);
            offsets[1] = offsets[1]-scaling_factors[1]*(normalized_falcon[1]-frozen_position[1]);
            offsets[2] = offsets[2]-scaling_factors[2]*(normalized_falcon[2]-frozen_position[2]);
            *reset_center = false;
        }

        // When not clutching, command based on the actual falcon position
        panda_pos[0] = scaling_factors[0] * normalized_falcon[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * normalized_falcon[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * normalized_falcon[2] + offsets[2];
        panda_pos[6] = 1;

        // For recording
        x = panda_pos[0];
        y = panda_pos[1];
        z = panda_pos[2];
    }
    
    publishPose(pose_goal_pub, panda_pos);
}

void feedbackFalcon(geometry_msgs::Wrench wrench) {
    double scale = 0.1; // force reflection
    double viscous = 50; // friction
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    double delta_T = 0.001;

    if(!replay_active){
        array<double, 3> falconPos = {0,0,0};
        m_falconDevice.runIOLoop();
        falconPos = m_falconDevice.getPosition();
        // Send force (bilateral + friction) to the falcon
        m_falconDevice.setForce({
                -wrench.force.x * scale-viscous*falcon_vel[0], 
                -wrench.force.z * scale-viscous*falcon_vel[1], 
                -wrench.force.y * scale-viscous*falcon_vel[2]});
    }

    else{
        // zero displacement running in other process

        //  // zero displacement mode
        // // falcon has offset in z
        // m_falconDevice.setForce({
        //         -stiffness*falconPos[0]-viscous_replay*falcon_vel[0], 
        //         -stiffness*falconPos[1]-viscous_replay*falcon_vel[1], 
        //         -stiffness*(falconPos[2]-0.1)-viscous_replay*falcon_vel[2]});
    }

    // For recording and hybrid replay
    fx = wrench.force.x;
    fy = wrench.force.y;
    fz = wrench.force.z;

}

void actualPose(geometry_msgs::Pose pose) {
    actual_pos[0] = pose.position.x;
    actual_pos[1] = pose.position.y;
    actual_pos[2] = pose.position.z;
}

int main(int argc, char **argv) {    
    ros::init(argc, argv, "Falcon");
    ros::NodeHandle n("~");  
    std::vector<double> scaling_factors = {-6.0, 6.0, 6.0};
    std::vector<double> offsets = {-0.249, -0.6, 1.1084};
    //n.getParam("offsets",offsets);
    //n.getParam("scaling_factors",scaling_factors);

    if (!init_falcon()) {
        cout << "Failed to init falcon" << endl;
        return -1;
    }
    
    ros::Publisher pose_goal_pub = 
        n.advertise<geometry_msgs::Pose>("/panda/ee_pose_goals", 5);

    ros::Publisher command_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackFalcon);
    ros::Subscriber actual_pose_sub = n.subscribe("/panda/pose_actual", 10, actualPose);
    

    ros::Publisher file_pub = 
        n.advertise<std_msgs::String>("/dmp/filepub", 5);
    ros::Publisher reset_pub = 
        n.advertise<std_msgs::String>("/dmp/reset", 5);

    ros::Publisher gripper_toggle = 
        n.advertise<std_msgs::String>("/gripperToggle", 1);


    bool last_buttons[4] = {false, false, false, false};
    bool clutch = false;
    bool reset_center = false;
    bool freeze = false;
    bool recording = false;
    int file_iter = 0;
    bool replay_mode = false;
    
    bool timing_press = false;
    chrono::time_point<chrono::steady_clock> start_timer;
    chrono::steady_clock sc;

    while(ros::ok() && (!replay_mode)){      
        // Check button presses
        array<bool,4> buttons = getButtons();
        
        if(buttons[0]!=last_buttons[0]){
            // Clutching functionality
            if(buttons[0]==1){
                clutch = true;
                freeze = true;
                cout << "Clutch On" << endl;
            }

            else{
                clutch = false;
                reset_center = true;
                cout << "Clutch Off" << endl;
            }
        }

        if(buttons[1]!=last_buttons[1]){
            cout << "Mode!" << endl;
            replay_mode = true;
        }

        if(buttons[2]!=last_buttons[2]){
            if(buttons[2]==true){
                gripper_toggle.publish(to_string(1));
            }
        }

        if(buttons[3]!=last_buttons[3]){
            if(buttons[3]==true){
                if(recording==false){
                    // Short press is record - Long press is reset
                    timing_press = true;
                    start_timer = sc.now();
                }
                else{
                    outputfile.close();
                    cout << "Ending Recording" << endl;
                    recording=false;
                    // Let ROS know a file has been published to update the DMP
                    file_pub.publish("panda_demo_"+to_string(file_iter-1)+".csv");
                }
            }

            else{
                if(timing_press)
                {
                    timing_press = false;
                    // Get amount of time button was held
                    auto end_timer = sc.now();
                    auto time_span = static_cast<chrono::duration<double>>(end_timer - start_timer);
                    if(time_span.count()>0.4) // long hold is erase
                    {
                        cout << "Reset DMP!" << endl;
                        reset_pub.publish(to_string(1));
                    }
                    else{ // Short hold is record
                         string filename = {"panda_demo_"+to_string(file_iter)+".csv"};
                        remove( filename.c_str() );
                        outputfile.open (filename.c_str());
                        cout << "Starting Recording: " << filename.c_str() <<  endl;
                        file_iter++;
                        recording=true;
                    }
                }
            }
        }

        last_buttons[0] = buttons[0];
        last_buttons[1] = buttons[1];
        last_buttons[2] = buttons[2];
        last_buttons[3] = buttons[3];

        pollFalcon(pose_goal_pub,command_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze);
        falconVelocity();

        if (recording){
            log_demonstration(x,y,z,fx,fy,fz);
        }

        ros::spinOnce();
        usleep(1000);   
    }

    // Switch into replay mode
    // TODO: switch this to a loop and something that can go back and forth
    replay_active = true;
    bool quit_replay = false;

    // Thread a process for the zero-displacement falcon controller!
    std::thread t1(run_falcon_deformation_controller);

    while(ros::ok() && !quit_replay)
    {
        array<bool,4> buttons = getButtons();
        if(buttons[0]!=last_buttons[0]){
            // Run Replay
            if(buttons[0]==1){
                cout << "Running the replay." << endl;
                replay_demo(pose_goal_pub, n);
            }
        }

        if(buttons[1]!=last_buttons[1]){
            // Quit
            if(buttons[1]==1){
                quit_replay=true;
            }
        }

        last_buttons[0] = buttons[0];
        last_buttons[1] = buttons[1];
        last_buttons[2] = buttons[2];
        last_buttons[3] = buttons[3];
        
        usleep(1000);
        ros::spinOnce();
    }
    
    m_falconDevice.close();
    return 0;
}
