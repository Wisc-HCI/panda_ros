#!/usr/bin/env python
import numpy as np
import PyBSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def printPathSection(csvfile,starting_point,ending_point,num_pts):

    # Set up interpolation of the quaternions
    key_inds = [0, num_pts-1];
    key_rots = R.from_quat(np.array([[starting_point[3], starting_point[4], starting_point[5], starting_point[6]],[ending_point[3], ending_point[4], ending_point[5], ending_point[6]]]))
    slerp = Slerp(key_inds, key_rots)

    for ii in range(0,num_pts):
        # Position
        interp_x = starting_point[0]+(ending_point[0]-starting_point[0])*(float(ii)/float(num_pts-1))
        interp_y = starting_point[1]+(ending_point[1]-starting_point[1])*(float(ii)/float(num_pts-1))
        interp_z = starting_point[2]+(ending_point[2]-starting_point[2])*(float(ii)/float(num_pts-1))

        # Orientation
        temp_q = slerp([ii])
        interp_qx = temp_q.as_quat()[0][0]
        interp_qy = temp_q.as_quat()[0][1]
        interp_qz = temp_q.as_quat()[0][2]
        interp_qw = temp_q.as_quat()[0][3]

        # Forces
        interp_fx = starting_point[7]+(ending_point[7]-starting_point[7])*(float(ii)/float(num_pts-1))
        interp_fy = starting_point[8]+(ending_point[8]-starting_point[8])*(float(ii)/float(num_pts-1))
        interp_fz = starting_point[9]+(ending_point[9]-starting_point[9])*(float(ii)/float(num_pts-1))

        # Torques
        interp_tx = starting_point[10] + (ending_point[10] - starting_point[10]) * (float(ii) / float(num_pts - 1))
        interp_ty = starting_point[11] + (ending_point[11] - starting_point[11]) * (float(ii) / float(num_pts - 1))
        interp_tz = starting_point[12] + (ending_point[12] - starting_point[12]) * (float(ii) / float(num_pts - 1))
       
        csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+','
                      + str(interp_qx) + ',' + str(interp_qy) + ',' + str(interp_qz) + ',' + str(interp_qw) + ','
                      + str(interp_fx)+','+str(interp_fy)+','+str(interp_fz) + ','
                      + str(interp_tx) + ',' + str(interp_ty) + ',' + str(interp_tz))
        csvfile.write('\n')


def calculateQuaternion(normal_vector, x, y):
    R_new = np.eye(3)
    R_new[:,0]=x.reshape((3,))
    R_new[:,1]=y.reshape((3,))
    R_new[:,2]=normal_vector.reshape((3,))

    r = R.from_dcm(R_new)
    q_new = r.as_quat() # x,y,z,w
    qx = q_new[0]
    qy = q_new[1]
    qz = q_new[2]
    qw = q_new[3]

    return qx,qy,qz,qw

def main():
    num_pts = 100
    # with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/painting.csv', 'w') as csvfile:

    #########################################################
    ############# COMPOSITE LAYUP ###########################
    #########################################################
    # with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/composite.csv', 'w') as csvfile:
    #
    #     # Note the goal points are specified in parameterized coordinates if it is a contact event
    #
    #     surfaceModel = PyBSpline.BSplineSurface()
    #     surfaceModel.loadSurface("curved")
    #
    #     # csvfile.write(",curved,,,curved,,,curved,")
    #     csvfile.write(",curved,,,curved,")
    #     csvfile.write('\n')
    #     csvfile.write("0,100,200,300,400,500")
    #     csvfile.write('\n')
    #     csvfile.write("1,0,1,1,0,1")
    #     csvfile.write('\n')
    #     csvfile.write("2 2 2,3 10 150,2 2 2,2 2 2,3 10 150,2 2 2")
    #     csvfile.write('\n')
    #
    #     # Format for path section is [rx,ry,rz, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #
    #     ########################
    #     ### PASS 1 #############
    #     ########################
    #     surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.3, 0.5)
    #     starting_point = surface_start + normal_start*0.3
    #
    #     print("SS:",surface_start)
    #     # starting_vel = r_u * 0.8 + r_v * 0.2
    #     starting_vel = r_u * 1.0 + r_v * 0.0
    #     starting_vel = starting_vel / np.linalg.norm(starting_vel)
    #     starting_y = np.cross(normal_start, starting_vel)
    #     qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)
    #
    #     printPathSection(csvfile, np.array([starting_point[0], starting_point[1], starting_point[2], 0, 0, 0, 1, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array([surface_start[0], surface_start[1], surface_start[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),num_pts)
    #
    #     # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     printPathSection(csvfile, np.array([0.3, 0.5, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.9, 0.5, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),num_pts)
    #
    #     surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.9, 0.5)
    #     ending_vel = r_u * 1.0 + r_v * 0.0
    #     ending_vel = ending_vel / np.linalg.norm(ending_vel)
    #     ending_y = np.cross(normal_end, ending_vel)
    #     qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)
    #
    #     # From pass 2 - try to fix orientation during kinematic motion
    #     surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.1, 0.7)
    #     starting_vel = r_u * 1.0 + r_v * 0.0
    #     starting_vel = starting_vel / np.linalg.norm(starting_vel)
    #     starting_y = np.cross(normal_start, starting_vel)
    #     qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)
    #
    #     printPathSection(csvfile, np.array([surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.1, -0.1, 1.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),num_pts)
    #
    #     ########################
    #     ### PASS 2 #############
    #     ########################
    #
    #     printPathSection(csvfile, np.array([0.1, -0.1, 1.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array(
    #                          [surface_start[0], surface_start[1], surface_start[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0,
    #                           0.0, 0.0, 0.0]), num_pts)
    #
    #     # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     printPathSection(csvfile, np.array([0.1, 0.7, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.9, 0.7, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), num_pts)
    #
    #     surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.9, 0.7)
    #     ending_vel = r_u * 1.0 + r_v * 0.0
    #     ending_vel = ending_vel / np.linalg.norm(ending_vel)
    #     ending_y = np.cross(normal_end, ending_vel)
    #     qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)
    #
    #     printPathSection(csvfile, np.array(
    #         [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.0, -0.1, 1.1, qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), num_pts)
    #
    #     # ########################
    #     # ### PASS 3 #############
    #     # ########################
    #     # surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.1, 0.7)
    #     # starting_vel = r_u * 1.0 + r_v * 0.0
    #     # starting_vel = starting_vel / np.linalg.norm(starting_vel)
    #     # starting_y = np.cross(normal_start, starting_vel)
    #     # qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)
    #     #
    #     # printPathSection(csvfile, np.array([0.1, -0.1, 1.0, qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #     #                  np.array(
    #     #                      [surface_start[0], surface_start[1], surface_start[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0,
    #     #                       0.0, 0.0, 0.0]), num_pts)
    #     #
    #     # # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     # printPathSection(csvfile, np.array([0.1, 0.7, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #     #                  np.array([0.9, 0.7, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), num_pts)
    #     #
    #     # surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.1, 0.7)
    #     # ending_vel = r_u * 1.0 + r_v * 0.0
    #     # ending_vel = ending_vel / np.linalg.norm(ending_vel)
    #     # ending_y = np.cross(normal_end, ending_vel)
    #     # qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)
    #     #
    #     # printPathSection(csvfile, np.array(
    #     #     [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]),
    #     #                  np.array([-0.1, 0.0, 1.2, qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), num_pts)



    ##########################################################
    #   Layup 1 TASK #########################################
    ##########################################################
    with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/layup1_execution.csv', 'w') as csvfile:

        surfaceModel = PyBSpline.BSplineSurface()
        surfaceModel.loadSurface("layup1")

        csvfile.write(",layup1,")
        csvfile.write('\n')
        csvfile.write("0,100,200")
        csvfile.write('\n')
        csvfile.write("1,0,1")
        csvfile.write('\n')
        # csvfile.write("2 2 2,3 10 150,2 2 2")
        csvfile.write("0 0 0,0 0 0,0 0 0")
        csvfile.write('\n')

        surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.8, 0.8)

        starting_vel = r_u * -1.0 + r_v * 0.0
        starting_vel = starting_vel / np.linalg.norm(starting_vel)
        starting_y = np.cross(normal_start, starting_vel)
        qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)


        printPathSection(csvfile, np.array(
            [0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array(
                             [surface_start[0], surface_start[1], surface_start[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0,
                              0.0, 0.0, 0.0]), num_pts)

        # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
        printPathSection(csvfile, np.array([0.8, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.2, 0.8, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

        surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.2, 0.8)
        ending_vel = r_u * -1.0 + r_v* 0.0
        ending_vel = ending_vel / np.linalg.norm(ending_vel)
        ending_y = np.cross(normal_end, ending_vel)
        qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)

        printPathSection(csvfile, np.array(
            [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
                         np.array([0.50, 0.0, 0.25, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)

    # ##########################################################
    # ############## WRITING TASK ##############################
    # ##########################################################
    # with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/writing.csv', 'w') as csvfile:
    #
    #     surfaceModel = PyBSpline.BSplineSurface()
    #     surfaceModel.loadSurface("table")
    #
    #     csvfile.write(",table,table,table,")
    #     csvfile.write('\n')
    #     csvfile.write("0,100,200,300,400")
    #     csvfile.write('\n')
    #     csvfile.write("1,0,0,0,1")
    #     csvfile.write('\n')
    #     csvfile.write("2 2 2,3 10 150,3 10 150,3 10 150,2 2 2")
    #     # csvfile.write("0 0 0,0 0 0,0 0 0,0 0 0,0 0 0")
    #     csvfile.write('\n')
    #
    #     surface_start, normal_start, r_u, r_v = surfaceModel.calculate_surface_point(0.95, 0.17)
    #
    #     starting_vel = r_u * -1.0 + r_v * 0.0
    #     starting_vel = starting_vel / np.linalg.norm(starting_vel)
    #     starting_y = np.cross(normal_start, starting_vel)
    #     qx_s, qy_s, qz_s, qw_s = calculateQuaternion(normal_start, starting_vel, starting_y)
    #
    #
    #     printPathSection(csvfile, np.array(
    #         [-0.6, 0.078, 1.2, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
    #                      np.array(
    #                          [surface_start[0], surface_start[1], surface_start[2], qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0,
    #                           0.0, 0.0, 0.0]), num_pts)
    #
    #     # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     printPathSection(csvfile, np.array([0.95, 0.17, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.085, 0.17, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)
    #
    #     # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     printPathSection(csvfile, np.array([0.085, 0.17, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.085, 0.73, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)
    #
    #     # Format for path section is [ru,rv,null, qx, qy, qz, qw, fx, fy, fz, tx, ty, tz]
    #     printPathSection(csvfile, np.array([0.085, 0.73, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
    #                      np.array([0.95, 0.73, 0.0, qx_s, qy_s, qz_s, qw_s, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)
    #
    #     surface_end, normal_end, r_u, r_v = surfaceModel.calculate_surface_point(0.95, 0.72)
    #     ending_vel = r_u * 1.0 + r_v* 0.0
    #     ending_vel = ending_vel / np.linalg.norm(ending_vel)
    #     ending_y = np.cross(normal_end, ending_vel)
    #     qx_e, qy_e, qz_e, qw_e = calculateQuaternion(normal_end, ending_vel, ending_y)
    #
    #     printPathSection(csvfile, np.array(
    #         [surface_end[0], surface_end[1], surface_end[2], qx_e, qy_e, qz_e, qw_e, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]),
    #                      np.array([-0.6, 0.078, 1.2, 0, 0, 0, 1, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0]), num_pts)


        # # PAINTING - TODO: UPDATE TO WORK WITH SURFACE MODEL
        # csvfile.write("0,100,200,300,400")
        # csvfile.write('\n')
        # csvfile.write("1,1,1,1,1")
        # csvfile.write('\n')
        # printPathSection(csvfile,np.array([0.2, 0.1, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, 0.1, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([-0.5, 0.1, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, -0.05, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([-0.5, -0.05, 1.1, 0.0, 0.0, 0.0]),np.array([0.2, -0.05, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([0.2, -0.05, 1.1, 0.0, 0.0, 0.0]),np.array([0.2, -0.2, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([0.2, -0.2, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, -0.2, 1.1, 0.0, 0.0, 0.0]),num_pts)


if __name__ == "__main__":
    main()




