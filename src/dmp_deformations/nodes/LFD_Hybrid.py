#!/usr/bin/env python

""" Using a series of hand-done demonstrations,
compute the transformed and segmented data for
the DMP learning
 Created: 06/22/2020
"""

__author__ = "Mike Hagenow"

import numpy as np

from LFD_helpers.preProcessTongs import preProcessTongs
from LFD_helpers.bagfile_reader import bagfile_reader
from LFD_helpers.algorithms.transform import transform_coord_frame
from scipy import signal
from dtw import dtw
import PyBSpline
from scipy.optimize import minimize
from DMP_learner import calculateDMP


def obj_closest_surface(x,surface,xf,yf,zf):
    r, n_hat, r_u_norm, r_v_norm = surface.calculate_surface_point(x[0],x[1])
    return (r[0]-xf)*(r[0]-xf)+(r[1]-yf)*(r[1]-yf)+(r[2]-zf)*(r[2]-zf)

def getClosestParams(x,y,z,u,v,surface):
    x0 = np.array([u, v])
    res = minimize(obj_closest_surface, x0, method='COBYLA',
                   options={'xatol': 1e-8, 'disp': True}, bounds=([0.0, 0.0],[1.0, 1.0]), args=(surface,x,y,z))
    return res.x[0], res.x[1]

def getSegmentation(forces):
    # Go through demonstration and look for contact, un-contact events
    number_samples_contact = 10
    number_samples_not_contact = 10
    force_threshold = 5.0

    states = []

    in_contact = False
    states.append((0,np.array([1, 1, 1, 1, 1, 1, 1])))

    # 2nd order filter
    b, a = signal.butter(2, 1.0,fs=50)

    # TODO: check if axis 1 is correct in this case
    y = signal.filtfilt(b, a, np.linalg.norm(forces,axis=1), padlen=0)
    y = y-np.min(y)


    # import matplotlib.pyplot as plt
    # plt.plot(np.linalg.norm(forces,axis=1))
    # plt.plot(y)
    # plt.show()

    for ii in range(1,len(forces)):
        # Add the position control event
        if not in_contact and np.all(y[ii:ii+number_samples_contact]>force_threshold):
            states.append((ii,np.array([1, 1, 0, 1, 1, 1, 1])))
            in_contact = True
        # Add the force start control event
        if in_contact and np.all(y[ii:ii+number_samples_not_contact]<force_threshold): # and norm whatever
            states.append((ii,np.array([1, 1, 1, 1, 1, 1, 1])))
            # add constraint frame eventually
            in_contact = False

    return states

# Based on a transform from the rigid registration,
# rotate all of the recorded position, orientation, and force data
def rotate_data(transform_r,transform_q,r,q,f):
    r_rotated, q_rotated = transform_coord_frame(r,q,transform_r,transform_q)
    f_rotated, not_used = transform_coord_frame(f,q,transform_r,transform_q)
    return r_rotated,q_rotated,f_rotated

def calculate_alignment_curves(demonstration_data):
    x = []
    for ii in range(0,np.shape(demonstration_data[0])[0]):
        # Features are stored as lists of tuples
        x.append((demonstration_data[0][ii,0],demonstration_data[0][ii,1],demonstration_data[0][ii,2]))

    alignment_curves = []
    for ii in range(0,len(demonstration_data)):
        y = []
        # compare each of the demos to the first which is used for segmentation
        manhattan_distance = lambda x, y: np.abs(x[0] - y[0])+np.abs(x[1] - y[1])+np.abs(x[2] - y[2])

        for jj in range(0, np.shape(demonstration_data[ii])[0]):
            # Features are stored as lists of tuples
            y.append((demonstration_data[ii][jj, 0], demonstration_data[ii][jj, 1], demonstration_data[ii][jj, 2]))

        d, cost_matrix, acc_cost_matrix, path = dtw(x, y, dist=manhattan_distance)
        alignment_curves.append((path[0],path[1]))
    return alignment_curves


# This is the main routine that loads recorded demonstration files
# and the ultimately calls to construct the DMPs for robot replay
def learn_from_demonstrations():
    # Specify the files where the demonstrations are located. There are bag files
    # where all of the raw data has been recorded from the instrument
    # specify the full filenames (e.g. files = ['/home/mike/desktop....','/home/mike/desktop...']

    files = ['/home/mike/desktop/test.bag','/home/mike/desktop/test2.bag']

    # surface model for hybrid interaction (in robot frame)
    surface_file = 'curved.csv'
    surfaceModel = PyBSpline.BSplineSurface()
    surfaceModel.loadSurface(surface_file)

    # Rigid Registration parameters
    mocap_to_robot_r = np.array([0.0, 0.0, 0.0])
    mocap_to_robot_q = np.array([0.0, 0.0, 0.0, 1.0]) # xyzw

    demonstrations = []

    for file in files:
        bfr = bagfile_reader(file)

        # From the raw data, need to get the positions and forces of the end-effector
        # This is done using a specific function that is tailored to the specific instrument

        # Tongs
        procData = preProcessTongs(bfr)
        forces = procData['f']
        pos = procData['p']
        rots = procData['q']

        # # Instrumented rolling pin
        # procData = preProcessRoller(bfr)
        # forces = procData['f']
        # pos = procData['p']
        # rots = procData['q']

        # Rotate into the robot frame. After performing a rigid registration process, there should be an
        # optimal transform (e.g., position and quaternion) that will transform the original demonstration
        # into the frame of the robot

        r_rotated, q_rotated, f_rotated = rotate_data(mocap_to_robot_r,mocap_to_robot_q,pos,rots,forces)

        demonstrations.append(r_rotated,q_rotated,f_rotated)

    # Next the demonstrations are segmented based on contact events (i.e., filter the forces
    # to estimate when hybrid control should be enacted)
    # Note: this is done based on the first demonstration
    segmented_inds = getSegmentation(demonstrations[0][2]) # send in the forces

    # Figure out the optimal alignment for the demonstrations based on DTW
    alignment_curves = calculate_alignment_curves()

    # Prepare the data for DMP learning

    demonstration_data = []
    # prepopulate the demonstration data with the kinematic data (x,y,z,q,f)
    for demo in demonstrations:
        # TODO: need to check once I have data
        demonstration_data.append(np.concatenate(demo[0],demo[1], demo[2]))

    # initial values for u and v so that the previous is used for each next guess in the NLoptimization
    u,v=0,0

    for segment_id in range(0,segmented_inds):
        # Check if contact event
        segmentation_z = segmented_inds[segment_id][1][2]

        # don't need to do anything for position control since kinematics are already in the arrays
        if segmentation_z==0:
            # if it is a contact segment, need to convert to parameterized representation
            for demo_ind in range(0,demonstrations):

                # look through each relative index and compute the closest parameterized surface point
                start_index = alignment_curves[demo_ind][1][
                    int(np.round(np.median(np.where(alignment_curves[demo_ind][0] == segmented_inds[segment_id][0]))))]

                # set the end index appropriately
                if(segment_id+1<len(segmented_inds)):
                    end_index = alignment_curves[demo_ind][1][
                        int(np.round(np.median(np.where(alignment_curves[demo_ind][0] == segmented_inds[segment_id+1][0]))))]
                else:
                    # todo: this might be wrong too
                    end_index = len(demonstrations[demo_ind][0])

                # convert each index... this will take a while
                # todo: add some print statements or something...
                for ii in range(start_index,end_index):
                    # convert to parameterized representation
                    u,v = getClosestParams(demonstration_data[demo_ind][0, ii],demonstration_data[demo_ind][1, ii],demonstration_data[demo_ind][2, ii],u,v,surfaceModel)
                    demonstration_data[demo_ind][0, ii] = u
                    demonstration_data[demo_ind][1, ii] = v
                    demonstration_data[demo_ind][2, ii] = 0.0 # not reqd, but for cleanliness

    # Put all of the required info needed for the segmentation for the DMP calculators!!
    segmentation=[]
    for segment in segmented_inds:
        segmentation.append((segment[0],"",segment[1],surface_file))

    # Output
    calculateDMP(demonstration_data, segmentation, alignment_curves)

def main():
    learn_from_demonstrations()

if __name__ == "__main__":
    main()




