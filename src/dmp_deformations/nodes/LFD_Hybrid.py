#!/usr/bin/env python

""" Using a series of hand-done demonstrations,
compute the transformed and segmented data for
the DMP learning
 Created: 06/22/2020
"""

__author__ = "Mike Hagenow"

from LFD_helpers.preProcessTongs import preProcessTongs
from LFD_helper.bagfile_reader import bagfile_reader


def getSegmentation(demo):
    # Go through demonstration and look for contact, un-contact events
    # The data has been downsampled to 50 Hz
    number_samples_contact = 10
    number_samples_not_contact = 10
    force_threshold = 5.0

    states = []

    in_contact = False
    states.append((0,np.array([1, 1, 1])))

    # print "LENGTH"
    # print np.shape(demo)

    # 2nd order filter
    b, a = signal.butter(2, 1.0,fs=50)
    y = signal.filtfilt(b, a, np.linalg.norm(demo[:,3:],axis=1), padlen=0)
    y = y-np.min(y)
    # import matplotlib.pyplot as plt
    # plt.plot(np.linalg.norm(demo[:,3:],axis=1))
    # plt.plot(y)
    # plt.show()

    for ii in range(1,len(demo)):
        # Add the position control event
        if not in_contact and np.all(y[ii:ii+number_samples_contact]>force_threshold):
            states.append((ii,np.array([1, 1, 0])))
            in_contact = True
        # Add the force start control event
        if in_contact and np.all(y[ii:ii+number_samples_not_contact]<force_threshold): # and norm whatever
            states.append((ii,np.array([1, 1, 1])))
            # add constraint frame eventually
            in_contact = False

    return states



# Based on a transform from the rigid registration,
# rotate all of the recorded position, orientation, and force data
def rotate_data(transform_r,transform_q,r,q,f):
    return r,q,f




def learn_from_demonstrations():

    # Specify the files where the demonstrations are located. There are bag files
    # where all of the raw data has been recorded from the instrument
    # specify the full filenames (e.g. files = ['/home/mike/desktop....','/home/mike/desktop...']

    files = ['/home/mike/desktop/test.bag','/home/mike/desktop/test2.bag']

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

        r_rotated, q_rotated, f_rotated = rotate()



    # Next the demonstrations are segmented based on contact events (i.e., filter the forces
    # to estimate when hybrid control should be enacted)
    # Note: this is done based on the first demonstration

    segmented_inds = segmentyo(the thing)

    # Prepare the data for DMP learning
    # if it not a contact segment, feed the kinematic data
    # if it is a contact segment, need to convert to parameterized representation

    # How to deal with DTW?!

def segmentation(file):
    # here is what segmentation should look like
    # segmentation.append((point, temp_var, np.array([1, 1, 1, 1, 1, 1, 1]), surface_model))


def main():
    learn_from_demonstrations()

if __name__ == "__main__":
    main()




