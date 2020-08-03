"""Preprocesses tongs data for constraint recognition

 Last Updated: 02/22/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

import numpy as np
from scipy import interpolate
from algorithms.quaternion import quaternion_multiply, quaternion_flip, rotq, filter_quaternions, quaternion_inverse
from algorithms.filters import butter_lowpass_filter
from algorithms.quaternion import qtoA,diff0, get_angular_velocity

from tf.transformations import quaternion_from_euler # ROS Libraries
from tf.transformations import quaternion_multiply as qm_single
from tf.transformations import quaternion_slerp
from tf.transformations import quaternion_matrix,translation_matrix,quaternion_from_matrix
from algorithms.transform import transform_coord_frame

#Nonvetted Imports
from ros_data_type_helpers import wrench_to_np_array, rotate_vec,pose_array_to_np_array, q_rotate_vec

def preProcessTongs(bfr,bfr_zeros = None,cutoff = 10.0):
    """
    Pulls tongs data from bagfile, combines the two signals, filters signals, and interpolates all arrays to common length

    :param bfr: bagfile with demonstration data
    :param cutoff: break frequency of filter
    :type bfr: bagfile object
    :type cutoff: float
    :returns: outstruct
    :rtype: struct with pose, and FT data
    """

    #Tongs Topic Configuration
    ftuppermsgname = "/ftmini40"
    ftlowermsgname = "/ftmini402"
    uppermocapname = "/vrpn_client_node/UpperTongs/pose"
    lowermocapname = "/vrpn_client_node/LowerTongs/pose"

    # get forces, torques, positions from bag file
    wrench_v_upper,wrench_t_upper = bfr.get_topic_msgs(ftuppermsgname)
    wrench_v_lower, wrench_t_lower = bfr.get_topic_msgs(ftlowermsgname)

    ##############################
    if bfr_zeros is not None:
        wrench_v_upper_z, _ = bfr_zeros.get_topic_msgs(ftuppermsgname)
        wrench_v_lower_z, _ = bfr_zeros.get_topic_msgs(ftlowermsgname)
        w_arr_u_z = wrench_to_np_array(wrench_v_upper_z)
        w_arr_l_z = wrench_to_np_array(wrench_v_lower_z)
        w_arr_u_z = np.average(w_arr_u_z,axis = 0)
        w_arr_l_z = np.average(w_arr_l_z,axis = 0)
    else:
        w_arr_u_z = np.zeros(6)
        w_arr_l_z = np.zeros(6)
    ##############################



    p_v_upper, timesamples_upper = bfr.get_topic_msgs(uppermocapname)
    p_v_lower, timesamples_lower = bfr.get_topic_msgs(lowermocapname)

    # Sample times constructed by diffing the timestamp datasets
    wrench_sample_time = np.mean(np.diff(wrench_t_upper))*1.0 #both have same sample time

    mocap_sample_time = np.mean(np.diff(timesamples_upper))*1.0 #both have same sample time

    #convert to NP array for consistent data format
    wrenches_sensor_values_upper = wrench_to_np_array(wrench_v_upper)
    wrenches_sensor_values_lower = wrench_to_np_array(wrench_v_lower)

    wrenches_sensor_values_upper = wrenches_sensor_values_upper - w_arr_u_z[np.newaxis,:]
    wrenches_sensor_values_lower = wrenches_sensor_values_lower - w_arr_l_z[np.newaxis,:]

    #low-pass filter the forces/torques
    wrenches_sensor_values_upper = butter_lowpass_filter(wrenches_sensor_values_upper,
                                                   mocap_sample_time/wrench_sample_time,
                                                   1.0/wrench_sample_time, order=4, axis=0)

    wrenches_sensor_values_lower = butter_lowpass_filter(wrenches_sensor_values_lower,
                                                   mocap_sample_time/wrench_sample_time,
                                                   1.0/wrench_sample_time, order=4, axis=0)


    #Interpolate all datasets to the most sparse dataset (one of the two motion capture feeds)
    if(len(timesamples_upper)<len(timesamples_lower)):
        # interpolate wrench to the slower motion capture rate
        wrenches_interp_upper = interpolate.interp1d(wrench_t_upper, wrenches_sensor_values_upper, axis = 0, fill_value="extrapolate")(timesamples_upper)
        wrenches_interp_lower = interpolate.interp1d(wrench_t_lower, wrenches_sensor_values_lower, axis=0, fill_value="extrapolate")(timesamples_upper)
        p_array_lower, q_array_lower = pose_array_to_np_array(p_v_lower)
        p_array_upper, q_array_upper = pose_array_to_np_array(p_v_upper)
        p_array_lower = interpolate.interp1d(timesamples_lower, p_array_lower, axis=0, fill_value="extrapolate")(timesamples_upper)
        q_array_lower = interpolate.interp1d(timesamples_lower, q_array_lower, axis=0, fill_value="extrapolate")(
            timesamples_upper)
        timesamples=timesamples_upper
    else:
        wrenches_interp_upper = interpolate.interp1d(wrench_t_upper, wrenches_sensor_values_upper, axis=0, fill_value="extrapolate")(
            timesamples_lower)
        wrenches_interp_lower = interpolate.interp1d(wrench_t_lower, wrenches_sensor_values_lower, axis=0, fill_value="extrapolate")(
            timesamples_lower)
        p_array_lower, q_array_lower = pose_array_to_np_array(p_v_lower)
        p_array_upper, q_array_upper = pose_array_to_np_array(p_v_upper)
        p_array_upper = interpolate.interp1d(timesamples_upper, p_array_upper, axis=0, fill_value="extrapolate")(
            timesamples_lower)
        q_array_upper = interpolate.interp1d(timesamples_upper, q_array_upper,axis=0, fill_value="extrapolate")(
            timesamples_lower)
        timesamples=timesamples_lower

    #Split data into forces and torques
    calibrated_forces_upper = wrenches_interp_upper[:,:3]
    calibrated_torques_upper = wrenches_interp_upper[:,3:]
    calibrated_forces_lower = wrenches_interp_lower[:,:3]
    calibrated_torques_lower = wrenches_interp_lower[:,3:]

    # Transform the force and torques into the motion capture frame
    calibrated_forces_global_upper = np.array([rotate_vec(p, f) for p, f in zip(p_v_upper, calibrated_forces_upper)])
    calibrated_torques_global_upper = np.array([rotate_vec(p, t) for p, t in zip(p_v_upper, calibrated_torques_upper)])
    calibrated_forces_global_lower = np.array([rotate_vec(p, f) for p, f in zip(p_v_lower, calibrated_forces_lower)])
    calibrated_torques_global_lower = np.array([rotate_vec(p, t) for p, t in zip(p_v_lower, calibrated_torques_lower)])

    # Find center point of tongs

    p_array_upper_x=np.zeros(np.shape(p_array_upper))
    p_array_upper_y=np.zeros(np.shape(p_array_upper))
    p_array_upper_z=np.zeros(np.shape(p_array_upper))

    for i,(p,q) in enumerate(zip(p_array_upper,q_array_upper)):
        mocap_tong_upper = quaternion_matrix(q)
        mocap_tong_upper[:3,3] = np.reshape(p[np.newaxis].T,(3,))

        x = np.reshape(np.matmul(mocap_tong_upper,[[0.1],[0],[0],[1]]),(4))
        y = np.reshape(np.matmul(mocap_tong_upper,[[0],[0.1],[0],[1]]),(4,))
        z = np.reshape(np.matmul(mocap_tong_upper,[[0],[0],[0.1],[1]]),(4,))

        p_array_upper_x[i] = x[:3]
        p_array_upper_y[i] = y[:3]
        p_array_upper_z[i] = z[:3]

    p_array_lower_x=np.zeros(np.shape(p_array_lower))
    p_array_lower_y=np.zeros(np.shape(p_array_lower))
    p_array_lower_z=np.zeros(np.shape(p_array_lower))

    for i,(p,q) in enumerate(zip(p_array_lower,q_array_lower)):

        mocap_tong_lower = quaternion_matrix(q)
        mocap_tong_lower[:3,3] = np.reshape(p[np.newaxis].T,(3,))

        x = np.reshape(np.matmul(mocap_tong_lower,[[0.1],[0],[0],[1]]),(4))
        y = np.reshape(np.matmul(mocap_tong_lower,[[0],[0.1],[0],[1]]),(4,))
        z = np.reshape(np.matmul(mocap_tong_lower,[[0],[0],[0.1],[1]]),(4,))

        p_array_lower_x[i] = x[:3]
        p_array_lower_y[i] = y[:3]
        p_array_lower_z[i] = z[:3]

    p_array_center,q_array_center = get_center_tongs(p_array_lower,q_array_lower,p_array_upper,q_array_upper)

    p_array_center_x,q_array_center_x = get_center_tongs(p_array_lower_x,q_array_lower,p_array_upper_x,q_array_upper)
    p_array_center_y,q_array_center_y = get_center_tongs(p_array_lower_y,q_array_lower,p_array_upper_y,q_array_upper)
    p_array_center_z,q_array_center_z = get_center_tongs(p_array_lower_z,q_array_lower,p_array_upper_z,q_array_upper)
    ##########################################################################################################################
    # Compute the net force and torque as per equation 2.66 in Murray (A Mathematical Introduction to Robotic Manipulation)  #
    # RbcT is wrong - should be Rbc
    ##########################################################################################################################

    calibrated_forces_sum_local = np.array([fl + fu
                                      for fl, fu in zip(calibrated_forces_lower,
                                                        calibrated_forces_upper)])

    #Force and torque transform using transpose of adjoint transformation matrix
    calibrated_forces_sum = np.array([fl + fu
                                      for fl, fu in zip(calibrated_forces_global_lower,
                                                        calibrated_forces_global_upper)])
    # TODO add moment transforms document
    calibrated_torques_sum = np.array([np.cross((pu - p),fu) + np.cross((pl - p),fl) + tl + tu
                                       for pu,pl,p, qu,ql,q,fu,fl,tu,tl in zip(p_array_upper,p_array_lower,p_array_center,
                                                                               q_array_upper,q_array_lower,q_array_center,
                                                                               calibrated_forces_global_upper,
                                                                               calibrated_forces_global_lower,
                                                                            calibrated_torques_global_upper,
                                                                            calibrated_torques_global_lower)])



    q_array_center = quaternion_flip(q_array_center)

    # Optitrack has Y-axis as the identity so all matrices are rotated by positive pi/2 about x
    q_offset = np.array([rotq([1,0,0],np.pi/2)])
    q_offset = np.repeat(q_offset,len(q_array_center),axis = 0)
    q_array_center = quaternion_multiply(q_array_center,q_offset)

    # filtering position and rotation data
    p_array_center = butter_lowpass_filter(p_array_center,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array_center = filter_quaternions(q_array_center,cutOff=cutoff,fs=1.0/mocap_sample_time)

    #Rotate torques into the local frame -> for formulation in derive constraint equations (required)
    calibrated_torques_local = np.array([np.matmul(np.transpose(qtoA(q)),t) for q,t in zip(q_array_center,calibrated_torques_sum)])

    w_array_center = get_angular_velocity(q_array_center)
    w_array_center_local = get_angular_velocity(q_array_center,fixed_frame = False)
    v_array_center = diff0(p_array_center)
    # removing forces in direction of motion (approximately removing friction forces)
    v_array_hat = v_array_center/np.linalg.norm(v_array_center,axis = 1)[:,np.newaxis]
    sudo_reaction_forces = calibrated_forces_sum - np.sum(calibrated_forces_sum*v_array_hat,axis=1)[:,np.newaxis]*v_array_hat
    w_array_hat = w_array_center/np.linalg.norm(w_array_center,axis = 1)[:,np.newaxis]
    sudo_reaction_torques_global = calibrated_torques_sum \
                                   - np.sum(calibrated_torques_sum * v_array_hat, axis=1)[:, np.newaxis] * v_array_hat
    sudo_reaction_torques_local = np.array(
        [np.matmul(np.transpose(qtoA(q)), t) for q, t in zip(q_array_center, sudo_reaction_torques_global)])

    #Return data to workspace
    out_struct = {}
    out_struct["timestamps"] = timesamples
    out_struct["f"] = calibrated_forces_sum
    out_struct["f_local"] = calibrated_forces_sum_local
    out_struct["t_local"] = calibrated_torques_local # in local frame
    out_struct["t_global"] = calibrated_torques_sum # in global frame
    out_struct["f_n"] = sudo_reaction_forces
    out_struct["t_n_global"] = sudo_reaction_torques_global# in global frame
    out_struct["t_n_local"] = sudo_reaction_torques_local# in local frame

    out_struct["p"] = p_array_center # position of tongs
    out_struct["q"] = q_array_center # orientation of tongs

    out_struct["w_global"] = w_array_center # angular velocity global frame
    out_struct["w_local"] = w_array_center_local # angular velocity local frame
    out_struct["v"] = v_array_center # linear velocity

    out_struct["f_u"] = calibrated_forces_global_upper
    out_struct["f_u_local"]=calibrated_forces_upper
    out_struct["f_l"] = calibrated_forces_global_lower
    out_struct["f_l_local"]=calibrated_forces_lower

    out_struct["t_u"] = calibrated_torques_global_upper
    out_struct["t_l"] = calibrated_torques_global_lower

    out_struct["p_x"] = p_array_center_x
    out_struct["q_x"] = q_array_center_x

    out_struct["p_y"] = p_array_center_y
    out_struct["q_y"] = q_array_center_y

    out_struct["p_z"] = p_array_center_z
    out_struct["q_z"] = q_array_center_z

    # Adding labels if they exist in bagfile
    if '/labels' in bfr.topics:
        labels, label_timestamps = bfr.get_topic_msgs('/labels')
        labels = np.array([label.data for label in labels], dtype='S100')
        still_on = (labels == 'constraint').astype(float)
        labels_interp = interpolate.interp1d(label_timestamps, still_on, kind='nearest', fill_value='extrapolate')(
            timesamples)
        labels_interp = np.squeeze([labels_interp > 0.5]).astype(bool)
        out_struct['labels'] = labels_interp


    return out_struct

def preProcessFrame(bfr,cutoff = 10.0):
    """
    Pulls tongs data from bagfile, combines the two signals, filters signals, and interpolates all arrays to common length

    :param bfr: bagfile with demonstration data
    :param cutoff: break frequency of filter
    :type bfr: bagfile object
    :type cutoff: float
    :returns: outstruct
    :rtype: struct with pose, and FT data
    """

    #Tongs Topic Configuration
    uppermocapname = "/vrpn_client_node/RigidBody01/pose"

    p_v_upper, timesamples_upper = bfr.get_topic_msgs(uppermocapname)

    mocap_sample_time = np.mean(np.diff(timesamples_upper))*1.0 #both have same sample time

    p_array, q_array= pose_array_to_np_array(p_v_upper)


    q_array = quaternion_flip(q_array)

    # Optitrack has Y-axis as the identity so all matrices are rotated by positive pi/2 about x
    q_offset = np.array([rotq([1,0,0],np.pi/2)])
    q_offset = np.repeat(q_offset,len(q_array),axis = 0)
    q_array = quaternion_multiply(q_array,q_offset)

    # filtering position and rotation data
    p_array = butter_lowpass_filter(p_array,cutoff,1.0/mocap_sample_time,axis = 0)
    q_array = filter_quaternions(q_array,cutOff=cutoff,fs=1.0/mocap_sample_time)

    #Rotate torques into the local frame -> for formulation in derive constraint equations (required)
   #Return data to workspace
    out_struct = {}
    out_struct["timestamps"] = timesamples_upper

    out_struct["p"] = p_array
    out_struct["q"] = q_array


    return out_struct

def get_center_tongs(p_lower,q_lower,p_upper,q_upper):

    q_center = []
    q_offsetu = quaternion_from_euler(0,np.pi,-np.pi*22.5/180 + np.pi) # Rotating sensor at mount to align with forward direction
    q_offsetl = quaternion_from_euler(0,0,-np.pi*22.5/180)
    q_offsetc = qm_single(quaternion_from_euler(0,np.pi/2,0),
                                    quaternion_from_euler(0,0,np.pi/2)) # This worked

    for qlq,quq in zip(q_lower,q_upper):
        qlq = qm_single(qlq,q_offsetl)
        quq = qm_single(quq,q_offsetu)
        qcq = quaternion_slerp(qlq,quq,0.5)
        qcq = qm_single(qcq,q_offsetc)
        q_center.append(qcq)
    q_center = np.array(q_center)
    p_center = (p_upper + p_lower)/2
    return p_center,q_center
