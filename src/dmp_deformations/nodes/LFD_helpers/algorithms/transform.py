import numpy as np
from tf.transformations import quaternion_matrix,translation_matrix,quaternion_from_matrix

def transform_coord_frame(p_array,q_array,p_trans,q_rot):
    Mrot = quaternion_matrix(q_rot)
    Mtrans = translation_matrix(p_trans)
    Mregister = np.dot(Mtrans,Mrot)
    p_array_transformed = []
    q_array_transformed = []
    for p,q in zip(p_array,q_array):
        goal_trans = np.dot(translation_matrix(p),quaternion_matrix(q))
        goal_new = np.dot(Mregister,goal_trans)
        goal_rot = goal_new.copy()
        goal_rot[:,3] = 0
        goal_rot[3,3] = 1
        goalq = quaternion_from_matrix(goal_rot)
        goalp = goal_new[0:3,3]
        p_array_transformed.append(goalp)
        q_array_transformed.append(goalq)
    p_array_transformed = np.array(p_array_transformed)
    q_array_transformed = np.array(q_array_transformed)
    return p_array_transformed,q_array_transformed


