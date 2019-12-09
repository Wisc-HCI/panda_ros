#!/usr/bin/python
import rospy
from relaxed_ik.msg import JointAngles
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, Pose
from relaxed_ik.msg import EEPoseGoals
from std_msgs.msg import Int8, String
import transformations as T
import copy
import numpy as np

glob_vector3 = ''
def vector3_cb(data):
    global glob_vector3
    glob_vector3 = []
    glob_vector3.append(data.vector.x)
    glob_vector3.append(data.vector.y)
    glob_vector3.append(data.vector.z)



glob_quaternion = ''
def quaternion_cb(data):
    global glob_quaternion
    glob_quaternion = []
    glob_quaternion.append(data.quaternion.w)
    glob_quaternion.append(data.quaternion.x)
    glob_quaternion.append(data.quaternion.y)
    glob_quaternion.append(data.quaternion.z)


glob_clutch_down = 0
def clutch_cb(data):
    global glob_clutch_down
    glob_clutch_down = data.data

glob_grab_down = 0
def grab_cb(data):
    global glob_grab_down
    if data.data == 1 and glob_grab_down == 0:
        commands_pub.publish("button_pressed")
    elif data.data == 0 and glob_grab_down == 1:
        commands_pub.publish("button_released")
    glob_grab_down = data.data

rospy.init_node('pose_goal_pub')

rospy.Subscriber('/vive_controller/quaternion_r', QuaternionStamped, quaternion_cb)
rospy.Subscriber('/vive_controller/position_r', Vector3Stamped, vector3_cb)
rospy.Subscriber('/vive_controller/clutch', Int8, clutch_cb)
rospy.Subscriber('/vive_controller/grab', Int8, grab_cb)
commands_pub = rospy.Publisher('/interaction/commands', String, queue_size=3)
eepg_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)
rospy.sleep(0.3)

while glob_vector3 == '': continue


mode = 'clutch'

clutch_val = 0
prev_clutch_down = 0
prev_pos_goal = [0,0,0]
prev_quat_goal = [1,0,0,0]
prev_pos_raw = [0,0,0]
prev_quat_raw = [1,0,0,0]
position_on_clutch = [0,0,0]
rotation_on_clutch = [1,0,0,0]

rate = rospy.Rate(500)
while not rospy.is_shutdown():
    quaternion = copy.deepcopy(glob_quaternion)
    vector3 = copy.deepcopy(glob_vector3)
    clutch_down = glob_clutch_down

    length_error = False
    if not len(vector3) == 3: length_error = True
    if not len(quaternion) == 4: length_error = True

    if length_error: 
        rate.sleep()
        continue
    
    eepg = EEPoseGoals()
    pose = Pose()

    if mode == 'standard':
        pose.position.x = vector3[0]
        pose.position.y = vector3[1]
        pose.position.z = vector3[2]

        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]

    elif mode == 'clutch':
        if (clutch_down != prev_clutch_down) and (clutch_down == 1):
            # It's fine not to deep-copy since values are not updated unless clutch == 0
            position_on_clutch = prev_pos_goal
            rotation_on_clutch = prev_quat_goal

        if clutch_down == 1:
            pose.position.x = position_on_clutch[0]
            pose.position.y = position_on_clutch[1]
            pose.position.z = position_on_clutch[2]

            pose.orientation.w = rotation_on_clutch[0]
            pose.orientation.x = rotation_on_clutch[1]
            pose.orientation.y = rotation_on_clutch[2]
            pose.orientation.z = rotation_on_clutch[3]
        elif clutch_down == 0:
            translation_velocity = np.array(vector3) - np.array(prev_pos_raw)
            curr_pos_goal = np.array(prev_pos_goal) + translation_velocity
            pose.position.x = curr_pos_goal[0]
            pose.position.y = curr_pos_goal[1]
            pose.position.z = curr_pos_goal[2]
            prev_pos_goal = curr_pos_goal

            qv1 = T.rotate_quaternion_representation(prev_quat_raw, T.quaternion_matrix(quaternion)[:3,:3])
            qv = T.rotate_quaternion_representation(T.quaternion_dispQ(qv1, quaternion),
                                                    T.quaternion_matrix([1., 0., 0., 0.])[:3, :3])
            curr_quat_goal = T.quaternion_multiply(qv, prev_quat_goal)

            pose.orientation.w = curr_quat_goal[0]
            pose.orientation.x = curr_quat_goal[1]
            pose.orientation.y = curr_quat_goal[2]
            pose.orientation.z = curr_quat_goal[3]
            prev_quat_goal = curr_quat_goal

        prev_pos_raw = copy.deepcopy(vector3)
        prev_quat_raw = copy.deepcopy(quaternion)
        prev_clutch_down = clutch_down

    eepg.ee_poses.append(pose)
    eepg_pub.publish(eepg)

    rate.sleep()


