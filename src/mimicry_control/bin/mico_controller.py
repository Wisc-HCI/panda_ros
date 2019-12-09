#!/usr/bin/python

import rospy
from relaxed_ik.msg import JointAngles
from sensor_msgs.msg import JointState
from broadcaster import *
# from robot_type import Mico
from std_msgs.msg import Int8
import numpy as np


mico_solution = ''
def mico_solution_cb(data):
    global mico_solution
    mico_solution = []
    for a in data.angles.data:
        mico_solution.append(a)


mico_current_state = ''
def mico_current_state_cb(data):
    global mico_current_state
    mico_current_state = []
    for i,x in enumerate(data.position):
        if i >= 6: return
        mico_current_state.append(x)

# 0 is open, 1 is closed
gripper_val = 0
def gripper_open_cb(data):
    global gripper_val
    gripper_val = data.data

########################################################################################################################
rospy.init_node('mico_controller')

rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, mico_solution_cb)
rospy.Subscriber('/m1n6s200_driver/out/joint_state', JointState, mico_current_state_cb)
rospy.Subscriber('gripper_val', Int8, gripper_open_cb)
gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
rospy.sleep(0.3)

# robot_obj = Mico()

# robot_obj.reset()
starting_config = [ 0, 3.41, 5.0, 0, 0, 1.57 ]
move_mico(starting_config, duration=30.0)


while mico_current_state == '':
    print 'waiting for mico_current_state'
    continue
while mico_solution == '':
    print 'waiting for mico_solution'
    continue



rate = rospy.Rate(103)
while not rospy.is_shutdown():
    length_error = False
    if not len(mico_solution) == 6:
        length_error = True
    if not len(mico_current_state) == 6:
        length_error = False

    if not length_error:
        print mico_solution
        move_mico_velocity(mico_solution, mico_current_state)
        pubGripper_robotiq(gripper_val, gripper_pub)

        rate.sleep()








