#!/usr/bin/python

import rospy
from relaxed_ik.msg import JointAngles
from sensor_msgs.msg import JointState
from broadcaster import *
# from robot_type import Mico
from std_msgs.msg import Int8
import numpy as np
from networking import Networking
import math as M

solution = ''
def solution_cb(data):
    global solution
    solution = []
    for a in data.angles.data:
        solution.append(a)

# 0 is open, 1 is closed
gripper_val = 0
def gripper_open_cb(data):
    global gripper_val
    gripper_val = data.data

########################################################################################################################
rospy.init_node('ur5_controller')

rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, solution_cb)
rospy.Subscriber('gripper_val', Int8, gripper_open_cb)
# gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
rospy.sleep(0.3)

net = Networking()
ur5_sock = net.init_ur_socket(net.urip, net.UR_PORT)


disp = [-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 2]
starting_config = [ 3.14, -0.38, -1.2, -1.57, -1.57, -1.57 ]

# reset ur5
print 'resetting robot!'
move_ur5(ur5_sock, starting_config, disp, time=4.)

rospy.sleep(4.0)

wait_rate = rospy.Rate(100)
idx = 1
while solution == '':
    if idx % 200 == 0:
        print 'waiting for robot solution'
    idx += 1
    wait_rate.sleep()


rate = rospy.Rate(125)
while not rospy.is_shutdown():
    length_error = False
    if not len(solution) == 6:
        length_error = True

    if not length_error:
        print solution
        move_ur5(ur5_sock, solution, disp)
        # pubGripper_robotiq(gripper_val, gripper_pub)

        rate.sleep()


