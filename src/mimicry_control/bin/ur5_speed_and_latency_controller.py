#!/usr/bin/python

import rospy
from relaxed_ik.msg import JointAngles
from sensor_msgs.msg import JointState
from broadcaster import *
# from robot_type import Mico
from std_msgs.msg import Int8, Bool
import numpy as np
from networking import Networking
import math as M
import joint_utils as ju
import filter as f

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
rospy.init_node('ur5_speed_and_latency_controller')

rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, solution_cb)
rospy.Subscriber('gripper_val', Int8, gripper_open_cb)
relaxedik_reset_pub = rospy.Publisher('relaxed_ik/reset', Bool, queue_size=2)
# gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
rospy.sleep(0.3)

net = Networking()
ur5_sock = net.init_ur_socket(net.urip, net.UR_PORT)

disp = [-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 2]
starting_config = [ 3.14, -0.38, -1.2, -1.57, -1.57, -1.57 ]

ema_filter = f.EMA_filter(starting_config, a=0.99)

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

# make sure relaxed_ik is reset!!!
reset_msg = Bool()
reset_msg.data = True
relaxedik_reset_pub.publish(reset_msg)

idx = 1
while not np.linalg.norm(np.array(solution) - np.array(starting_config)) < 0.1:
    if idx % 100 == 0:
        print 'waiting for relaxed_ik to properly reset!'
    idx += 1
    wait_rate.sleep()

print 'got here!'
fixed_frequency = 125
rate = rospy.Rate(fixed_frequency)
prev_config = starting_config

latency = 0.0

state_queue = []
start_time = rospy.get_time()

while not rospy.is_shutdown():
    length_error = False
    if not (len(solution) == 6 and len(prev_config) == 6):
        length_error = True

    if not length_error:
        velocity_limit_slider = rospy.get_param('joint_vel_slider', default=1.0)
        velocity_limits = velocity_limit_slider * np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])

        clipped_config, ret_k = ju.interpolate_to_velocity_limits(prev_config, solution, t=0.008, numDOF=6, velocity_limits=velocity_limits)
        state_to_send = ema_filter.filter(clipped_config)
        state_queue.append(state_to_send)
        prev_config = state_to_send

        curr_time = rospy.get_time()
        accumulated_time = curr_time - start_time

        if accumulated_time > latency:
            curr_state = state_queue.pop(0)
            print curr_state
            move_ur5(ur5_sock, curr_state, disp)
            # pubGripper_robotiq(gripper_val, gripper_pub)

        rate.sleep()

