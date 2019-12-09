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
from ur5_datareader import UR5_dataReader

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
rospy.init_node('ur5_velocity_controller')

rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, solution_cb)
rospy.Subscriber('/vive_controller/grab', Int8, gripper_open_cb)
relaxedik_reset_pub = rospy.Publisher('relaxed_ik/reset', Bool, queue_size=2)
gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)
rospy.sleep(0.3)

net = Networking()
ur5_sock = net.init_ur_socket(net.urip, net.UR_PORT)

dr = UR5_dataReader()

disp = [-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 2]
starting_config = [ 3.14, -0.38, -1.2, -1.57, -1.57, -1.57 ]

ema_filter = f.EMA_filter([0.,0.,0.,0.,0.,0.], a=0.95)

# reset ur5
print 'resetting robot!'
move_ur5(ur5_sock, starting_config, disp, time=4.)

rospy.sleep(4.5)

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

velocity_limit_slider = 1.0
velocity_limits = velocity_limit_slider * np.array([ 3.15, 3.15, 3.15, 3.2, 3.2, 3.2 ])

dr.update()
curr_config = np.array(dr.q_actual) - np.array(disp)
print curr_config
while not np.linalg.norm(np.array(curr_config) - np.array(starting_config)) < 0.1:
    if idx % 100 == 0:
        print 'waiting for encoders to be correct.  You probably should just reset and start fresh.'
    idx += 1
    dr = UR5_dataReader()
    wait_rate.sleep()

iter = 0.6
while not rospy.is_shutdown():
    dr.update()
    curr_config = np.array(dr.q_actual) - np.array(disp)

    length_error = False
    if not len(solution) == 6:
        length_error = True

    if not len(curr_config) == 6:
        length_error = True

    if not length_error:
        target_vel = np.array(solution) - np.array(curr_config)

        scalar = math.pow(iter, 6.0)
        scalar = min(3.5, scalar)
        target_vel = scalar*target_vel
        filtered_vel = ema_filter.filter(target_vel)
        for i in xrange(6):
	    filtered_vel[i] = min(velocity_limits[i], filtered_vel[i])

        iter += 0.005
        print filtered_vel

        move_ur5_vel(ur5_sock, filtered_vel, time=0.04)
        pubGripper_robotiq(gripper_val, gripper_pub)

        rate.sleep()
