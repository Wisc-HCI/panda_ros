from networking import Networking
from broadcaster import move_sawyer_set_velocities, move_sawyer, pubGripper_sawyer
import math as M
import numpy as np
import rospy
from relaxed_ik.msg import JointAngles
from std_msgs.msg import Bool, Int8
import intera_interface
import intera_interface.gripper as G


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


rospy.init_node('sawyer_controller')

rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, solution_cb)
rospy.Subscriber('/vive_controller/grab', Int8, gripper_open_cb)
relaxedik_reset_pub = rospy.Publisher('relaxed_ik/reset', Bool, queue_size=2)
rospy.sleep(0.3)

initState = [ 0.0, -0.57307964, 0.3837615, 2.25103946, -2.92990265, 1.7088147, 1.2739 ]

limb = intera_interface.Limb('right')
g = G.Gripper()

print 'resetting...'
move_sawyer(initState, limb, timeout=7.0)

rospy.sleep(7.0)

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

prev_gripper_val = 0

latency = 0.0
state_queue = []
start_time = rospy.get_time()


idx = 1
while not np.linalg.norm(np.array(solution) - np.array(initState)) < 0.6:
    if idx % 100 == 0:
        print 'waiting for relaxed_ik to properly reset!'
    idx += 1
    wait_rate.sleep()


rate = rospy.Rate(125)
while not rospy.is_shutdown():
    error = False
    if not len(solution) == 7:
        error = True

    if not error:
        state_queue.append(solution)
        curr_time = rospy.get_time()
        accumulated_time = curr_time - start_time

        if accumulated_time > latency:
            curr_state = state_queue.pop(0)
            velocity_scalar = rospy.get_param('/velocity_scalar', default=3.0)
            move_sawyer_set_velocities(curr_state, limb, velocity_scalar=velocity_scalar)
            if not prev_gripper_val == gripper_val:
                print 'got here'
                pubGripper_sawyer(gripper_val, g)

        prev_gripper_val = gripper_val

    rate.sleep()