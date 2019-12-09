#! /usr/bin/env python


import rospy
# from robotiq_broadcaster import pubGripper_robotiq
# from robotiq_85_msgs.msg import GripperCmd
import intera_interface.gripper as G

rospy.init_node('open_gripper')

# gripper_pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

# rospy.sleep(1.0)
print 'opening gripper in 7 seconds...'

rospy.sleep(7.0)

print 'opening gripper...'
g = G.Gripper()
print g
g.open()

# pubGripper_robotiq(1, gripper_pub)