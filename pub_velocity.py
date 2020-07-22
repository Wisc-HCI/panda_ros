import rospy
from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    rospy.init_node('vel_pub')

    pub = rospy.Publisher('/panda/cart_velocity', TwistStamped, queue_size=1)
    rospy.sleep(0.5)
    msg = TwistStamped()
    msg.twist.linear.x = -0.01
    msg.twist.angular.x = 0.1
    msg.header.stamp = rospy.Time.now()
    msg.header.stamp.secs += 1
    pub.publish(msg)