#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped

def callback(msg):
    pub.publish(msg.twist)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    output_topic = rospy.get_param('~output_cmd_vel_topic', '/cmd_vel')
    pub = rospy.Publisher(output_topic, Twist, queue_size=1)
    rospy.Subscriber('/cmd_vel_stamped', TwistStamped, callback)
    rospy.spin()
