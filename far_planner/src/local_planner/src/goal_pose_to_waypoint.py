#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped


def goal_callback(msg, pub):
    out = PointStamped()
    out.header = msg.header
    out.point.x = msg.pose.position.x
    out.point.y = msg.pose.position.y
    out.point.z = msg.pose.position.z
    pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("goal_pose_to_waypoint_bridge")
    out_topic = rospy.get_param("~waypoint_topic", "/way_point")
    in_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")

    pub = rospy.Publisher(out_topic, PointStamped, queue_size=1)
    rospy.Subscriber(in_topic, PoseStamped, goal_callback, callback_args=pub, queue_size=1)
    rospy.spin()
