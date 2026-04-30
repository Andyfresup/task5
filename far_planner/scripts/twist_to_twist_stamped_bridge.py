#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


class TwistToTwistStampedBridge:
    """Bridge /cmd_vel_nav (Twist) -> /motion_target/target_speed_chassis (TwistStamped)."""

    def __init__(self):
        self.in_topic = rospy.get_param("~in_topic", "/cmd_vel_nav")
        self.out_topic = rospy.get_param("~out_topic", "/motion_target/target_speed_chassis")
        self.frame_id = rospy.get_param("~frame_id", "")
        self.release_brake = rospy.get_param("~release_brake", True)
        self.brake_topic = rospy.get_param("~brake_topic", "/motion_target/brake_mode")

        self.pub = rospy.Publisher(self.out_topic, TwistStamped, queue_size=10)
        self.brake_pub = rospy.Publisher(self.brake_topic, Bool, queue_size=10) if self.release_brake else None
        self.sub = rospy.Subscriber(self.in_topic, Twist, self._cb, queue_size=10)

        rospy.loginfo("[twist_bridge] %s (Twist) -> %s (TwistStamped)", self.in_topic, self.out_topic)
        if self.release_brake:
            rospy.loginfo("[twist_bridge] brake release enabled: publish False to %s", self.brake_topic)
        else:
            rospy.loginfo("[twist_bridge] brake release disabled")

    def _cb(self, msg: Twist):
        if self.brake_pub is not None:
            self.brake_pub.publish(Bool(data=False))

        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rospy.init_node("twist_to_twist_stamped_bridge", anonymous=False)
    TwistToTwistStampedBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
