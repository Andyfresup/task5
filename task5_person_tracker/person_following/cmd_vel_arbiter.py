#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Arbitrate between planner velocity and search velocity.

Priority order:
- If search velocity is fresh and non-zero, use it.
- Otherwise use planner velocity if it is fresh.
- If nothing is fresh, publish zero.
"""

import math
import rospy
from geometry_msgs.msg import Twist


class CmdVelArbiter:
    def __init__(self):
        rospy.init_node("cmd_vel_arbiter", anonymous=False)

        self.search_topic = rospy.get_param("~search_topic", "/person_following/search_cmd_vel")
        self.nav_topic = rospy.get_param("~nav_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self.search_timeout = rospy.get_param("~search_timeout", 0.5)
        self.nav_timeout = rospy.get_param("~nav_timeout", 1.0)
        self.twist_deadband = rospy.get_param("~twist_deadband", 1e-3)

        self.search_msg = None
        self.search_time = rospy.Time(0)
        self.nav_msg = None
        self.nav_time = rospy.Time(0)

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        rospy.Subscriber(self.search_topic, Twist, self.search_callback, queue_size=10)
        rospy.Subscriber(self.nav_topic, Twist, self.nav_callback, queue_size=10)

        self.rate = rospy.Rate(rospy.get_param("~rate", 20.0))
        rospy.loginfo(
            "cmd_vel_arbiter started: search=%s nav=%s output=%s",
            self.search_topic,
            self.nav_topic,
            self.output_topic,
        )

    def search_callback(self, msg):
        self.search_msg = msg
        self.search_time = rospy.Time.now()

    def nav_callback(self, msg):
        self.nav_msg = msg
        self.nav_time = rospy.Time.now()

    def _fresh(self, stamp, timeout):
        if stamp == rospy.Time(0):
            return False
        return (rospy.Time.now() - stamp).to_sec() <= timeout

    def _nonzero(self, twist):
        return (
            abs(twist.linear.x) > self.twist_deadband
            or abs(twist.linear.y) > self.twist_deadband
            or abs(twist.linear.z) > self.twist_deadband
            or abs(twist.angular.x) > self.twist_deadband
            or abs(twist.angular.y) > self.twist_deadband
            or abs(twist.angular.z) > self.twist_deadband
        )

    def _select_twist(self):
        search_msg = self.search_msg
        search_time = self.search_time
        nav_msg = self.nav_msg
        nav_time = self.nav_time

        search_fresh = self._fresh(search_time, self.search_timeout)
        nav_fresh = self._fresh(nav_time, self.nav_timeout)

        if search_fresh and search_msg is not None and self._nonzero(search_msg):
            return search_msg

        if nav_fresh and nav_msg is not None:
            return nav_msg

        return Twist()

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self._select_twist())
            self.rate.sleep()


if __name__ == "__main__":
    CmdVelArbiter().run()
