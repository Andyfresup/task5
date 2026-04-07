# person tracking and following node with obstacle avoidance

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower_node')

        # === Parameters ===
        self.goal_update_distance = rospy.get_param('~goal_update_distance', 0.6)
        self.follow_distance = rospy.get_param('~follow_distance', 1.5)
        self.global_frame = rospy.get_param('~global_frame', 'map')

        # When the robot is closer than this radius to the table,
        # stop following the person and enter "go-to-table mode"
        self.table_stop_distance = rospy.get_param('~table_stop_distance', 1.5)

        # In table mode, resend the table goal at this interval (seconds)
        self.table_goal_interval = rospy.get_param('~table_goal_interval', 1.0)

        # === New: Idle detection parameters in table mode ===
        # If in table mode and the robot's displacement remains below
        # idle_move_threshold for idle_timeout seconds, shut down the node
        self.idle_timeout = rospy.get_param('~idle_timeout', 15)          # Idle time threshold (seconds)
        self.idle_move_threshold = rospy.get_param('~idle_move_threshold', 0.02)  # Allowed displacement (meters)

        # Record time and position when entering table mode
        self.idle_start_time = None           # rospy.Time
        self.idle_start_position = None       # (x, y)

        # === TF2 listener ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # === Publishers and Subscribers ===
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/person_marker', Marker, queue_size=1)

        # Wait until /move_base_simple/goal has at least one subscriber
        rospy.loginfo("Waiting for subscriber on /move_base_simple/goal...")
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("move_base connected.")

        rospy.Subscriber('/person/base_link_3d_position', PointStamped, self.person_callback)
        self.odom_sub = rospy.Subscriber('/local_odom', Odometry, self.odom_callback, queue_size=1)

        # === Variables ===
        # Last navigation goal generated from person following
        self.last_goal_position = None

        # Current robot pose (from /local_odom)
        self.robot_pose = None

        # Whether the robot has entered "go-to-table mode"
        self.final_goal_sent = False

        # Last time the table goal was sent (for rate limiting)
        self.last_table_goal_time = rospy.Time(0)

        from tf.transformations import euler_from_quaternion, quaternion_from_euler

        # === Predefined final table goal ===
        self.table_goal = PoseStamped()

        _, _, yaw = euler_from_quaternion([0.01, 0.29312990, -0.0117560529, 0.95594676026104])
        q = quaternion_from_euler(0.0, 0.0, yaw)

        self.table_goal.header.frame_id = self.global_frame
        self.table_goal.pose.position.x = -3.04071
        self.table_goal.pose.position.y = -2.394332968
        self.table_goal.pose.position.z = 0.0
        self.table_goal.pose.orientation.x = q[0]
        self.table_goal.pose.orientation.y = q[1]
        self.table_goal.pose.orientation.z = q[2]
        self.table_goal.pose.orientation.w = q[3]

        rospy.loginfo("Person follower node started with table resend logic and idle shutdown logic.")

    # ========== Odometry callback: update robot pose ==========
    def odom_callback(self, odom_msg):
        # Store full pose (only planar position is used)
        self.robot_pose = odom_msg.pose.pose

        # Check whether to switch to table mode
        self.check_and_send_table_goal()

        # Check for prolonged idle condition in table mode
        self.check_idle_and_shutdown()

    # ========== Check whether to switch to table goal and resend ==========
    def check_and_send_table_goal(self):
        if self.robot_pose is None:
            return

        # Compute planar distance to table
        dx = self.table_goal.pose.position.x - self.robot_pose.position.x
        dy = self.table_goal.pose.position.y - self.robot_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.table_stop_distance:

            # First time entering table zone
            if not self.final_goal_sent:
                rospy.loginfo("Within %.2f m of table. Switching to table mode and stopping person following.", dist)
                self.final_goal_sent = True

                # Initialize idle monitoring
                self.idle_start_time = rospy.Time.now()
                self.idle_start_position = (
                    self.robot_pose.position.x,
                    self.robot_pose.position.y
                )
                rospy.loginfo("Started idle monitoring in table mode.")

            # Resend table goal at fixed interval
            now = rospy.Time.now()
            if (now - self.last_table_goal_time).to_sec() >= self.table_goal_interval:
                self.table_goal.header.stamp = now
                self.goal_pub.publish(self.table_goal)
                self.last_table_goal_time = now
                rospy.loginfo("Resending table goal. Distance: %.2f m", dist)

    # ========== Idle detection in table mode ==========
    def check_idle_and_shutdown(self):

        if not self.final_goal_sent:
            return

        if self.robot_pose is None:
            return

        if self.idle_start_time is None or self.idle_start_position is None:
            self.idle_start_time = rospy.Time.now()
            self.idle_start_position = (
                self.robot_pose.position.x,
                self.robot_pose.position.y
            )
            return

        cur_x = self.robot_pose.position.x
        cur_y = self.robot_pose.position.y

        dx = cur_x - self.idle_start_position[0]
        dy = cur_y - self.idle_start_position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.idle_move_threshold:
            self.idle_start_time = rospy.Time.now()
            self.idle_start_position = (cur_x, cur_y)
            return

        elapsed = (rospy.Time.now() - self.idle_start_time).to_sec()
        if elapsed >= self.idle_timeout:
            rospy.loginfo("Robot idle in table mode for %.1f seconds (%.3f m displacement). Shutting down.", elapsed, dist)
            rospy.signal_shutdown("Robot idle at table for too long")

    # ========== Person position callback ==========
    def person_callback(self, point_stamped_msg):

        if self.final_goal_sent:
            return

        # Transform person position to global frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                point_stamped_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            person_point_map = tf2_geometry_msgs.do_transform_point(point_stamped_msg, transform)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF transform failed: %s", e)
            return

        # Check whether person moved enough to update goal
        if self.last_goal_position:
            dist = math.sqrt(
                (person_point_map.point.x - self.last_goal_position.x) ** 2 +
                (person_point_map.point.y - self.last_goal_position.y) ** 2
            )
            if dist < self.goal_update_distance:
                self.publish_person_marker(person_point_map.point)
                return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.global_frame
        goal_msg.header.stamp = rospy.Time.now()

        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.global_frame, 'base_link', rospy.Time(0)
            )
            robot_pos = robot_transform.transform.translation

            dx = person_point_map.point.x - robot_pos.x
            dy = person_point_map.point.y - robot_pos.y
            angle_to_person = math.atan2(dy, dx)

            goal_x = person_point_map.point.x - self.follow_distance * math.cos(angle_to_person)
            goal_y = person_point_map.point.y - self.follow_distance * math.sin(angle_to_person)

            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.position.z = 0.0

            q = tf.transformations.quaternion_from_euler(0, 0, angle_to_person)
            goal_msg.pose.orientation.x = q[0]
            goal_msg.pose.orientation.y = q[1]
            goal_msg.pose.orientation.z = q[2]
            goal_msg.pose.orientation.w = q[3]

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get robot position: %s. Using person position directly.", e)
            goal_msg.pose.position = person_point_map.point

        self.goal_pub.publish(goal_msg)
        self.last_goal_position = goal_msg.pose.position

        rospy.loginfo("Updated follow goal to: (%.2f, %.2f)",
                      goal_msg.pose.position.x,
                      goal_msg.pose.position.y)

        self.publish_person_marker(person_point_map.point)

    # ========== RViz person visualization ==========
    def publish_person_marker(self, position):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "person"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
