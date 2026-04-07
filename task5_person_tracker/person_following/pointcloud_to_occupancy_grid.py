#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Convert PointCloud2 into a rolling 2D OccupancyGrid.

This node is intended to bridge FAST-LIO/far_planner point-cloud maps to
an OccupancyGrid topic that downstream navigation helpers can consume.
"""

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node("pointcloud_to_occupancy_grid", anonymous=False)

        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.grid_topic = rospy.get_param("~grid_topic", "/person_following/occupancy_grid")

        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.grid_width = rospy.get_param("~grid_width", 20.0)
        self.grid_height = rospy.get_param("~grid_height", 20.0)
        self.resolution = rospy.get_param("~resolution", 0.10)

        self.min_z_rel = rospy.get_param("~min_z_rel", -0.20)
        self.max_z_rel = rospy.get_param("~max_z_rel", 1.20)
        self.robot_clear_radius = rospy.get_param("~robot_clear_radius", 0.45)
        self.inflate_radius = rospy.get_param("~inflate_radius", 0.20)

        self.publish_rate = rospy.get_param("~publish_rate", 8.0)
        self.cloud_timeout = rospy.get_param("~cloud_timeout", 1.0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.grid_pub = rospy.Publisher(self.grid_topic, OccupancyGrid, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        self.latest_cloud = None
        self.last_cloud_time = rospy.Time(0)

        self.width_cells = max(1, int(round(self.grid_width / self.resolution)))
        self.height_cells = max(1, int(round(self.grid_height / self.resolution)))
        self.inflate_cells = int(math.ceil(self.inflate_radius / self.resolution))

        rospy.loginfo(
            "pointcloud_to_occupancy_grid started: cloud=%s grid=%s frame=%s size=%.1fx%.1f res=%.2f",
            self.cloud_topic,
            self.grid_topic,
            self.target_frame,
            self.grid_width,
            self.grid_height,
            self.resolution,
        )

    def cloud_callback(self, msg):
        self.latest_cloud = msg
        self.last_cloud_time = rospy.Time.now()

    def _lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, "TF lookup failed for occupancy grid center: %s", exc)
            return None

        return tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z

    def _mark_inflated(self, grid, mx, my):
        w = self.width_cells
        h = self.height_cells

        if self.inflate_cells <= 0:
            if 0 <= mx < w and 0 <= my < h:
                grid[my * w + mx] = 100
            return

        r2 = self.inflate_cells * self.inflate_cells
        for dx in range(-self.inflate_cells, self.inflate_cells + 1):
            for dy in range(-self.inflate_cells, self.inflate_cells + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = mx + dx
                ny = my + dy
                if 0 <= nx < w and 0 <= ny < h:
                    grid[ny * w + nx] = 100

    def _build_grid(self, cloud, robot_x, robot_y, robot_z):
        # Start with free space in the rolling window and mark observed obstacles.
        grid = [0] * (self.width_cells * self.height_cells)

        origin_x = robot_x - 0.5 * self.grid_width
        origin_y = robot_y - 0.5 * self.grid_height

        clear_r2 = self.robot_clear_radius * self.robot_clear_radius

        if cloud.header.frame_id and cloud.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(
                2.0,
                "Cloud frame '%s' != target_frame '%s'. Configure cloud in target frame.",
                cloud.header.frame_id,
                self.target_frame,
            )

        for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p

            z_rel = z - robot_z
            if z_rel < self.min_z_rel or z_rel > self.max_z_rel:
                continue

            dx = x - robot_x
            dy = y - robot_y
            if dx * dx + dy * dy < clear_r2:
                continue

            mx = int((x - origin_x) / self.resolution)
            my = int((y - origin_y) / self.resolution)

            if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
                self._mark_inflated(grid, mx, my)

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid

        return msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            if self.latest_cloud is None:
                rospy.logwarn_throttle(2.0, "Waiting for point cloud on %s", self.cloud_topic)
                rate.sleep()
                continue

            if (rospy.Time.now() - self.last_cloud_time).to_sec() > self.cloud_timeout:
                rospy.logwarn_throttle(1.0, "Point cloud timeout on %s", self.cloud_topic)
                rate.sleep()
                continue

            pose = self._lookup_robot_pose()
            if pose is None:
                rate.sleep()
                continue

            robot_x, robot_y, robot_z = pose
            grid_msg = self._build_grid(self.latest_cloud, robot_x, robot_y, robot_z)
            self.grid_pub.publish(grid_msg)

            rate.sleep()


if __name__ == "__main__":
    node = PointCloudToOccupancyGrid()
    node.run()
