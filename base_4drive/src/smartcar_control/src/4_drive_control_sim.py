#!/usr/bin/env python3

from enum import IntEnum
# import commands
from threading import Lock, Thread
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from smartcar_control.srv import *
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool, Float32, Float64, Header, String
# from ackermann_msgs.msg import AckermannDriveStamped
import math
import tf2_ros
import tf
import tf.transformations as tft

flag_move = 0


class AngleCtrlMode(IntEnum):
    Angular_speed = 0
    Front_Angle = 1


def limit_45degree(theta):
    return min(max(-math.pi / 4, theta), math.pi / 4)


def normalize_90degree(theta):
    if theta > math.pi / 2:
        theta = -math.pi + theta
    elif theta < -math.pi / 2:
        theta = math.pi + theta
    return theta


class Four_Drive_Car():
    def __init__(self):
        self.slider_position = 0.0
        self.left_arm_position = 0.0
        self.right_arm_position = 0.0

        self.state_list = ['ackman', 'translation', 'rotate', 'parking']
        self.state = self.state_list[0]
        self.angle_ctrl_mode = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_theta = 0.0
        self.gtruth_vel_x = 0.0
        self.gtruth_vel_y = 0.0
        self.gtruth_vel_theta = 0.0
        self.gtruth_theta = 0.0
        self.acc_vel = rospy.get_param('/acc_vel', 0.1)
        self.acc_theta = rospy.get_param('/acc_theta', 0.1)
        self.dist_axis = rospy.get_param('/dist_axis', 0.52)
        self.dist_wheel = rospy.get_param('/dist_wheel', 0.35)
        self.wheel_radius = rospy.get_param('/wheel_radius', 0.085)

        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_received_stamp = None
        self.tf_publisher = tf2_ros.TransformBroadcaster()

        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self._cb_cmd_vel)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._sub_robot_pose_update)

        self.srv_state = rospy.Service('/state', SetString, self._cb_srv_state)
        self.srv_slider_ctrl = rospy.Service('/slider_ctrl', SetFloat, self._cb_srv_slider_ctrl)
        # self.srv_left_arm_height_ctrl = rospy.Service('/left_arm_height_ctrl', SetFloat, self._cb_srv_left_arm_height_ctrl)
        # self.srv_right_arm_height_ctrl = rospy.Service('/right_arm_height_ctrl', SetFloat, self._cb_srv_right_arm_height_ctrl)
        self.srv_angle_ctrl = rospy.Service('/angle_ctrl_mode', SetInt, self._cb_srv_angle_ctrl)

        self.state_publisher = rospy.Publisher('/smartcar_state', String, queue_size=1)

        self.pub_pos_slider = rospy.Publisher('/smartcar/slider_position_controller/command', Float64,
                                                       queue_size=1)

        # self.pub_pos_arm_left = rospy.Publisher('/smartcar/arm_left_position_controller/command', Float64,
        #                                       queue_size=1)
        # self.pub_pos_arm_right = rospy.Publisher('/smartcar/arm_right_position_controller/command', Float64,
        #                                       queue_size=1)

        self.pub_pos_left_j1 = rospy.Publisher('/smartcar/arm_left_position_controller1/command', Float64,
                                                       queue_size=1)
        self.pub_pos_left_j2 = rospy.Publisher('/smartcar/arm_left_position_controller2/command', Float64,
                                                        queue_size=1)
        self.pub_pos_left_j3 = rospy.Publisher('/smartcar/arm_left_position_controller3/command', Float64,
                                                        queue_size=1)
        self.pub_pos_left_j4 = rospy.Publisher('/smartcar/arm_left_position_controller4/command', Float64,
                                                         queue_size=1)
        self.pub_pos_left_j5 = rospy.Publisher('/smartcar/arm_left_position_controller5/command', Float64,
                                                        queue_size=1)
        self.pub_pos_left_j6 = rospy.Publisher('/smartcar/arm_left_position_controller6/command', Float64,
                                                         queue_size=1)

        self.pub_vel_left_rear_wheel = rospy.Publisher('/smartcar/rear_left_velocity_controller/command', Float64,
                                                       queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/smartcar/rear_right_velocity_controller/command', Float64,
                                                        queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/smartcar/front_left_velocity_controller/command', Float64,
                                                        queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/smartcar/front_right_velocity_controller/command', Float64,
                                                         queue_size=1)

        self.pub_pos_left_rear_hinge = rospy.Publisher('/smartcar/left_back_bridge_position_controller/command',
                                                       Float64,
                                                       queue_size=1)
        self.pub_pos_right_rear_hinge = rospy.Publisher('/smartcar/right_back_bridge_position_controller/command',
                                                        Float64,
                                                        queue_size=1)
        self.pub_pos_left_front_hinge = rospy.Publisher('/smartcar/left_bridge_position_controller/command', Float64,
                                                        queue_size=1)
        self.pub_pos_right_front_hinge = rospy.Publisher('/smartcar/right_bridge_position_controller/command', Float64,
                                                         queue_size=1)
        
        self.wheel_vel_left_front = 0
        self.wheel_vel_right_front = 0
        self.wheel_vel_left_rear = 0
        self.wheel_vel_right_rear = 0
        self.wheel_angle_left_front = 0
        self.wheel_angle_right_front = 0
        self.wheel_angle_left_rear = 0
        self.wheel_angle_right_rear = 0

        self._th_run = Thread(target=self.run)
        self._th_run.start()

    def _cb_srv_angle_ctrl(self, data):
        self.angle_ctrl_mode = data.data
        return SetIntResponse(True, '')

    def _cb_srv_slider_ctrl(self, data):
        self.slider_position = data.data
        self.pub_pos_slider.publish(data.data)
        return SetFloatResponse(True, '')

    # def _cb_srv_left_arm_height_ctrl(self, data):
    #     self.left_arm_position = data.data
    #     self.pub_pos_arm_left.publish(data.data)
    #     return SetFloatResponse(True, '')

    # def _cb_srv_right_arm_height_ctrl(self, data):
    #     self.right_arm_position = data.data
    #     self.pub_pos_arm_right.publish(data.data)
    #     return SetFloatResponse(True, '')

    def _cb_cmd_vel(self, data):
        # rospy.loginfo("Get Twist x = {}, y = {}, theta = {}".format(data.linear.x,data.linear.y,data.angular.z*180/math.pi))
        self.vel_x = min(max(data.linear.x, self.vel_x - self.acc_vel), self.vel_x + self.acc_vel)
        self.vel_y = min(max(data.linear.y, self.vel_y - self.acc_vel), self.vel_y + self.acc_vel)
        self.vel_theta = min(max(data.angular.z, self.vel_theta - self.acc_theta), self.vel_theta + self.acc_theta)

        # if abs(self.theta + self.vel_theta) <= math.pi/2:
        #     self.theta += self.vel_theta
        # else:
        #     self.theta = math.pi/2 if self.theta>0 else -math.pi/2

        # rospy.loginfo("Current body_speed x = {}, y = {}, theta = {}".format(self.vel_x,self.vel_y,self.vel_theta))

    def _sub_robot_pose_update(self, msg):
        index = msg.name.index('mrobot::base_footprint')

        self.gtruth_vel_x = msg.twist[index].linear.x
        self.gtruth_vel_y = msg.twist[index].linear.y
        self.gtruth_theta = tft.euler_from_quaternion([
            msg.pose[index].orientation.x,
            msg.pose[index].orientation.y,
            msg.pose[index].orientation.z,
            msg.pose[index].orientation.w])[2]
        self.gtruth_vel_theta = msg.twist[index].angular.z

        self.last_received_pose = msg.pose[index]
        self.last_received_twist = msg.twist[index]
        self.last_received_stamp = rospy.Time.now()
        cmd = Odometry()
        cmd.header.stamp = self.last_received_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_footprint'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
                               0, 1e-3, 0, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 1e3]
        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9]
        self.odom_publisher.publish(cmd)
        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_publisher.sendTransform(tf)

    def _cb_srv_state(self, data):
        if data.data in self.state_list:
            self.state = data.data
            rospy.loginfo('Switch to state {}'.format(data.data))
            return SetStringResponse(True, '')
        else:
            rospy.logerr('No such state {}'.format(data.data))
            return SetStringResponse(False, 'No such state {}'.format(data.data))

    def run(self):
        rate = rospy.Rate(10)
        vel = 0.0
        theta = 0.0
        l_r = l_f = self.dist_axis / 2
        l_o2r = 0
        ref_vel_x = self.vel_x
        angle_front = 0
        state = String()
        while not rospy.is_shutdown():
            if self.state == 'ackman':
                if self.angle_ctrl_mode == AngleCtrlMode.Angular_speed:
                    if abs(self.vel_x) <= 1e-6:
                        ref_vel_x = 1
                    else:
                        ref_vel_x = self.vel_x
                    # tan_beta = (self.vel_theta * l_f * l_r) / (2 * self.dist_axis * ref_vel_x)
                    # angle_beta = math.atan2(tan_beta, 1)
                    # angle_front = math.atan2(tan_beta * self.dist_axis, l_r)
                    angle_front = math.atan2(self.vel_theta * self.dist_axis / ref_vel_x, 1)
                    # print(self.vel_theta, self.dist_axis, ref_vel_x)
                    # angle_front = limit_45degree(angle_front)
                elif self.angle_ctrl_mode == AngleCtrlMode.Front_Angle:
                    angle_front = limit_45degree(self.vel_theta)

                if angle_front == 0:
                    vel_front = self.vel_x
                    theta_left = 0
                    theta_right = 0
                    vel_rear_left = self.vel_x
                    vel_rear_right = self.vel_x
                    vel_front_left = self.vel_x
                    vel_front_right = self.vel_x
                else:
                    l_o2r = self.dist_axis / math.tan(angle_front)
                    l_o2f = math.sqrt(l_o2r * l_o2r + self.dist_axis * self.dist_axis)

                    vel_rear_left = self.vel_x * (1 - (self.dist_wheel/2) / l_o2r)
                    vel_rear_right = self.vel_x * (1 + (self.dist_wheel/2) / l_o2r)

                    vel_front = self.vel_x / math.cos(angle_front)
                    vel_angle_front = vel_front / l_o2f
                    # print(vel_angle_front)

                    # vel_angle_front = self.vel_theta
                    vel_front_left = vel_angle_front * math.sqrt((l_o2r - self.dist_wheel / 2) * (
                            l_o2r - self.dist_wheel / 2) + self.dist_axis * self.dist_axis)
                    vel_front_right = vel_angle_front * math.sqrt((l_o2r + self.dist_wheel / 2) * (
                            l_o2r + self.dist_wheel / 2) + self.dist_axis * self.dist_axis)

                    theta_left = normalize_90degree(math.atan2(self.dist_axis, (l_o2r - self.dist_wheel / 2)))
                    theta_right = normalize_90degree(math.atan2(self.dist_axis, (l_o2r + self.dist_wheel / 2)))

                # else: # vel_x == 0
                #     vel_rear_left = 0.
                #     vel_rear_right = 0
                #     vel_front = 0
                #     vel_front_left = 0
                #     vel_front_right = 0
                #     # angle_front = math.atan2((self.vel_theta * self.dist_axis / 2), 1)
                #     angle_front = self.vel_theta
                #     angle_front = limit_45degree(angle_front)
                #     if angle_front == 0:
                #         theta_left = 0
                #         theta_right = 0
                #     else:
                #         l_o2r = self.dist_axis / math.tan(angle_front)
                #
                #         theta_left = normalize_90degree(math.atan2(self.dist_axis, (l_o2r - self.dist_wheel / 2)))
                #         theta_right = normalize_90degree(math.atan2(self.dist_axis, (l_o2r + self.dist_wheel / 2)))
                # rospy.loginfo("Ground truth vel_x = {}, vel_y = {}".format(self.gtruth_vel_x,self.gtruth_vel_y))
                # print("-")
                # tan_beta = math.tan(angle_front)
                # angle_beta = math.atan2(tan_beta/2, 1)
                # vel = self.vel_x/math.cos(angle_beta)
                # vel_x = vel * math.cos(angle_beta + self.gtruth_theta)
                # vel_y = vel * math.sin(angle_beta + self.gtruth_theta)
                # rospy.loginfo("Calculated   vel_x = {}, vel_y = {}".format(vel_x,vel_y))
                # print("-")
                v = (vel_rear_left + vel_rear_right) * 0.5
                if abs(theta_left) <= 1e-5:
                    omega = 0
                else:
                    l_or = (self.dist_axis / math.tan(theta_left)) + self.dist_wheel / 2
                    omega = v / l_or
                rospy.loginfo("Calculated from wheels  v = {}, omega = {}".format(v,omega))
                print("-")
                # rospy.loginfo("Calculate vel_front = {}, vel_front_left = {}, vel_front_right = {}".format(vel_front,
                #                                                                                            vel_front_left,
                #                                                                                            vel_front_right))
                # print("-")
                # rospy.loginfo("Calculate vel_rear_left = {}, vel_rear_right = {}".format(vel_rear_left, vel_rear_right))
                # print("-")
                # rospy.loginfo(
                #     "Calculate angle_front = {}, theta_left = {}, theta_right = {}".format(angle_front, theta_left,
                #                                                                            theta_right))

                self.wheel_vel_left_front = vel_front_left / self.wheel_radius
                self.wheel_vel_right_front = vel_front_right / self.wheel_radius
                self.wheel_vel_left_rear = vel_rear_left / self.wheel_radius
                self.wheel_vel_right_rear = vel_rear_right / self.wheel_radius

                self.wheel_angle_left_front = min(max(theta_left, self.wheel_angle_left_front - self.acc_theta), self.wheel_angle_left_front + self.acc_theta)
                self.wheel_angle_right_front = min(max(theta_right, self.wheel_angle_right_front - self.acc_theta), self.wheel_angle_right_front + self.acc_theta)
                self.wheel_angle_left_rear = min(max(0, self.wheel_angle_left_rear - self.acc_theta), self.wheel_angle_left_rear + self.acc_theta)
                self.wheel_angle_right_rear = min(max(0, self.wheel_angle_right_rear - self.acc_theta), self.wheel_angle_right_rear + self.acc_theta)

            elif self.state == 'translation':
                theta = self.vel_theta
                angular_vel = self.vel_x / self.wheel_radius
                rospy.loginfo("Calculate theta = {}, angular_vel = {}".format(theta, angular_vel))
                print("-")

                if theta > math.pi / 2:
                    theta = -math.pi + theta
                    angular_vel = -angular_vel
                elif theta < -math.pi / 2:
                    theta = math.pi + theta
                    angular_vel = -angular_vel
                rospy.loginfo("Execute theta = {}, angular_vel = {}".format(theta, angular_vel))

                self.wheel_vel_left_front = angular_vel
                self.wheel_vel_right_front = angular_vel
                self.wheel_vel_left_rear = angular_vel
                self.wheel_vel_right_rear = angular_vel

                self.wheel_angle_left_front = min(max(theta, self.wheel_angle_left_front - self.acc_theta), self.wheel_angle_left_front + self.acc_theta)
                self.wheel_angle_right_front = min(max(theta, self.wheel_angle_right_front - self.acc_theta), self.wheel_angle_right_front + self.acc_theta)
                self.wheel_angle_left_rear = min(max(theta, self.wheel_angle_left_rear - self.acc_theta), self.wheel_angle_left_rear + self.acc_theta)
                self.wheel_angle_right_rear = min(max(theta, self.wheel_angle_right_rear - self.acc_theta), self.wheel_angle_right_rear + self.acc_theta)

            elif self.state == 'rotate':
                theta = math.atan2(self.dist_axis, self.dist_wheel)
                radius = math.sqrt(self.dist_axis*self.dist_axis+self.dist_wheel*self.dist_wheel) / 2
                angular_vel = self.vel_theta * radius / self.wheel_radius
                rospy.loginfo("Calculate theta = {}, angular_vel = {}".format(theta, angular_vel))

                self.wheel_vel_left_front = -angular_vel
                self.wheel_vel_right_front = angular_vel
                self.wheel_vel_left_rear = -angular_vel
                self.wheel_vel_right_rear = angular_vel

                self.wheel_angle_left_front = min(max(-theta, self.wheel_angle_left_front - self.acc_theta), self.wheel_angle_left_front + self.acc_theta)
                self.wheel_angle_right_front = min(max(theta, self.wheel_angle_right_front - self.acc_theta), self.wheel_angle_right_front + self.acc_theta)
                self.wheel_angle_left_rear = min(max(theta, self.wheel_angle_left_rear - self.acc_theta), self.wheel_angle_left_rear + self.acc_theta)
                self.wheel_angle_right_rear = min(max(-theta, self.wheel_angle_right_rear - self.acc_theta), self.wheel_angle_right_rear + self.acc_theta)

            elif self.state == 'parking':
                theta = math.pi / 4
                angular_vel = 0
                rospy.loginfo("Calculate theta = {}, angular_vel = {}".format(theta, angular_vel))

                self.wheel_vel_left_front = angular_vel
                self.wheel_vel_right_front = angular_vel
                self.wheel_vel_left_rear = angular_vel
                self.wheel_vel_right_rear = angular_vel

                self.wheel_angle_left_front = min(max(theta, self.wheel_angle_left_front - self.acc_theta), self.wheel_angle_left_front + self.acc_theta)
                self.wheel_angle_right_front = min(max(-theta, self.wheel_angle_right_front - self.acc_theta), self.wheel_angle_right_front + self.acc_theta)
                self.wheel_angle_left_rear = min(max(-theta, self.wheel_angle_left_rear - self.acc_theta), self.wheel_angle_left_rear + self.acc_theta)
                self.wheel_angle_right_rear = min(max(theta, self.wheel_angle_right_rear - self.acc_theta), self.wheel_angle_right_rear + self.acc_theta)

            self.pub_vel_left_rear_wheel.publish(self.wheel_vel_left_rear)
            self.pub_vel_right_rear_wheel.publish(self.wheel_vel_right_rear)
            self.pub_vel_left_front_wheel.publish(self.wheel_vel_left_front)
            self.pub_vel_right_front_wheel.publish(self.wheel_vel_right_front)

            self.pub_pos_left_rear_hinge.publish(self.wheel_angle_left_rear)
            self.pub_pos_right_rear_hinge.publish(self.wheel_angle_right_rear)
            self.pub_pos_left_front_hinge.publish(self.wheel_angle_left_front)
            self.pub_pos_right_front_hinge.publish(self.wheel_angle_right_front)
            print("------------------------------")
            state.data = self.state
            self.state_publisher.publish(state)
            rate.sleep()


def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)

    car = Four_Drive_Car()

    rospy.spin()


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
