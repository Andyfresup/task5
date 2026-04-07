#!/usr/bin/env python3

from enum import IntEnum
import struct
import os
import sys
import time
from threading import Lock, Thread
import ctypes
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from smartcar_control.srv import *
from smartcar_control.msg import *
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool, Float32, Float64, Header, String, Int8
# from ackermann_msgs.msg import AckermannDriveStamped
import math

import tf2_ros
import tf
import tf.transformations as tft

# Add serui module to path
serui_root = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
if serui_root not in sys.path:
    sys.path.insert(0, serui_root)
from serui import *

flag_move = 0


def _str_bytes(bs):
    # return ':'.join(['{:02X}'.format(ord(c)) for c in bs or []])
    return ''.join(['{:02X}'.format(c) for c in bs or []]).encode()


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


class FourDriveCar:
    def __init__(self):
        self._ser_dev = rospy.get_param('~serial_dev', '/dev/ttyUSB0')
        self._ser_baud_rate = rospy.get_param('~serial_baud_rate', 115200)
        self._ser_timeout = rospy.get_param('~serial_timeout', 100)
        self._ser_reconnect_timeout = rospy.get_param('~serial_reconnect_timeout', 5.)
        self.acc_vel = rospy.get_param('/acc_vel', 0.1)
        self.acc_theta = rospy.get_param('/acc_theta', 0.7)
        self.dist_axis = rospy.get_param('/dist_axis', 0.52)
        self.dist_wheel = rospy.get_param('/dist_wheel', 0.35)
        self.wheel_radius = rospy.get_param('/diameter_wheel', 0.17) * 0.5
        self._cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', 1.)
        self._disconnected_as_estop = rospy.get_param('~disconnected_as_estop', False)
        self.update_state_by_imu = rospy.get_param('~update_state_by_imu', False)
        self._revert_z = -1 if rospy.get_param('~revert_z', False) else 1
        self._pub_odom_flag = rospy.get_param('~publish_odom_transform', True)
        self.start_enable = rospy.get_param('~start_enable', True)
        self._last_cmd_vel_time = rospy.Time.now()
        self._cmd_vel_lock = Lock()
        self._enable_cmd_vel = True
        
        self.ser = None
        self.init = False
        self._ser_initialize()
        
        self.cmds = {'enable': dict(periph_id=0x01, reg_addr=0x0204, reg_val=0x0001),
                     'disable': dict(periph_id=0x01, reg_addr=0x0204, reg_val=0x0000),
                     'clear_error': dict(periph_id=0x01, reg_addr=0x0205, reg_val=0x0001),
                     'ackman': dict(periph_id=0x01, reg_addr=0x0201, reg_val=0x0000),
                     'translation': dict(periph_id=0x01, reg_addr=0x0201, reg_val=0x0001),
                     'rotate': dict(periph_id=0x01, reg_addr=0x0201, reg_val=0x0002),
                     'parking': dict(periph_id=0x01, reg_addr=0x0201, reg_val=0x0003),
                     'send_vel': dict(periph_id=0x01, reg_addr_base=0x0202,
                                      reg_vals=[0x0000, 0x0000]),
                     'read_drivers': dict(periph_id=0x01, reg_addr_base=0x0101, nreg=0x0008),
                     'read_state': dict(periph_id=0x01, reg_addr_base=0x0111, nreg=0x0001),
                     'check_motor': dict(periph_id=0x01, reg_addr_base=0x010B, nreg=0x0006),
                     'battery': dict(periph_id=0x01, reg_addr_base=0x00005, nreg=0x0005),
                     'estop': dict(periph_id=0x01, reg_addr_base=0x000A, nreg=0x0001),
                     'imu': dict(periph_id=0x01, reg_addr_base=0x000B, nreg=0x0003),
                     'LED': dict(periph_id=0x01, reg_addr_base=0x0002, nreg=0x0001),
                     'bumper': dict(periph_id=0x01, reg_addr_base=0x0003, nreg=0x0001),
                     'remote_controller': dict(periph_id=0x01, reg_addr_base=0x0004, nreg=0x0001)}
        self._enable_id = 0x0000
        self._enable_update = True
        self._disable_id = 0x0000
        self._disable_update = True
        self._clear_error_id = 0x0000
        self._clear_error_update = True
        self._ackman_id = 0x0000
        self._ackman_update = True
        self._translation_id = 0x0000
        self._translation_update = True
        self._rotate_id = 0x0000
        self._rotate_update = True
        self._send_vel_id = 0x0000
        self._send_vel_update = True
        self._parking_id = 0x0000
        self._parking_update = True
        self._read_state_id = 0x0000
        self._read_state_update = True
        self._check_motor_id = 0x0000
        self._check_motor_update = True
        self._read_drivers_id = 0x0000
        self._read_drivers_update = True
        self._write_drivers_id = 0x0000
        self._write_drivers_update = True
        self._battery_id = 0x0000
        self._battery_update = True
        self._estop_id = 0x0000
        self._estop_update = True
        self._imu_id = 0x0000
        self._imu_update = True
        self._LED_id = 0x0000
        self._LED_update = True
        self._bumper_id = 0x0000
        self._bumper_update = True
        self._remote_controller_id = 0x0000
        self._remote_controller_update = True
        self._set_continuous_id = 0x0000
        self._set_continuous_update = True
        self._ctrl_mode_id = 0x0000
        self._ctrl_mode_update = True
        
        # set_continuous_report(self,trans_id: int, periph_id: int,reg_vals: T.Sequence[int])

        self.trans_id = 0
        self._trans_id_lock = Lock()

        self._msg_motor_error = Bool()
        self._motor_state_lock = Lock()
        self.motor_error = False
        self._pub_motor_error = rospy.Publisher('motor_error', Bool, queue_size=1)

        self.slider_position = 0.0
        self.left_arm_position = 0.0
        self.right_arm_position = 0.0
        
        self.srv_state = rospy.Service('/clear_error', SetBool, self._cb_srv_clear_error)
        self.srv_state = rospy.Service('/state', SetString, self._cb_srv_state)
        self.srv_ctrl_mode = rospy.Service('/ctrl_mode', SetInt, self._cb_srv_ctrl_mode)
        self._srv_enable_cmd_vel = rospy.Service('/enable_cmd_vel', SetBool, self._cb_srv_enable_cmd_vel)
        
        self._sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self._cb_sub_cmd_vel)
        self._sub_drivers = rospy.Subscriber('/drivers_state', DriversInput, self._cb_sub_drivers_state)
        self._pub_drivers = rospy.Publisher('/drivers_state_vals', DriversInput, queue_size=1)
        self.drivers_setting = DriversInput()
        
        self._pub_battery = rospy.Publisher('battery', BatteryState, queue_size=1)
        self.battery = BatteryState()
        self.battery.header.frame_id = 'battery'
        self.battery.header.stamp = rospy.Time.now()
        
        self._pub_img = rospy.Publisher('imu', Imu, queue_size=1)
        self.msg_img = Imu()
        self.msg_img.header.frame_id = 'imu_link'
        self.msg_img.orientation.w = 1.
        
        self.LED = Int8()
        self._pub_LED = rospy.Publisher('LED', Int8, queue_size=1)
        self.bumper = Int8()
        self._pub_bumper = rospy.Publisher('bumper', Int8, queue_size=1)
        self.remote_ctrl = Int8()
        self._pub_remote_ctrl = rospy.Publisher('remote_ctrl', Int8, queue_size=1)
        
        self._pub_base_state = rospy.Publisher('base_state', State, queue_size=1)
        
        self._pub_base_ctrl_mode = rospy.Publisher('/base_ctrl_mode', String, queue_size=1)
        
        if self._disconnected_as_estop:
            self._msg_estop = Bool()
            self._pub_estop = rospy.Publisher('estop', Bool, queue_size=1)
        
        self.state_list = ['ackman', 'translation', 'rotate', 'parking']
        self.state = self.state_list[0]
        self.ctrl_mode = 1
        self.ctrl_mode_error = False
        self.state_error = False
        self.on_remote_ctrl = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_theta = 0.0
        self.gtruth_vel_x = 0.0
        self.gtruth_vel_y = 0.0
        self.gtruth_vel_theta = 0.0
        self.gtruth_theta = 0.0
        self._state_x = 0.0
        self._state_y = 0.0
        self._state_d = 0.0
        self._state_vx = 0.0
        self._state_vy = 0.0
        self._state_vd = 0.0

        self._pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_received_stamp = None
        self.tf_publisher = tf2_ros.TransformBroadcaster()
        self._pub_odom_flag = True
        self._msg_odom = Odometry()
        self._msg_odom.header.frame_id = 'odom'
        self._msg_odom.child_frame_id = rospy.get_param('~link_name', 'base_footprint')
        self._msg_odom.pose.covariance[0] = .1
        self._msg_odom.pose.covariance[7] = .1
        self._msg_odom.pose.covariance[35] = 0.05
        self._msg_odom.pose.covariance[14] = 1e10
        self._msg_odom.pose.covariance[21] = 1e10
        self._msg_odom.pose.covariance[28] = 1e10

        self._tf = tf.TransformBroadcaster()
        self._tf_odom = TransformStamped()
        self._tf_odom.header.frame_id = 'odom'
        self._tf_odom.child_frame_id = rospy.get_param('~link_name', 'base_footprint')
        
        self._th_poll = None
        # self._th_poll = Thread(target=self.ser.poll)
        # self._th_poll = Thread(target=self._poll_loop)
        # self._th_poll.daemon = True
        # self._th_run = Thread(target=self.run)
        # self._th_run.start()

        self._last_ser_read = rospy.Time.now()
        self._last_update_state = rospy.Time.now()
        self.start_enable = True

    def _ser_initialize(self):
        self.ser = SerUiCl(port=self._ser_dev, baud_rate=self._ser_baud_rate, timeout=self._ser_timeout,
                          inter_byte_timeout=None, rx_buf_size=64, pdu_len_hint=128, no_response_mode=False)
        
        self.ser.register_callback_on_error_response(self.on_error_response)
        self.ser.register_callback_on_read_holding_reg(self.on_read_holding_reg)
        self.ser.register_callback_on_read_input_reg(self.on_read_input_reg)
        self.ser.register_callback_on_write_single_reg(self.on_write_single_reg)
        self.ser.register_callback_on_write_multi_reg(self.on_write_multi_reg)
        self.ser.register_callback_on_continuous_report(self.on_continuous_report)
        self.ser.register_callback_on_recv_bytes(self.on_recv_bytes)
        self.ser.register_callback_on_log_string(self.on_log_string)
        # self.ser.register_callback_on_version(self.on_version)
        
    def _poll_loop(self):
        while not rospy.is_shutdown():
            self.ser.pollOnce()
            time.sleep(0.02)
    
    def _cb_srv_enable_cmd_vel(self, req):
        self._enable_cmd_vel = req.data
        rospy.loginfo('cmd_vel is ' + ('enabled' if self._enable_cmd_vel else 'disabled'))
        return SetBoolResponse(True, '')

    def _cb_srv_ctrl_mode(self, data):
        rospy.logerr('_cb_srv_ctrl_mode')
        self.ctrl_mode = data.data
        self._write_ctrl_mode(self.ctrl_mode)
        time.sleep(0.05)
        self._set_continuous_report(True)
        return SetIntResponse(True, '')
    
    def _cb_srv_clear_error(self, data):
        if data.data:
            self._write_cmd('clear_error')
            time.sleep(0.05)
            self.enable(True)
            time.sleep(0.05)
        return SetBoolResponse(True, '')

    def _cb_sub_cmd_vel(self, data):
        with self._cmd_vel_lock:
            # rospy.loginfo("Get Twist x = {}, y = {}, theta = {}".format(data.linear.x,data.linear.y,data.angular.z*180/math.pi))
            self.vel_x = min(max(data.linear.x, self.vel_x - self.acc_vel), self.vel_x + self.acc_vel)
            self.vel_y = min(max(data.linear.y, self.vel_y - self.acc_vel), self.vel_y + self.acc_vel)
            self.vel_theta = min(max(data.angular.z, self.vel_theta - self.acc_theta), self.vel_theta + self.acc_theta)
            self._last_cmd_vel_time = rospy.Time.now()
        
    def _cb_sub_drivers_state(self, data):
        self.drivers_setting = data
    
    def publish_odom(self, curr_time):
        q = tft.quaternion_from_euler(0, 0, self._state_d)
        seq = self._msg_odom.header.seq
        seq += 1

        self._msg_odom.header.stamp = curr_time
        self._msg_odom.header.seq = seq
        self._msg_odom.twist.twist.linear.x = self._state_vx
        self._msg_odom.twist.twist.linear.y = self._state_vy
        self._msg_odom.twist.twist.angular.z = self._state_vd
        self._msg_odom.pose.pose.position.x = self._state_x
        self._msg_odom.pose.pose.position.y = self._state_y
        self._msg_odom.pose.pose.orientation.x = q[0]
        self._msg_odom.pose.pose.orientation.y = q[1]
        self._msg_odom.pose.pose.orientation.z = q[2]
        self._msg_odom.pose.pose.orientation.w = q[3]
        if self._pub_odom_flag:
            self._pub_odom.publish(self._msg_odom)

        self._tf_odom.header.stamp = curr_time
        self._tf_odom.header.seq = seq
        self._tf_odom.transform.translation.x = self._state_x
        self._tf_odom.transform.translation.y = self._state_y
        self._tf_odom.transform.rotation.x = q[0]
        self._tf_odom.transform.rotation.y = q[1]
        self._tf_odom.transform.rotation.z = q[2]
        self._tf_odom.transform.rotation.w = q[3]
        self._tf.sendTransformMessage(self._tf_odom)

    def _cb_srv_state(self, data):
        if self.ctrl_mode == 1:
            if data.data in self.state_list:
                self.state = data.data
                rospy.loginfo('Switch to state {}'.format(data.data))
                self._write_cmd(self.state)
                return SetStringResponse(True, '')
            else:
                rospy.logerr('No such state {}'.format(data.data))
                return SetStringResponse(False, 'No such state {}'.format(data.data))
        else:
            rospy.logerr('Control as mode {}, can not switch state'.format(self.ctrl_mode))
            return SetStringResponse(False, 'Control as mode {}, can not switch state'.format(self.ctrl_mode))

    def _set_continuous_report(self, run):
        with self._trans_id_lock:
            self.trans_id = ((self.trans_id + 1) & 0xFFFF)
            if self.trans_id - self._set_continuous_id >= 10:
                self._set_continuous_update = True
        if run:
            if self._set_continuous_update:
                self._set_continuous_id = self.trans_id
                self._set_continuous_update = False
            if self.ctrl_mode == 1:
                self.ser.set_continuous_report(trans_id=self.trans_id, periph_id=0x01,
                                reg_vals=[0x0002])
            else:
                self.ser.set_continuous_report(trans_id=self.trans_id, periph_id=0x01,
                                reg_vals=[0x0003])
        else:
            self.ser.set_continuous_report(trans_id=self.trans_id, periph_id=0x01,
                                reg_vals=[0x0000])
            
    def _write_cmd(self, cmd):
        # print('Write cmd: {}'.format(cmd))
        with self._trans_id_lock:
            self.trans_id = ((self.trans_id + 1) & 0xFFFF)
            
        if cmd == 'send_vel':
            if self.trans_id - self._send_vel_id >= 10:
                self._send_vel_update = True
            if self._send_vel_update:
                self._send_vel_id = self.trans_id
                self._send_vel_update = False
            ret = self.ser.write_multi_reg(trans_id=self.trans_id,
                                       periph_id=self.cmds[cmd]['periph_id'],
                                       reg_addr_base=self.cmds[cmd]['reg_addr_base'],
                                       reg_vals=self.cmds[cmd]['reg_vals'])
            
        elif cmd in ['read_state', 'check_motor', 'read_drivers', 'battery', 'estop', 'imu', 'LED', 'bumper','remote_controller']:
            if cmd == 'read_state':
                if self.trans_id - self._read_state_id >= 10:
                    self._read_state_update = True
                if self._read_state_update:
                    self._read_state_id = self.trans_id
                    self._read_state_update = False
            if cmd == 'check_motor':
                if self.trans_id - self._check_motor_id >= 10:
                    self._check_motor_update = True
                if self._check_motor_update:
                    self._check_motor_id = self.trans_id
                    self._check_motor_update = False
            if cmd == 'read_drivers':
                if self.trans_id - self._read_drivers_id >= 10:
                    self._read_drivers_update = True
                if self._read_drivers_update:
                    self._read_drivers_id = self.trans_id
                    self._read_drivers_update = False
            if cmd == 'battery':
                if self.trans_id - self._battery_id >= 10:
                    self._battery_update = True
                if self._battery_update:
                    self._battery_id = self.trans_id
                    self._battery_update = False
            if cmd == 'estop':
                if self.trans_id - self._estop_id >= 10:
                    self._estop_update = True
                if self._estop_update:
                    self._estop_id = self.trans_id
                    self._estop_update = False
            if cmd == 'imu':
                if self.trans_id - self._imu_id >= 10:
                    self._imu_update = True
                if self._imu_update:
                    self._imu_id = self.trans_id
                    self._imu_update = False
            if cmd == 'LED':
                if self.trans_id - self._LED_id >= 10:
                    self._LED_update = True
                if self._LED_update:
                    self._LED_id = self.trans_id
                    self._LED_update = False
            if cmd == 'bumper':
                if self.trans_id - self._bumper_id >= 10:
                    self._bumper_update = True
                if self._bumper_update:
                    self._bumper_id = self.trans_id
                    self._bumper_update = False
            if cmd == 'remote_controller':
                if self.trans_id - self._remote_controller_id >= 10:
                    self._remote_controller_update = True
                if self._remote_controller_update:
                    self._remote_controller_id = self.trans_id
                    self._remote_controller_update = False
            
            ret = self.ser.read_input_reg(trans_id=self.trans_id,
                                      periph_id=self.cmds[cmd]['periph_id'],
                                      reg_addr_base=self.cmds[cmd]['reg_addr_base'],
                                      nreg=self.cmds[cmd]['nreg'])
        
        else:
            if cmd == 'enable':
                if self.trans_id - self._enable_id >= 10:
                    self._enable_update = True
                if self._enable_update:
                    self._enable_id = self.trans_id
                    self._enable_update = False
            if cmd == 'disable':
                if self.trans_id - self._disable_id >= 10:
                    self._disable_update = True
                if self._disable_update:
                    self._disable_id = self.trans_id
                    self._disable_update = False
            if cmd == 'clear_error':
                if self.trans_id - self._clear_error_id >= 10:
                    self._clear_error_update = True
                if self._clear_error_update:
                    self._clear_error_id = self.trans_id
                    self._clear_error_update = False
            if cmd == 'ackman':
                if self.trans_id - self._ackman_id >= 10:
                    self._ackman_update = True
                if self._ackman_update:
                    self._ackman_id = self.trans_id
                    self._ackman_update = False
            if cmd == 'translation':
                if self.trans_id - self._translation_id >= 10:
                    self._translation_update = True
                if self._translation_update:
                    self._translation_id = self.trans_id
                    self._translation_update = False
            if cmd == 'rotate':
                if self.trans_id - self._rotate_id >= 10:
                    self._rotate_update = True
                if self._rotate_update:
                    self._rotate_id = self.trans_id
                    self._rotate_update = False
            if cmd == 'parking':
                if self.trans_id - self._parking_id >= 10:
                    self._parking_update = True
                if self._parking_update:
                    self._parking_id = self.trans_id
                    self._parking_update = False
            if cmd == 'ctrl_mode':
                if self.trans_id - self._ctrl_mode_id >= 10:
                    self._ctrl_mode_update = True
                if self._ctrl_mode_update:
                    self._ctrl_mode_id = self.trans_id
                    self._ctrl_mode_update = False
            ret = self.ser.write_single_reg(trans_id=self.trans_id,
                                        periph_id=self.cmds[cmd]['periph_id'],
                                        reg_addr=self.cmds[cmd]['reg_addr'],
                                        reg_val=self.cmds[cmd]['reg_val'])
        # print(ret)

    def _write_ctrl_mode(self, mode):
        with self._trans_id_lock:
            self.trans_id = ((self.trans_id + 1) & 0xFFFF)
            if self.trans_id - self._ctrl_mode_id >= 10:
                self._ctrl_mode_update = True
        if self._ctrl_mode_update:
            self._ctrl_mode_id = self.trans_id
            self._ctrl_mode_update = False
        # print('self._ctrl_mode_id = {}'.format(self._ctrl_mode_id))
        ret = self.ser.write_single_reg(trans_id=self.trans_id,
                                        periph_id=0x01,
                                        reg_addr=0x000B,
                                        reg_val=mode)
        
        # self.ser.write_multi_reg(trans_id=self.trans_id,
        #                          periph_id=0x01,
        #                          reg_addr_base=0x000B,
        #                          reg_vals=[mode])

    def _write_vel(self, vel):
        with self._trans_id_lock:
            self.trans_id = ((self.trans_id + 1) & 0xFFFF)
            if self.trans_id - self._send_vel_id >= 10:
                self._send_vel_update = True
        if self._send_vel_update:
            self._send_vel_id = self.trans_id
            self._send_vel_update = False
            
        self.ser.write_multi_reg(trans_id=self.trans_id,
                                 periph_id=self.cmds['send_vel']['periph_id'],
                                 reg_addr_base=self.cmds['send_vel']['reg_addr_base'],
                                 reg_vals=vel)
        
    def _write_4wheel_drivers(self, vals):
        with self._trans_id_lock:
            self.trans_id = ((self.trans_id + 1) & 0xFFFF)
            if self.trans_id - self._write_drivers_id >= 10:
                self._write_drivers_update = True
        if self._write_drivers_update:
            self._write_drivers_id = self.trans_id
            self._write_drivers_update = False
            
        self.ser.write_multi_reg(trans_id=self.trans_id,
                                 periph_id=0x01,
                                 reg_addr_base=0x0101,
                                 reg_vals=vals)

    def _check_motor(self, data):
        motors = ['front_left_vel', 'front_right_vel', 'rear_left_vel', 'rear_right_vel', 'front_left_angle', 'front_right_angle', 'rear_left_angle', 'rear_right_angle']
        for i in range(len(data)):
            error_msg = []
            if i <= 3:
                if (data[i] & 0b0000000000000001) == 0b0000000000000001:  # Bit 0
                    error_msg.append("Power Low")
                if (data[i] & 0b0000000000000010) == 0b0000000000000010:  # Bit 1
                    error_msg.append("Position Error")
                if (data[i] & 0b0000000000000100) == 0b0000000000000100:  # Bit 2
                    error_msg.append("Hall Feedback Error")
                if (data[i] & 0b0000000000001000) == 0b0000000000001000:  # Bit 3
                    error_msg.append("Current Overflow")
                if (data[i] & 0b0000000000010000) == 0b0000000000010000:  # Bit 4
                    error_msg.append("Driver Overload")
                if (data[i] & 0b0000000000100000) == 0b0000000000100000:  # Bit 5
                    error_msg.append("EEPROM Error")
                if (data[i] & 0b0000000001000000) == 0b0000000001000000:  # Bit 6
                    error_msg.append("IGBT Error")
                if (data[i] & 0b0000000010000000) == 0b0000000010000000:  # Bit 7
                    error_msg.append("Driver Overheating")
                if (data[i] & 0b0000000100000000) == 0b0000000100000000:  # Bit 8
                    error_msg.append("Motor Phase Loss")
                if (data[i] & 0b0000001000000000) == 0b0000001000000000:  # Bit 9
                    error_msg.append("Motor Current Exceeds The Tolerance")
                if (data[i] & 0b0000010000000000) == 0b0000010000000000:  # Bit 10
                    error_msg.append("Motor Speed Exceeds The Tolerance")
                if (data[i] & 0b0000100000000000) == 0b0000100000000000:  # Bit 11
                    error_msg.append("Motor Overheating")
                if (data[i] & 0b0001000000000000) == 0b0001000000000000:  # Bit 12
                    error_msg.append("Power Overload")
                # if (data[i] & 0b0010000000000000) == 0b0010000000000000:  # Bit 13
                #     error_msg += "Bit 13 temperature error"
                # if (data[i] & 0b0100000000000000) == 0b0100000000000000:  # Bit 14
                #     error_msg += "Bit 14 find motor error"
                if (data[i] & 0b1000000000000000) == 0b1000000000000000:  # Bit 15
                    error_msg.append("Locked")
                    
                if len(error_msg) > 0:
                    self.motor_error = True
                    rospy.logerr("Motor {} Error. Error Message: {}".format(motors[i], error_msg))
            else:
                # print(data[i]>>4)
                for k in range(2):
                    error_msg = []
                    if k==0:
                        val = (data[i]>>4) & 0x11
                    else:
                        val = data[i] & 0x11
                    
                    if val == 0:
                        error_msg.append("Motor Disabled")
                    if val == 8:
                        error_msg.append("Motor Overpressure")
                    if val == 9:
                        error_msg.append("Motor Undervoltage")
                    if val == 10:
                        error_msg.append("Motor Current Overflow")
                    if val == 11:
                        error_msg.append("Motor MOS Overheating")
                    if val == 12:
                        error_msg.append("Motor Coil Overheating")
                    if val == 13:
                        error_msg.append("Motor Loss Connection")
                    if val == 14:
                        error_msg.append("Motor Overload")
                    print('check value={}'.format(val))
                    if len(error_msg) > 0:
                        self.motor_error = True
                        if i == 5:
                            rospy.logerr("Motor {} Error. Error Message: {}".format(motors[i+k+1], error_msg))
                        else:
                            rospy.logerr("Motor {} Error. Error Message: {}".format(motors[i+k], error_msg))

    def on_recv_bytes(self, data):
        # a = ''.join(['{:02X}'.format(ord(i)) for i in data])
        # a = ''.join(['{:02X}'.format(i) for i in data])
        # print('on_recv_bytes: {}'.format(a))
        self._last_ser_read = rospy.Time.now()

    def on_version(self, *args):
        print('version')
        self._last_ser_read = rospy.Time.now()
       
    def on_log_string(self, trans_id, periph_id, log_str):
        # print('------------------------on_log_string------------------------')
        # # print(log_str)
        # rospy.logdebug(log_str)
        # print('------------------------end_log_string------------------------')
        self._last_ser_read = rospy.Time.now() 

    def on_error_response(self,
                          trans_id, periph_id,
                          fcode, err_code):
        print('on_error_response')
        error_cmd = ''
        if trans_id == self._enable_id:
            error_cmd = 'enable'
            self._enable_update = True
        if trans_id == self._disable_id:
            error_cmd = 'disable'
            self._disable_update = True
        if trans_id == self._clear_error_id:
            error_cmd = 'clear_error'
            self._clear_error_update = True
        if trans_id == self._ackman_id:
            error_cmd = 'ackman'
            self._ackman_update = True
        if trans_id == self._translation_id:
            error_cmd = 'translation'
            self._translation_update = True
        if trans_id == self._rotate_id:
            error_cmd = 'rotate'
            self._rotate_update = True
        if trans_id == self._send_vel_id:
            error_cmd = 'send_vel'
            self._send_vel_update = True
        if trans_id == self._parking_id:
            error_cmd = 'parking'
            self._parking_update = True
        if trans_id == self._read_state_id:
            error_cmd = 'read_state'
            self._read_state_update = True
        if trans_id == self._check_motor_id:
            error_cmd = 'check_motor'
            self._check_motor_update = True
        if trans_id == self._read_drivers_id:
            error_cmd = 'read_drivers'
            self._read_drivers_update = True
        if trans_id == self._write_drivers_id:
            error_cmd = 'write_drivers'
            self._write_drivers_update = True
        if trans_id == self._battery_id:
            error_cmd = 'battery'
            self._battery_update = True
        if trans_id == self._estop_id:
            error_cmd = 'estop'
            self._estop_update = True
        if trans_id == self._imu_id:
            error_cmd = 'imu'
            self._imu_update = True
        if trans_id == self._LED_id:
            error_cmd = 'LED'
            self._LED_update = True
        if trans_id == self._bumper_id:
            error_cmd = 'bumper'
            self._bumper_update = True
        if trans_id == self._remote_controller_id:
            error_cmd = 'remote_controller'
            self._remote_controller_update = True
        if trans_id == self._set_continuous_id:
            error_cmd = 'set_continuous'
            self._set_continuous_update = True
        if trans_id == self._ctrl_mode_id:
            error_cmd = 'ctrl_mode'
            self._ctrl_mode_update = True
        
        self._last_ser_read = rospy.Time.now()
        rospy.logerr('Error on cmd: {}, error_code = {}'.format(error_cmd, err_code))

    def on_read_holding_reg(self,
                            trans_id, periph_id,
                            reg_vals):
        # print('on_read_holding_reg')
        self._last_ser_read = rospy.Time.now()
        # print(reg_vals)

    def on_read_input_reg(self,
                          trans_id, periph_id,
                          reg_vals):
        print('on_read_input_reg: id = {}, reg_vals = {}'.format(trans_id,reg_vals))
        if trans_id == self._read_state_id:
            print('read state')
            self._get_state(reg_vals)
            self._read_state_update = True
            
        elif trans_id == self._check_motor_id:
            print('Check motor: {}'.format(reg_vals))
            self._check_motor(reg_vals)
            self._check_motor_update = True
            
        elif trans_id == self._read_drivers_id:
            print('read_drivers')
            self._publish_drivers(reg_vals)
            self._read_drivers_update = True
            
        if trans_id == self._battery_id :
            print('read battery')
            self._publish_battery(reg_vals)
            self._battery_update = True
            
        if trans_id == self._estop_id:
            print('read estop')
            self._msg_estop.data = True if reg_vals[0]==1 else False
            if self._msg_estop.data:
                rospy.logwarn_throttle(1.0, 'Estop is running')
            self._pub_estop.publish(self._msg_estop)
            self._estop_update = True
            
        # if trans_id == self._imu_id:
        #     self._publish_imu(reg_vals)
            
        # if trans_id == self._LED_id:
        #     self._publish_LED(reg_vals)
            
        # if trans_id == self._bumper_id:
        #     self.bumper.data = reg_vals[0]
        #     self._pub_bumper.publish(self.bumper)
            
        if trans_id == self._remote_controller_id:
            print('read remote_controller')
            self.remote_ctrl.data = reg_vals[0]
            self._pub_remote_ctrl.publish(self.remote_ctrl)
            if self.remote_ctrl.data == 1:
                self.on_remote_ctrl = True
            else:
                self.on_remote_ctrl = False
            self._remote_controller_update = True
            
        self._last_ser_read = rospy.Time.now()

    def on_write_single_reg(self,
                            trans_id, periph_id,
                            reg_addr, reg_val):
        print('on_write_single_reg: {}'.format(reg_val))
        if trans_id == self._enable_id:
            print('enable: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._enable_update = True
        if trans_id == self._disable_id:
            print('disable: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._disable_update = True
        if trans_id == self._clear_error_id:
            print('clear error: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._clear_error_update = True
        if trans_id == self._ackman_id:
            print('ackman: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._ackman_update = True
        if trans_id == self._translation_id:
            print('translation: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._translation_update = True
        if trans_id == self._rotate_id:
            print('rotate: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._rotate_update = True
        if trans_id == self._parking_id:
            print('parking: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._parking_update = True
        if trans_id == self._ctrl_mode_id:
            print('ctrl_mode: reg_addr = {}, reg_val = {}'.format(reg_addr, reg_val))
            self._ctrl_mode_update = True
            
        self._last_ser_read = rospy.Time.now()

    def on_write_multi_reg(self,
                           trans_id, periph_id,
                           reg_addr_base, nreg):
        print('on_write_multi_reg')
        if trans_id == self._send_vel_id:
            print('send_vel')
            self._send_vel_update = True
        if trans_id == self._write_drivers_id:
            print('write_drivers')
            self._write_drivers_update = True
        if trans_id == self._ctrl_mode_id:
            print('ctrl_mode')
            self._ctrl_mode_update = True
        self._last_ser_read = rospy.Time.now()

    def on_continuous_report(self,
                             trans_id, periph_id,
                             ret_bytes):
        self._set_continuous_update = True
        print('-------------------on_continuous_report------------------------------')
        # print(''.join(['{:02X}'.format(ord(i)) for i in ret_bytes]))
        # print(ret_bytes)
        data_imu = struct.unpack('<12h', ret_bytes[:24])
        data_motor, = struct.unpack('>h', ret_bytes[24:26])
        data_mode, = struct.unpack('>h', ret_bytes[26:28])
        # print(data_motor)
        # print(data_mode)
        
        if data_mode != self.ctrl_mode:
            self.ctrl_mode_error = True
        elif data_mode==1 and len(ret_bytes[28:])!= 8:
            self.ctrl_mode_error = True
        elif data_mode==0 and len(ret_bytes[28:])!= 16:
            self.ctrl_mode_error = True
            # rospy.logwarn('data_mode != self.ctrl_mode')
            # self._write_ctrl_mode(self.ctrl_mode)
            # # time.sleep(0.05)
            # self._set_continuous_report(True)
            # # time.sleep(0.05)
        else:
            self.ctrl_mode_error = False
            
        if not self.ctrl_mode_error:
            if data_mode == 1:
                data_state = struct.unpack('>4h', ret_bytes[28:])
            else:
                # print(''.join(['{:02X}'.format(ord(i)) for i in ret_bytes[28:]]))
                print(''.join(['{:02X}'.format(i) for i in ret_bytes[28:]]))
                data_state = struct.unpack('>8h', ret_bytes[28:])
            # print(data_state)
            
            if data_mode == 1:
                print('mode state: {}'.format(data_state))
                self._get_state(data_state)
            else:
                print('mode drivers: {}'.format(data_state))
                self._publish_drivers(data_state)
        # else:
            # rospy.logwarn('data_mode != self.ctrl_mode')
            # self._write_ctrl_mode(self.ctrl_mode)
            # time.sleep(0.05)
            # self._set_continuous_report(True)
            # time.sleep(0.05)
        
        if data_motor == 1:
            self.motor_error = True
            # rospy.logwarn('check_motor')
            # self._write_cmd('check_motor')
            # # time.sleep(0.5)
        else:
            self.motor_error = False
        
        self._publish_imu(data_imu)
        # print('-----------------------------------------------')
        self._last_ser_read = rospy.Time.now()
        
    def _get_state(self, reg_vals):
        base_state = State()
        base_state.code = reg_vals[0]
        base_state.name = self.state_list[base_state.code]
        
        if self.state != self.state_list[base_state.code]:
            rospy.logerr("Robot state {} not same with set state {}".format(self.state_list[base_state.code], self.state))
            self.state_error = True
            # self._write_cmd(self.state)
        else:
            self.state_error = False
            
        if not self.state_error:
            print('state {}'.format(base_state.name))
            
            vel_x = ctypes.c_int16(reg_vals[1]).value * 0.01
            # print('x {}'.format(vel_x))
            beta = ctypes.c_int16(reg_vals[2]).value * 0.001
            # print('beta {}'.format(beta))
            omega = ctypes.c_int16(reg_vals[3]).value * 0.001
            # print('omega {}'.format(omega))
            vel = vel_x / math.cos(beta)
            print('x = {}, beta = {}, omega = {}, vel = {}'.format(vel_x, beta, omega, vel))
            
            self._pub_base_state.publish(base_state)
            self._update_state(vel, omega, beta)
      
    def _update_state(self, vel, omega, beta):
        curr_time = rospy.Time.now()
        dt = (curr_time - self._last_update_state).to_sec()
        
        if self.state == 'translation':
            self._state_vx = vel * math.cos(self._state_d + omega)
            self._state_vy = vel * math.sin(self._state_d + omega)
            self._state_vd = 0.0
            self._state_x += self._state_vx * dt
            self._state_y += self._state_vy * dt
        else:
            self._state_vx = vel * math.cos(self._state_d + beta)
            self._state_vy = vel * math.sin(self._state_d + beta)
            self._state_vd = self.msg_img.angular_velocity.z if self.update_state_by_imu else omega
            self._state_x += self._state_vx * dt
            self._state_y += self._state_vy * dt
            self._state_d += self.msg_img.angular_velocity.z * dt if self.update_state_by_imu else omega * dt
            
        self._last_update_state = curr_time
    
    def _publish_drivers(self, reg_vals):
        drivers = DriversInput()
        drivers.front_left_vel = -ctypes.c_int16(reg_vals[0]).value * 0.01
        drivers.front_right_vel = ctypes.c_int16(reg_vals[1]).value * 0.01
        drivers.rear_left_vel = -ctypes.c_int16(reg_vals[2]).value * 0.01
        drivers.rear_right_vel = ctypes.c_int16(reg_vals[3]).value * 0.01
        drivers.front_left_angle = ctypes.c_int16(reg_vals[4]).value * 0.001
        drivers.front_right_angle = ctypes.c_int16(reg_vals[5]).value * 0.001
        drivers.rear_left_angle = ctypes.c_int16(reg_vals[6]).value * 0.001
        drivers.rear_right_angle = ctypes.c_int16(reg_vals[7]).value * 0.001
        self._pub_drivers.publish(drivers)
    
    def _publish_imu(self, data):
        # print(reg_vals)
        self.msg_img.header.stamp = rospy.Time.now()
        
        self.msg_img.linear_acceleration.x = (float(data[0]) * 16 * 9.8) / 32768
        self.msg_img.linear_acceleration.y = (float(data[1]) * 16 * 9.8) / 32768
        self.msg_img.linear_acceleration.z = (float(data[2]) * 16 * 9.8) / 32768
        
        self.msg_img.angular_velocity.x = (float(data[4]) * 2000 * (math.pi / 180)) / 32768
        self.msg_img.angular_velocity.y = (float(data[5]) * 2000 * (math.pi / 180)) / 32768
        self.msg_img.angular_velocity.z = (float(data[6]) * 2000 * (math.pi / 180)) / 32768

        self.msg_img.orientation.x = float(data[8]) / 32768  # q[0] 
        self.msg_img.orientation.y = float(data[9]) / 32768  # q[1]
        self.msg_img.orientation.z = float(data[10]) / 32768  # q[2]
        self.msg_img.orientation.w = float(data[11]) / 32768  # q[3]

        self._pub_img.publish(self.msg_img)
        
    def _publish_battery(self, reg_vals):
        self.battery.voltage = reg_vals[0] * 0.01
        self.battery.current = ctypes.c_int16(reg_vals[1]).value * 0.01
        self.battery.capacity = reg_vals[2] * 0.01
        self.battery.design_capacity = reg_vals[3] * 0.01
        self.battery.percentage = self.battery.capacity / self.battery.design_capacity
        self.battery.percentage = 1 if self.battery.percentage >= 1 else self.battery.percentage
        self.battery.header.stamp = rospy.Time.now()
        self._pub_battery.publish(self.battery)
        
    def _publish_LED(self, reg_vals):
        print(reg_vals)
        self._pub_LED.publish(self.LED)

    def connect(self):
        self.close()
        time.sleep(.5)
        # self.ser = SerUiCl(port=self._ser_dev, baud_rate=self._ser_baud_rate, timeout=self._ser_timeout,
        #                   inter_byte_timeout=None, rx_buf_size=64, pdu_len_hint=128, no_response_mode=False)
        
        self.ser.open()
        time.sleep(0.05)
        if not self.ser.isOpen():
            return False
        
        self.ser.stop = False
        self._th_poll = Thread(target=self.ser.poll)
        self._th_poll.daemon = True
        self._th_poll.start()
        
        self._set_continuous_report(False)
        time.sleep(0.05)
        
        self.enable(True)
        time.sleep(0.05)
        self._write_cmd('clear_error')
        time.sleep(0.05)
        self._write_ctrl_mode(1)
        time.sleep(0.05)
        self._write_cmd('send_vel')
        time.sleep(0.05)
        self._set_continuous_report(True)
        time.sleep(0.1)
        
        self._state_vx = self._state_vd = 0

        self._last_ser_read = rospy.Time.now()
        rospy.loginfo_throttle(10., 'success to connect to serial "{}"'.format(self._ser_dev))

        return True

    def close(self):
        if self._th_poll is not None:
            self.ser.stop = True
            self.ser.close()
            self._th_poll.join()
            th = self._th_poll
            self._th_poll = None
        rospy.loginfo_throttle(10., 'close the connection to serial "{}"'.format(self._ser_dev))

    def end(self):
        self._write_ctrl_mode(1)
        time.sleep(0.05)
        self._write_cmd('send_vel')
        time.sleep(0.1)
        # set drivers 0
        self._write_4wheel_drivers([0,0,0,0,0,0,0,0])
        time.sleep(0.1)
        self._set_continuous_report(False)
        time.sleep(0.1)
        self.enable(False)
        self.ser.stop = True
    
    def enable(self, enable=True):
        if not self.start_enable:
            return
        if enable:
            self._write_cmd('enable')
        else:
            self._write_cmd('disable')

    def run(self):
        rate = rospy.Rate(20)

        while not self.connect() and not rospy.is_shutdown():
            time.sleep(0.05)
            rospy.logwarn('reconnect to serial "{}"'.format(self._ser_dev))
            
        last_enable_cmd_vel = True
        battery_timer = rospy.Time.now()
        last_ctrl_mode = 1
        one_second_timer = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                # print('------------------------')
                rate.sleep()
                curr_time = rospy.Time.now()
                
                if self.state_error:
                    rospy.logwarn('state error')
                    self._write_cmd(self.state)
                    time.sleep(0.05)
                
                if self.motor_error:
                    rospy.logwarn('check_motor')
                    self._write_cmd('check_motor')
                    time.sleep(0.05)
                    
                if self.ctrl_mode_error:
                    rospy.logwarn('data_mode != self.ctrl_mode')
                    self._write_ctrl_mode(self.ctrl_mode)
                    time.sleep(0.05)
                    self._set_continuous_report(True)
                    time.sleep(0.05)
                
                if (curr_time - self._last_ser_read).to_sec() > self._ser_reconnect_timeout:
                    self.close()
                    time.sleep(0.05)
                    while not self.connect() and not rospy.is_shutdown():
                        time.sleep(0.05)
                        
                        rospy.logwarn_throttle(10., 'reconnect to serial "{}"'.format(self._ser_dev))

                    if self._disconnected_as_estop:
                        self._msg_estop.data = True
                        _estop_timer = self._last_ser_read
                
                
                # if last_ctrl_mode != self.ctrl_mode:
                #     self._write_cmd('send_vel')
                #     time.sleep(0.05)
                #     self._write_4wheel_drivers([0,0,0,0,0,0,0,0])
                #     time.sleep(2.)
                #     last_ctrl_mode = self.ctrl_mode
                
                if self.ctrl_mode==1:
                    with self._cmd_vel_lock:
                        if (curr_time - self._last_cmd_vel_time).to_sec() >= self._cmd_vel_timeout or self.motor_error:
                            vel_x = self.vel_x = 0.
                            vel_y = self.vel_y = 0.
                            vel_theta = self.vel_theta = 0.
                        else:
                            vel_x = self.vel_x
                            vel_y = self.vel_y
                            vel_theta = self.vel_theta

                        vel_x = int(vel_x * 100) & 0xFFFF
                        vel_y = int(vel_y * 100) & 0xFFFF
                        vel_theta = int(vel_theta * 1000) & 0xFFFF
                
                    _enable_cmd_vel = self._enable_cmd_vel
                    if _enable_cmd_vel:
                        if not last_enable_cmd_vel and not self.motor_error:
                            self.enable(True)
                            time.sleep(0.05)
                        
                        if not self.on_remote_ctrl:
                            self._write_vel([vel_x, vel_theta])
                            print('write speed v = {}, omega = {}'.format(vel_x, vel_theta))
                            time.sleep(0.05)
                
                    elif last_enable_cmd_vel and not self.motor_error:
                        self.enable(False)
                        time.sleep(0.05)
                
                    last_enable_cmd_vel = _enable_cmd_vel
                    # self._write_cmd('read_state')
                    # time.sleep(0.05)
                    self.publish_odom(curr_time)
                    
                else:
                    front_left_vel = int(-self.drivers_setting.front_left_vel * 100) & 0xFFFF
                    front_right_vel = int(self.drivers_setting.front_right_vel * 100) & 0xFFFF
                    rear_left_vel = int(-self.drivers_setting.rear_left_vel * 100) & 0xFFFF
                    rear_right_vel = int(self.drivers_setting.rear_right_vel * 100) & 0xFFFF
                    front_left_angle = int(self.drivers_setting.front_left_angle * 1000) & 0xFFFF
                    front_right_angle = int(self.drivers_setting.front_right_angle * 1000) & 0xFFFF
                    rear_left_angle = int(self.drivers_setting.rear_left_angle * 1000) & 0xFFFF
                    rear_right_angle = int(self.drivers_setting.rear_right_angle * 1000) & 0xFFFF
                    vals = [front_left_vel, front_right_vel, rear_left_vel, rear_right_vel, 
                            front_left_angle, front_right_angle, rear_left_angle, rear_right_angle]
                    
                    if not self.on_remote_ctrl:
                        self._write_4wheel_drivers(vals)
                        time.sleep(0.05)
                    
                # self._write_cmd('read_drivers')
                # time.sleep(0.05)
                    
                if (curr_time - battery_timer).to_sec() > 5:
                    self._write_cmd('battery')
                    time.sleep(0.05)
                    battery_timer = rospy.Time.now()
                    
                # self._write_cmd('imu')
                # time.sleep(0.05)
                
                if (curr_time - one_second_timer).to_sec() > 1:
                    self._write_cmd('remote_controller')
                    time.sleep(0.05)
                    # self._write_cmd('LED')
                    # time.sleep(0.05)
                    # self._write_cmd('bumper')
                    # time.sleep(0.05)
                    
                    if self._disconnected_as_estop:
                        self._write_cmd('estop')
                        time.sleep(0.05)
                        
                    one_second_timer = rospy.Time.now()    
                    
                self._msg_motor_error.data = self.motor_error
                self._pub_motor_error.publish(self._msg_motor_error)
            
            self.end()
                
        except KeyboardInterrupt:
            self.end()


if __name__ == '__main__':
    rospy.init_node('base_4drive')
    car = FourDriveCar()
    car.run()
