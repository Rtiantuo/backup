#!/usr/bin/env python2.7
# coding: UTF-8
import rospy
import numpy as np
import math
from geometry_msgs.msg import Quaternion, PointStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget, State
from simple_pid import PID


class Cfg:
    states = {"TRACKING": "t", "HOVERING": "h", "IDLE": "i", "CONVERTING": "c"}

    """Target Source"""
    use_qrcode = True
    target_source_topic = "/target_xywh"
    odom_source_topic = "/camera/odom/sample"
    # odom_source_topic = "/t265/odom/sample"

    """Camera and Image"""
    camera_pitch = np.pi * 33 / 180
    camera_K = np.array([[378.66393, 0., 315.38832, ],
                         [0., 378.81835, 241.69173, ],
                         [0., 0., 1.]])
    camera_K_inv = np.linalg.inv(camera_K)
    figure_width_x = 640
    figure_heigh_y = 480

    """Vehicle Dynamics"""
    max_xy_vel = 0.8
    max_z_vel = 0.4
    max_yawrate = 1.0


# TODO：何时进行current值的初始化以避免循环错误？
class Ctrl_(object):
    hover_altitude = None
    current_odom = None  # Odometry
    current_target_xywh = None  # Camera-RDF; Unit: pix
    current_target_position = None  # Camera-RDF; Unit: pix;Stamped for validating. 带时间戳以供失效检查
    current_ctrl_msg = None
    current_state = ''
    current_fcu_state = None
    init_time = None

    pid_feedback_x = PID(3., 0., 0.)
    pid_feedback_y = PID(0., 0., 0.)
    pid_feedback_z = PID(1., 0., 0.)
    pid_feedback_yaw = PID(1., 0., 0.)

    def __init__(self, use_qrcode):
        rospy.init_node('track_ctrl_node', anonymous=False)
        self.fcu_state_sub = rospy.Subscriber("/mavros/state", State, self.fcu_state_cb, queue_size=1)
        if use_qrcode:
            self.target_sub = rospy.Subscriber(Cfg.target_source_topic, Quaternion, self.qrcode_cb)
        else:
            self.target_sub = rospy.Subscriber(Cfg.target_source_topic, Quaternion, self.target_cb)

        self.odom_sub = rospy.Subscriber(Cfg.odom_source_topic, Odometry, self.odom_cb)
        self.ctrl_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=0)
        rospy.Timer(rospy.Duration(0, 20000000), self.ctrl_step)
        pass

    def go(self):
        self.current_state = Cfg.states["IDLE"]
        self.init_time = rospy.Time.now()
        rospy.spin()
        pass

    def target_cb(self, msg):
        """
        由检测跟踪算法发布，频率不定，可能掉帧
        """
        self.current_target_xywh = [msg.x, msg.y, msg.z, msg.w]
        if not self.current_target_position:
            self.current_target_position = PointStamped()

        # TODO: x,y,w,h求解距离及偏差(Camera RDF系)
        self.current_target_position.header.stamp = rospy.Time.now()
        c_x = msg.x + msg.z / 2
        c_y = msg.y + msg.w / 2
        self.current_target_position.point.x = -1 + 2 * c_x / Cfg.figure_width_x
        self.current_target_position.point.y = -1 + 2 * c_y / Cfg.figure_heigh_y
        pass

    def qrcode_cb(self, msg):
        """
        目标来源为二维码识别
        """

    def odom_cb(self, msg):
        """
        随着T265(200hz)或GPS更新，频率稳定
        """
        self.current_odom = msg
        if self.current_state == Cfg.states["IDLE"]:  # IDLE 模式下高度浮动，切入OFFBOARD模式后不再更新
            self.hover_altitude = msg.pose.pose.position.z

    def fcu_state_cb(self, msg):
        self.current_fcu_state = msg
        print self.current_fcu_state.mode

    def ctrl_step(self, timeEvent):
        """
        以Timer形式定频发布，每一帧都必须发布，否则掉帧超过一定时长fcu自动退出offboard模式
        - IDLE模式：vx,vy = 0, z = hover_altitude 此时fcu无控制响应，但如果不发则fcu无法切入OFFBOARD模式；
        为了确保切入之后飞机不发生突发运动，速度期望应当为0，悬停高度为当前高度
        -------------
        - TRACKING模式：vx,vy = pid(dx,dy) z = hover_altitude
        - HOVERING模式：vx,vy = 0，z = hover_altitude
        - 过渡模式？
        """
        if (rospy.Time.now() - self.init_time).to_sec() < 0.5:
            return
        if self.current_odom is None:
            print "Odom Not Ready"
            return
        if self.current_fcu_state is None:
            print "Fcu State Not Ready"
            return
        ctrl_msg = PositionTarget()
        ctrl_msg.header.stamp = rospy.Time.now()
        ctrl_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        ctrl_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW

        thre_ = lambda x, d: max(min(abs(d), x), -abs(d))
        if self.current_state == Cfg.states["IDLE"]:
            """非OFFBOARD的fcu模式时，自动更新悬停高度"""
            if self.current_fcu_state.mode == State.MODE_PX4_OFFBOARD:
                self.current_state = Cfg.states["HOVERING"]
            print "FCU_MODE: {}, FSM_STATE: {}".format(self.current_fcu_state.mode, self.current_state)
            self.hover_altitude = self.current_odom.pose.pose.position.z
            ctrl_msg.velocity.x = 0.
            ctrl_msg.velocity.y = 0.
            ctrl_msg.velocity.z = 0.
            ctrl_msg.yaw_rate = 0.

        if self.current_state == Cfg.states["HOVERING"]:
            """悬停在原地"""
            # State Check
            if self.current_fcu_state.mode != State.MODE_PX4_OFFBOARD:
                self.current_state = Cfg.states["IDLE"]
                print "FCU_MODE: {}, FSM_STATE: {}".format(self.current_fcu_state.mode, self.current_state)
            if self.current_target_position is None:
                print "No Target"
            elif (rospy.Time.now() - self.current_target_position.header.stamp).to_sec() < 0.2:
                self.current_state = Cfg.states["TRACKING"]
            # Default Ctrl
            vz, = self.feed_back(dz=(self.current_odom.pose.pose.position.z - self.hover_altitude))
            ctrl_msg.velocity.x = 0.
            ctrl_msg.velocity.y = 0.
            ctrl_msg.velocity.z = thre_(vz, Cfg.max_z_vel)
            ctrl_msg.yaw_rate = 0.
            pass

        if self.current_state == Cfg.states["TRACKING"]:
            """
            求解过程在target_cb环中完成
            Vehicle Body Frame: FLU
            """
            if self.current_fcu_state.mode != State.MODE_PX4_OFFBOARD:
                self.current_state = Cfg.states["IDLE"]
            if (rospy.Time.now() - self.current_target_position.header.stamp).to_sec() > 0.2:
                self.current_state = Cfg.states["HOVERING"]  # 目标更新超时-视为丢失目标,进入HOVERING模式

            """Camera RDF --> Vehicle Body FLU"""
            # print "CX:{},CY:{}".format(self.current_target_position.point.x,self.current_target_position.point.y)
            vx, vy, vz, vyaw = self.feed_back(dx=self.current_target_position.point.y,
                                              dy=self.current_target_position.point.x,
                                              dz=self.current_odom.pose.pose.position.z - self.hover_altitude,
                                              dyaw=self.current_target_position.point.x)
            vx, vy, vz = thre_(vx, Cfg.max_xy_vel), thre_(vy, Cfg.max_xy_vel), thre_(vz, 0.4)
            vyaw = thre_(vyaw, Cfg.max_yawrate)
            ctrl_msg.velocity.x = vx
            ctrl_msg.velocity.y = vy
            ctrl_msg.velocity.z = vz
            ctrl_msg.yaw_rate = vyaw
            pass

        self.ctrl_pub.publish(ctrl_msg)
        print "[{}] Delay after last tartget: {} Vx: {:.2f} Vy: {:.2f} Vz: {:.2f} Vyaw:{:.2f}".format(
            self.current_state, (
                    rospy.Time.now() - self.current_target_position.header.stamp).to_sec() if self.current_target_position else None,
            ctrl_msg.velocity.x,
            ctrl_msg.velocity.y, ctrl_msg.velocity.z, ctrl_msg.yaw_rate)

    def feed_back(self, dx=None, dy=None, dz=None, dyaw=None):
        return [y(x) for x, y
                in zip([dx, dy, dz, dyaw],
                       [self.pid_feedback_x, self.pid_feedback_y, self.pid_feedback_z, self.pid_feedback_yaw])
                if x is not None]

    @staticmethod
    def cal_position(pix_x, pix_y, height):
        """相机系:RDF"""
        pix_x = pix_x - Cfg.figure_width_x / 2
        pix_y = pix_y - Cfg.figure_heigh_y / 2
        xl, yl, foo = np.matmul(Cfg.camera_K_inv, np.array([pix_x, pix_y, 1]).T)
        z0, y0 = height / math.sin(Cfg.camera_pitch), height / math.cos(Cfg.camera_pitch)
        zl = 1 / (1 / z0 - yl / y0)
        return xl * zl, yl * zl, zl


if __name__ == '__main__':
    c = Ctrl_(False)
    c.go()
    pass
