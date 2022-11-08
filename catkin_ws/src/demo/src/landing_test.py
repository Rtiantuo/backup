#!/usr/bin/env python
# coding: UTF-8
import math
import numpy
import apriltag
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
import numpy as np
import time
import tf
from Queue import Queue
from simple_pid import PID
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandTOL, SetMode

'''Parameter'''
tag_size = 0.18
hal_size = tag_size / 2
fx = 378.66393
# fx = 371.05646
fy = 378.81835
# fy =371.85098
cx = 315.38832
# cx =314.52178
cy = 241.69173
# cy = 241.96411
states = {"REACHED": 'r', "DESCENDING": 'd', "INITIAL": 'i', "HOVER": 'h', "LANDING": 'l', "OVER": 'o'}
land_distance = 0.8
max_vel = 0.6


class T:
    K = numpy.array([[fx, 0., cx],
                     [0., fy, cy],
                     [0., 0., 1.]], dtype=np.float64)

    obj_points = np.array([[-hal_size, hal_size, 0.],
                           [hal_size, hal_size, 0.],
                           [hal_size, -hal_size, 0.],
                           [-hal_size, -hal_size, 0.]])
    lost_cnt = 0
    valid_cnt = 0
    t0 = time.time()
    bri = cv_bridge.CvBridge()
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))
    current_odom = Odometry()
    current_fcu_state = State()
    # target_position_queue = Queue(10)
    pid_controller = PID(1, 0., 0., setpoint=0.)
    pid_yaw_controller = PID(2., 0., 0., setpoint=0.)
    state_now = None

    def __init__(self):
        self.target_position_queue = Queue(10)
        self.qr_code_init_flag = False
        self.body_solid_init_flag = False
        rospy.init_node("landing_ctrl_node", anonymous=False)
        self.lost_cnt = 0
        self.arrive_cnt = 0
        self.ctrl_msg = PositionTarget()
        self.pos_map = None
        self.fcu_state_valid_flg = False
        self.pos_target = None
        pass

    def ctrl_publish_step(self, timeEvent):
        # Ctrl Loop
        if self.state_now == states[
            "REACHED"] and self.current_fcu_state.mode == State.MODE_PX4_OFFBOARD and self.fcu_state_valid_flg:
            print "REACHED"
            #self.fcu_state_valid_flg = False
            #res = self.landing_client(base_mode=0, custom_mode="AUTO.LAND")
            #self.state_now = states["LANDING"]
            #print res
            return
        if not self.pos_map:
            return
        pos_odom = [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y,
                    self.current_odom.pose.pose.position.z]
        vx, vy, vz, vyaw = self.get_ctrl_value(self.pos_map, self.pos_body, pos_odom)
        ctrl_msg = PositionTarget()

        ctrl_msg.header.stamp = rospy.Time.now()
        ctrl_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        ctrl_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW

        ctrl_msg.velocity.x = vx
        ctrl_msg.velocity.y = vy
        ctrl_msg.velocity.z = vz
        ctrl_msg.yaw_rate = vyaw

        self.ctrl_pub.publish(ctrl_msg)

    def position_update(self, pos_map, pos_body, pos_odom):
        """
        pos_map: [px:double, py:double, pz:double] 惯性系
        pos_body: 体轴系
        """
        # print np.average(self.target_position_queue.queue,axis=0), pos_map
        if self.target_position_queue.full():
            if np.linalg.norm(np.var(self.target_position_queue.queue, axis=0)) > 1:
                self.target_position_queue.queue.clear()
                return
            elif np.linalg.norm(np.average(self.target_position_queue.queue, axis=0) - pos_map) < 1:
                """此次估计有效"""
                self.target_position_queue.get()
                self.target_position_queue.put(pos_map)
                # pos_map[2] = max(0.1,pos_map[2])
                self.pos_map = pos_map
                self.pos_body = pos_body
                # print np.average(self.target_position_queue.queue,axis=0)
                # dx, dy, dz, dyaw = self.get_target(np.average(self.target_position_queue.queue, axis=0),pos_body,pos_odom)
            else:
                # print "Drop It"
                pass
        else:
            self.target_position_queue.put(pos_map)

    def get_ctrl_value(self, pos_map, pos_body, pos_odom):
        pos_map = np.array(pos_map)
        pos_odom = np.array(pos_odom)
        vec = pos_map[:2] - pos_odom[:2]
        vec = vec / np.linalg.norm(vec) * land_distance

        pos_target = np.array([x for x in pos_map])
        # pos_target[:2] = pos_target[:2] - vec
        # pos_target[2] = pos_target[2] + 0.3
        pos_target[2] = pos_target[2]+0.5
        self.pos_target = pos_target
        px, py, pz = pos_target - pos_odom
        pxm, pym, pzm = pos_map - pos_odom
        # gamma = 0.5 # 下滑角
        pxy = np.sqrt(px ** 2 + py ** 2)
        gamma = pz / pxy  # 下滑角
        dz = -pz
        dx = -px
        dy = -py
        thre = lambda x, d: max(min(abs(d), x), -abs(d))
        dyaw = -math.atan2(pos_map[1] - pos_odom[1], pos_map[0] - pos_odom[0])

        vx = self.pid_controller(dx, 0.03)
        vy = self.pid_controller(dy, 0.03)
        vz = self.pid_controller(dz, 0.03)
        v_norm = np.linalg.norm([vx, vy, vz])
        vx = thre(v_norm, max_vel) * vx / v_norm
        vy = thre(v_norm, max_vel) * vy / v_norm
        vz = thre(v_norm, max_vel) * vz / v_norm
        vz = thre(vz, 0.3)
        vyaw = self.pid_yaw_controller(dyaw, 0.03)
        vyaw = thre(vyaw, 0.5)
        # print vyaw
        print "p_M: ", pos_map
        print "p_T: ", pos_target
        # print "p_P: ", px, py, pz, -dyaw
        print "p_C: ", vx, vy, vz, "yaw_rate: ",vyaw, "|v|: ",np.linalg.norm([vx,vy,vz])
        print "+++++++++++++++++++++++++"

        if np.linalg.norm(pos_odom-pos_target) < 0.2 and pos_odom[2] < 1.5:
            self.arrive_cnt += 1
        return vx, vy, vz, vyaw

    def img_cb_(self, msg):
        if self.state_now == states["LANDING"] or self.state_now == states["REACHED"]:
            pass  # return

        self.ctrl_msg.header.stamp = rospy.Time.now()
        self.ctrl_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.ctrl_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW

        img = self.bri.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        if tags:
            if tags[0].tag_id == 0:
                retval, rvec, tvec = cv2.solvePnP(self.obj_points, tags[0].corners, self.K, None)
                # retval, rvec, tvec =cv2.solvePnPRansac(self.obj_points, tags[0].corners, self.K, None)
                R = cv2.Rodrigues(rvec)  # Rotation Matrix
                M_ = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
                t_body = np.matmul(M_[:3, :3], tvec)
                R_body = R[0]  # np.matmul(M_[:3, :3],R[0])
                Orie = np.eye(4)
                Orie[:3, :3] = R_body
                qrx, qry, qrz, qrw = tf.transformations.quaternion_from_matrix(Orie)  # Target Orientation In Fb
                # print "t_body: ", t_body.T

                qx, qy, qz, qw = self.current_odom.pose.pose.orientation.x, self.current_odom.pose.pose.orientation.y, self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w
                # qx, qy, qz, qw = 0,0,0.5,.866025
                tx, ty, tz = self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y, self.current_odom.pose.pose.position.z
                M_m_b = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
                # print M_m_b
                dpit = np.pi * 33 / 180
                R_y_33 = np.array([[math.cos(dpit), 0, -np.sin(dpit)],
                                   [0, 1, 0],
                                   [np.sin(dpit), 0, np.cos(dpit)]], dtype=np.float64)
                # M_m_b = np.eye(4)
                t_1 = np.matmul(R_y_33.T,t_body)
                # print "t_1", t_1.T
                t_m_b = np.array([[tx], [ty], [tz]])
                # t_map = np.matmul(M_m_b[:3, :3], t_body) + t_m_b # Pitch = 0
                #t_map = np.matmul(R_y_33.T, np.matmul(M_m_b[:3, :3], t_body))
                t_map = np.matmul(M_m_b[:3, :3], t_1) + t_m_b
                # print "t_odom",t_m_b.T
                #print t_map.T
                #t_map = t_map + t_m_b  # Pitch =  33 deg
                qr_msg = Odometry()
                qr_msg.header.stamp = rospy.Time.now()
                qr_msg.header.frame_id = "t265_odom_frame"
                qr_msg.pose.pose.position.x = t_map[0]
                qr_msg.pose.pose.position.y = t_map[1]
                qr_msg.pose.pose.position.z = t_map[2]
                qr_msg.pose.pose.orientation.w = 1.0
                self.target_pos_pub.publish(qr_msg)
                self.position_update([x[0] for x in t_map], [x[0] for x in t_body], [tx, ty, tz])

                [yaw, pit, rol] = tf.transformations.euler_from_quaternion([qx, qy, qz, qw], 'szyx')  # Odom
                dyaw_ = math.atan2(t_body[1], t_body[0])
                # print 'yaw_odom: ',yaw/np.pi*180,'yaw_body: ', dyaw_/np.pi*180, 'yaw_target: ', (dyaw_+yaw)/np.pi*180
                [yaw_b, pit_b, rol_b] = tf.transformations.euler_from_matrix(Orie, 'szyx')  #
                dyaw = math.atan2(t_body[1, 0], t_body[0, 0])
                rol = rol / np.pi * 180
                pit = pit / np.pi * 180
                yaw = yaw / np.pi * 180
                # print yaw

                # print "Odom: ",tx,ty,tz
                #print "t_map: ", t_map.T
                #print "yaw: ",yaw,"Pit: ",pit,"Rol: ",rol
                # print "--------------------------------------"
                # t_map = np.matmul(M_m_b[:3,:3].T,(t_body-np.array([[tx],[ty],[tz]])))

                # dx = land_distance - t_body[0, 0]
                # dy = 0 - t_body[1, 0]
                # dz = 0 - t_body[2, 0]
                # dyaw = 0 - math.atan2(t_body[1], t_body[0])

                # vx, vy, vz, yawspeed = self.feedback(dx, dy, dz, dyaw)
                # self.ctrl_msg.velocity.x = vx
                # self.ctrl_msg.velocity.y = vy
                # self.ctrl_msg.velocity.z = vz
                # self.ctrl_msg.yaw_rate = 0  # yawspeed

                self.state_now = states["DESCENDING"]
                self.lost_cnt = 0
                if self.pos_target is not None:
                    '''到达检测'''
                    # print np.linalg.norm([self.pos_target[0] - tx,self.pos_target[1] - ty,self.pos_target[2] - tz]), tz
                    if np.linalg.norm([self.pos_target[0] - tx,
                                       self.pos_target[1] - ty,
                                       self.pos_target[2] - tz]) < 0.2 and tz < 1.5:
                        self.arrive_cnt += 1
            else:  #
                self.arrive_cnt = 0
                self.lost_cnt += 1
        else:
            self.arrive_cnt = 0
            self.lost_cnt += 1

        if self.lost_cnt >= 10:
            self.ctrl_msg.velocity.x = 0
            self.ctrl_msg.velocity.y = 0
            self.ctrl_msg.velocity.z = 0
            self.ctrl_msg.yaw_rate = 0.
            self.state_now = states["HOVER"]
            pass
        if self.arrive_cnt > 2:
            self.ctrl_msg.velocity.x = 0
            self.ctrl_msg.velocity.y = 0
            self.ctrl_msg.velocity.z = 0
            self.ctrl_msg.yaw_rate = 0.
            self.state_now = states["REACHED"]

    def odom_cb(self, msg):
        self.current_odom = msg
        # pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        # ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        #        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        # # self.tf_broader.sendTransform(pos, ori, msg.header.stamp, "body_solid", "map")
        # # if not self.qr_code_init_flag:
        # #     self.qr_code_init_flag = True

    def state_cb(self, msg):
        self.current_fcu_state = msg
        self.fcu_state_valid_flg = True

    def feedback(self, dx, dy, dz, dyaw):
        vx = self.pid_controller(dx, 0.03)
        vy = self.pid_controller(dy, 0.03)
        vz = self.pid_controller(dz, 0.03)
        yawspeed = self.pid_yaw_controller(dyaw)
        thre = lambda x, d: max(min(abs(d), x), -abs(d))
        vx = thre(vx, 0.5)
        vy = thre(vy, 0.5)
        vz = thre(vz, 0.3)
        yawspeed = thre(yawspeed, 0.6)

        # msg = PositionTarget()
        # msg.header.stamp = rospy.Time.now()
        # msg.coordinate_frame = msg.FRAME_BODY_NED
        # msg.type_mask = msg.IGNORE_PX | msg.IGNORE_PY | msg.IGNORE_PZ | msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ | msg.IGNORE_YAW
        # msg.velocity.x = vx
        # msg.velocity.y = vy
        # msg.velocity.z = vz
        # msg.yaw_rate = yawspeed
        # print "dx: ", dx, "dy: ", dy, "dz: ", dz, "dyaw: ", dyaw
        # print "vx: ", vx, "vy: ", vy, "vz: ", vz, "vyaw: ", yawspeed
        # self.ctrl_pub.publish(msg)
        return vx, vy, vz, yawspeed

    def go(self):
        self.tf_listener = tf.TransformListener()
        self.body_map_tf_broader = tf.TransformBroadcaster(queue_size=10)
        self.tf_broader = tf.TransformBroadcaste(queue_size=10)
        '''Image Message'''
        rospy.Subscriber("/d400/color/image_raw", Image, self.img_cb_)
        # rospy.Subscriber("/d400/color/image_raw", Image, self.img_cb)
        '''Odometry Message'''
        # rospy.Subscriber("/mavros/local_position/odom",Odometry,self.odom_cb)
        rospy.Subscriber("/t265/odom/sample", Odometry, self.odom_cb)
        self.img_pub = rospy.Publisher("/dotted_img", Image, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        self.target_pos_pub = rospy.Publisher("/target_pose", Odometry, queue_size=10)
        self.ctrl_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        rospy.Timer(rospy.Duration(0, 20000000), self.ctrl_publish_step)
        # self.target_position_update_loop = rospy.Timer(rospy.Duration(0,30000000),self.targt_position_update_step)
        self.landing_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        rospy.spin()


# class Init(smach.State):
#     nh_ = None
#
#     def __init__(self, nh):
#         smach.State.__init__(self, outcomes=[])
#         self.nh_ = nh
#
#     def execute(self, ud):
#         pass
#
#
# class Manual(smach.State):
#     pass
#
#
# class Idle(smach.State):
#     pass
#
#
# class Landing(smach.State):
#     pass
#
#
# class Reached(smach.State):
#     pass
#
# class T0:
#     a = 0
#     cykablyat = 1
#     id()
#     class T11:
#         a = 10
#         b = 11
#         def __init__(self,dd):
#             self.a = dd
#             self.b = T0.cykablyat
#
#     def __init__(self):
#         self.c = self.T11(self.a)
#         print self.a

if __name__ == '__main__':
    # rospy.init_node("CykaBlyat", anonymous=False)
    # land_srv1 = CommandTOL()
    # land_srv2 = SetMode()
    # land_srv2.custom_mode = "AUTO.LAND"
    #
    # # rospy.wait_for_service("/mavros/cmd/land",0)
    # landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
    # res = landing_client(yaw=0., min_pitch=0., latitude=0., longitude=0., altitude=0.)

    t = T()
    t.go()
