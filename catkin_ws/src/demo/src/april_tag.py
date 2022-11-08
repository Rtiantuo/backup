#!/usr/bin/env python
# coding: UTF-8
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

tag_size = 0.285
hal_size = tag_size / 2
fx = 378.66393
# fx = 371.05646
fy = 378.81835
# fy =371.85098
cx = 315.38832
# cx =314.52178
cy = 241.69173


# cy = 241.96411
class T:
    K = numpy.array([[fx, 0., cx],
                     [0., fy, cy],
                     [0., 0., 1.]], dtype=np.float64)

    obj_points = np.array([[hal_size, hal_size, 0.],
                           [hal_size, -hal_size, 0.],
                           [-hal_size, -hal_size, 0.],
                           [-hal_size, hal_size, 0.]])
    cnt_ = 0
    valid_cnt = 0
    t0 = time.time()
    bri = cv_bridge.CvBridge()
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))
    current_odom = Odometry()
    target_position_queue = Queue(10)

    def __init__(self):
        self.qr_code_init_flag = False
        self.body_solid_init_flag = False
        rospy.init_node("node_x", anonymous=False)
        self.cnt_ = 0
        pass

    def img_cb(self, msg):
        self.cnt_ += 1
        img = self.bri.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        tar_pose_msg = Odometry()
        if tags:
            if tags[0].tag_id == 0:
                self.valid_cnt += 1
                x, y = tags[0].center
                x = int(x)
                y = int(y)
                cv2.circle(gray, (x, y), 4, (255, 0, 0), 2)
                c = [(int(x[0]), int(x[1])) for x in tags[0].corners]
                retval, rvec, tvec = cv2.solvePnP(self.obj_points, tags[0].corners, self.K, None)
                self.img_pub.publish(self.bri.cv2_to_imgmsg(gray))
                R = cv2.Rodrigues(rvec)  # Rotation Matrix
                tar_pos = np.matmul(R[0].T, -tvec)
                tar_pose_msg.header.frame_id = "map"
                tar_pose_msg.header.stamp = rospy.Time.now()
                tar_pose_msg.pose.pose.position.x = tar_pos[0, 0]
                tar_pose_msg.pose.pose.position.y = tar_pos[1, 0]
                tar_pose_msg.pose.pose.position.z = tar_pos[2, 0]
                tar_pose_msg.pose.pose.orientation.w = 1.

                R = cv2.Rodrigues(rvec)  # 旋转矩阵
                # M = tf.transformations.rotation_matrix(np.pi / 2, [1, 2, 3])
                M = np.identity(4, dtype=np.float64)
                M[:3, :3] = R[0]
                Mx = tf.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
                My = tf.transformations.rotation_matrix(-np.pi / 2, [0, 1, 0])
                Mz = tf.transformations.rotation_matrix(-np.pi / 2, [0, 0, 1])
                # M_QB = np.matmul(My,M)
                # t_B = np.matmul(M_QB[:3,:3],-tvec)

                # M_ = np.matmul(np.matmul(Mx,My),M)
                M_ = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
                qx, qy, qz, qw = tf.transformations.quaternion_from_matrix(Mx)
                t_b = np.matmul(M_[:3, :3], tvec)
                # t_b = np.matmul(np.matmul(Mx[:3,:3],My[:3,:3]),tvec)
                self.target_pos_pub.publish(tar_pose_msg)
                self.tf_broader.sendTransform(t_b, [qx, qy, qz, qw], rospy.Time.now(), "qr_code", "body_solid")
                if not self.body_solid_init_flag:
                    self.body_solid_init_flag = True

                # if retval:
                #     print tvec[-1][0]
                # print ("r: {}  \n t:{} \n =====".format(rvec,tvec))
        # else:
        #     print "None"

        # cv2.imshow("Cyka",gray)
        # cv2.waitKey(1)
        # if time.time()-self.t0>1:
        #     print self.cnt_,self.valid_cnt
        #     self.cnt_ = 0
        #     self.valid_cnt = 0
        #     self.t0 = time.time()
        # pass

    def img_cb_(self, msg):
        img = self.bri.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)

        odom = self.current_odom
        if tags:
            if tags[0].tag_id == 0:
                retval, rvec, tvec = cv2.solvePnP(self.obj_points, tags[0].corners, self.K, None)
                R = cv2.Rodrigues(rvec)  # Rotation Matrix
                M_ = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
                t_body = np.matmul(M_[:3, :3], tvec)
                print "t_body: ", t_body.T

                qx, qy, qz, qw = self.current_odom.pose.pose.orientation.x, self.current_odom.pose.pose.orientation.y, self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w
                # qx, qy, qz, qw = 0,0,0.5,.866025
                tx, ty, tz = self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y, self.current_odom.pose.pose.position.z
                M_m_b = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
                # print M_m_b

                # M_m_b = np.eye(4)
                t_m_b = np.array([[tx], [ty], [tz]])
                t_map = np.matmul(M_m_b[:3, :3], t_body) + t_m_b
                [rol, pit, yaw] = tf.transformations.euler_from_quaternion([qx, qy, qz, qw], 'szyx')
                rol = rol / np.pi * 180
                pit = pit / np.pi * 180
                yaw = yaw / np.pi * 180
                # print "Odom: ",tx,ty,tz
                print "t_map: ", t_map.T
                print "odom_: ", self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y, self.current_odom.pose.pose.position.z
                # print "yaw: ",yaw,"Pit: ",pit,"Rol: ",rol
                print "--------------------------------------"
                # t_map = np.matmul(M_m_b[:3,:3].T,(t_body-np.array([[tx],[ty],[tz]])))
                qr_msg = Odometry()
                qr_msg.header.stamp = rospy.Time.now()
                qr_msg.header.frame_id = "t265_odom_frame"
                qr_msg.pose.pose.position.x = t_map[0]
                qr_msg.pose.pose.position.y = t_map[1]
                qr_msg.pose.pose.position.z = t_map[2]
                qr_msg.pose.pose.orientation.w = 1.0
                self.target_pos_pub.publish(qr_msg)

                # print t_map[0,0],t_map[1,0],t_map[2,0]
                print (rospy.Time.now() - self.current_odom.header.stamp).to_sec()
                # print tf.transformations.euler_from_matrix(M_m_b,'sxyz')

    def odom_cb(self, msg):
        self.current_odom = msg
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.tf_broader.sendTransform(pos, ori, msg.header.stamp, "body_solid", "map")
        if not self.qr_code_init_flag:
            self.qr_code_init_flag = True

    def targt_position_update_step(self, timeEvent):
        # trans, rot = [],[]
        if not self.qr_code_init_flag:
            return
        if not self.body_solid_init_flag:
            return
        try:
            # self.tf_listener.waitForTransform('/qr_code', '/map', rospy.Time(0), rospy.Duration(0), None)
            trans, rot = self.tf_listener.lookupTransform('/map', '/qr_code', rospy.Time(0))
            # print trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            print e
            return
            pass
        '''Queue Manipulation'''
        if self.target_position_queue.full():
            if np.linalg.norm(np.var(self.target_position_queue.queue, axis=0)) > 2:
                self.target_position_queue.queue.clear()  # 方差太大则丢弃
                return
            elif np.linalg.norm(np.average(self.target_position_queue.queue, axis=0) - trans) < 1:
                self.target_position_queue.get()
                self.target_position_queue.put(trans)
                print np.average(self.target_position_queue.queue, axis=0)
                msg = Odometry()
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = trans
                msg.pose.pose.orientation.w = 0
                self.target_pos_pub.publish(msg)
        else:
            self.target_position_queue.put(trans)

    def go(self):
        self.tf_listener = tf.TransformListener()
        self.body_map_tf_broader = tf.TransformBroadcaster(queue_size=10)
        self.tf_broader = tf.TransformBroadcaster(queue_size=10)
        '''Image Message'''
        rospy.Subscriber("/d400/color/image_raw", Image, self.img_cb_)
        # rospy.Subscriber("/d400/color/image_raw", Image, self.img_cb)
        '''Odometry Message'''
        # rospy.Subscriber("/mavros/local_position/odom",Odometry,self.odom_cb)
        rospy.Subscriber("/t265/odom/sample", Odometry, self.odom_cb)

        self.img_pub = rospy.Publisher("/dotted_img", Image, queue_size=10)
        self.target_pos_pub = rospy.Publisher("/target_pose", Odometry, queue_size=10)
        # self.target_position_update_loop = rospy.Timer(rospy.Duration(0,30000000),self.targt_position_update_step)
        rospy.spin()


if __name__ == '__main__':
    t = T()
    t.go()
