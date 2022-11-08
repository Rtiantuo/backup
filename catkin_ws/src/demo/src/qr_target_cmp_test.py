#!/usr/bin/env python
# coding: UTF-8
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Quaternion
import apriltag
import numpy as np
import cv2
import math
import message_filters
import cv_bridge

tag_size = 0.18
hal_size = tag_size / 2
fx = 378.66393
fy = 378.81835
cx = 315.38832
# cx = 0
cy = 241.69173


# cy = 0

class Cfg:
    use_qrcode = True
    target_source_topic = "/target_xywh"
    img_source_topic = "/d400/color/image_raw/compressed"
    states = {"TRACKING": "t", "HOVERING": "h", "IDLE": "i", "CONVERTING": "c"}
    camera_pitch = np.pi * 33 / 180
    camera_K = np.array([[378.66393, 0., cx, ],
                         [0., 378.81835, cy, ],
                         [0., 0., 1.]])
    camera_K_inv = np.linalg.inv(camera_K)
    figure_width_x = 640
    figure_heigh_y = 480


class Cyka:
    K = np.array([[fx, 0., cx],
                  [0., fy, cy],
                  [0., 0., 1.]], dtype=np.float64)

    obj_points = np.array([[-hal_size, hal_size, 0.],
                           [hal_size, hal_size, 0.],
                           [hal_size, -hal_size, 0.],
                           [-hal_size, -hal_size, 0.]])

    def __init__(self):
        rospy.init_node("cyka_node_", anonymous=False)
        rospy.Subscriber(Cfg.img_source_topic, CompressedImage, self.img_cb, queue_size=10)
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag16h5'))

        self.cvb = cv_bridge.CvBridge()
        self.xywh_pub = rospy.Publisher(Cfg.target_source_topic, Quaternion, queue_size=1)
        # depth_sub = message_filters.Subscriber("/d400/depth/image_rect_raw",Image)
        # color_sub = message_filters.Subscriber("/d400/aligned_depth_to_color/image_raw",Image)
        # ts = message_filters.TimeSynchronizer([depth_sub,color_sub],10)
        # ts = message_filters.ApproximateTimeSynchronizer([msg1,msg2],10,0.01)# 0.01s --> 10ms
        # ts.registerCallback(self.mf_test)

    def img_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)

        # cv2.imshow('cv_img', img_np)
        # cv2.waitKey(1)
        if tags:
            if tags[0].tag_id == 0:
                retval, rvec, tvec = cv2.solvePnP(self.obj_points, tags[0].corners, self.K, None)
                R = cv2.Rodrigues(rvec)  # Rotation Matrix
                M_ = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
                t_body = np.matmul(M_[:3, :3], tvec)

                x, y, z = self.cal_position(tags[0].center[0], tags[0].center[1], 0.87)
                x, y, z = z, -x, -y
                print t_body.T
                print x / 2.5, y / 2.5, z / 2.5
                print "+++++++++++++++"
                xywh_msg = Quaternion()
                xywh_msg.x, xywh_msg.y = tags[0].center
                xywh_msg.z, xywh_msg.w = 0., 0.
                self.xywh_pub.publish(xywh_msg)

    @staticmethod
    def cal_position(pix_x, pix_y, height):
        """相机系:RDF"""
        xl, yl, foo = np.matmul(Cfg.camera_K_inv, np.array([pix_x, pix_y, 1]).T)
        z0, y0 = height / math.sin(Cfg.camera_pitch), height / math.cos(Cfg.camera_pitch)
        zl = 1 / (1 / z0 - yl / y0)
        return xl * zl, yl * zl, zl

    def mf_test(self, depth_msg, color_msg):
        # depth_msg = Image()
        # color_msg = Image()
        dep_img = self.cvb.imgmsg_to_cv2(depth_msg)
        clr_img = self.cvb.imgmsg_to_cv2(color_msg)
        print dep_img.shape
        print clr_img.shape
        print "+-+-+-"

    def go(self):
        rospy.spin()


if __name__ == '__main__':
    c = Cyka()
    c.go()
