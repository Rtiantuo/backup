# coding=utf-8
import rospy
import mavros
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import cv_bridge
import cv2
import apriltag
import numpy as np
import tf


tag_size = 0.18
hal_size = tag_size/2
class T:
    cnt_ = 0
    bri = cv_bridge.CvBridge()
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))
    coners_ = []
    K = np.array([[381.2862854003906, 0., 317.22650146484375],
                     [0., 381.2862854003906, 242.12477111816406],
                     [0., 0., 1.]], dtype=np.float64)
    obj_points = np.array([[-hal_size, hal_size, 0.],
                           [hal_size, hal_size, 0.],
                           [hal_size, -hal_size, 0.],
                           [-hal_size, -hal_size, 0.]])
    def __init__(self):
        rospy.init_node("CykaBlyat",anonymous=False)
        self.ctrl_pub = rospy.Publisher("/mavros/setpoint_raw/local",PositionTarget,queue_size=10)
        rospy.Subscriber("/d400/color/image_raw",Image,self.img_cb)
        self.odom_pub = rospy.Publisher("/gesture",Odometry,queue_size=2)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
        rospy.Timer(rospy.Duration(0,20000000),self.ctrl_loop)

    def img_cb(self,msg):
        odom_msg = Odometry()
        gray = self.bri.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(gray,cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        if tags:
            if tags[0].tag_id == 0:
                x_,y_ = tags[0].center
                self.coners_ = [(int(x[0]),int(x[1])) for x in tags[0].corners]
                retval, rvec, tvec = cv2.solvePnP(self.obj_points,tags[0].corners,self.K,None)
                R = cv2.Rodrigues(rvec) # 旋转矩阵
                M = tf.transformations.rotation_matrix(np.pi/2,[1,2,3])
                M = np.identity(4,dtype=np.float64)
                M[:3,:3] = R[0]
                qx,qy,qz,qw = tf.transformations.quaternion_from_matrix(M) # 四元数
                tar_pos = np.matmul(R[0].T, -tvec)
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "map"
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw
                odom_msg.pose.pose.position.x = tar_pos[0,0]
                odom_msg.pose.pose.position.y = tar_pos[1,0]
                odom_msg.pose.pose.position.z = tar_pos[2,0]
                r,p,y =tf.transformations.euler_from_matrix(M)
                self.odom_pub.publish(odom_msg)

    def go(self):
        rospy.spin()

    def ctrl_loop(self,timeEvent):
        msg = PositionTarget()
        # msg.type_mask = msg.IGNORE_PX | msg.IGNORE_PY | msg.IGNORE_PZ | msg.IGNORE_YAW |msg.IGNORE_YAW_RATE#| msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ | msg.FRAME_BODY_NED
        msg.type_mask = msg.IGNORE_PX | msg.IGNORE_PY | msg.IGNORE_PZ | msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ | msg.IGNORE_YAW
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = msg.FRAME_BODY_NED
        msg.velocity.x = 1.
        msg.velocity.y = 0.
        msg.velocity.z = 0.
        # msg.yaw_rate = np.pi / 10
        #
        # msg = TwistStamped()
        # msg.header.stamp = rospy.Time.now()
        # # msg.header.frame_id = ""
        # msg.twist.linear.x = 1.
        # msg.twist.linear.y = 0.
        # msg.twist.linear.z = 0.
        # msg.twist.angular.x = 0.
        # msg.twist.angular.y = 0.
        # msg.twist.angular.z = 0.
        # self.vel_pub.publish(msg)
        self.ctrl_pub.publish(msg)
        pass

if __name__ == '__main__':
    t = T()
    t.go()