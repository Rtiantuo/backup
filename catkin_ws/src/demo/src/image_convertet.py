# coding=utf-8
import numpy as np
import  rospy
import cv2
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion


class Cfg:
    """
    Parameters
    """
    xywh_topic_name = "<topic_name>"

class Cyka:
    def __init__(self,name):
        rospy.init_node(name,anonymous=False)
        self.cmpimg_sub = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage, self.img_cb, queue_size=1)

        self.xywh_pub = rospy.Publisher(Cfg.xywh_topic_name, Quaternion, queue_size=10)

    def img_cb(self,msg):
        np_arr = np.fromstring(msg.data,np.uint8)
        img_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        """输入图、得到xywh"""
        x,y,width,heigh = [0,0,0,0]
        msg_to_send = Quaternion()
        msg_to_send.x = x
        msg_to_send.y = y
        msg_to_send.z = width
        msg_to_send.w = heigh

        cv2.imshow('cv_img', img_np)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # print Cfg.xywh_topic_name
    cyka = Cyka("cyka_node_1")
    cyka.run()