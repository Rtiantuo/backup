#!/usr/bin/env python
'''camera_out ROS Node'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import struct
import cv_bridge

def callback_handmade(data):
    '''camera_out Callback Function'''
    pixel_num = data.width*data.height
    channel_num = len(data.data)/pixel_num
    img_list =struct.unpack(">"+"B"*len(data.data),data.data)
    img_ndarray = np.array(img_list)
    img_ndarray.resize([data.height,data.width,channel_num])
    print (img_ndarray.shape)
    print (img_ndarray[0,0,0])

def callback_auto(data):
    cb = cv_bridge.CvBridge()
    img_ndarray = cb.imgmsg_to_cv2(data)
    print ("PictureSize:",img_ndarray.shape,"Data Type:", type(img_ndarray))
    print (type(rospy.Time.now()))

def listener():
    '''camera_out Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_out', anonymous=False)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback_auto)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
