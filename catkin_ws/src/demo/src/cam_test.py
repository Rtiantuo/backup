# coding=utf-8
import cv2 as cv
import rospy
import webbrowser
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

bri = cv_bridge.CvBridge()
qrcode = cv.QRCodeDetector()
def cv_cb(data):
    img_cv = bri.imgmsg_to_cv2(data)
    cv.imshow("?",img_cv)
    cv.waitKey(0)
    result, points, code=qrcode.detectAndDecode(img_cv)
    if result:
        print points
    pass
def empty(data):
    pass

# cap = cv.VideoCapture(1, cv.CAP_DSHOW)  # 默认参数0，为本机摄像头——即计算机摄像头/也可以传入非零数据，置换其它多媒体端口
# while(True):                           #值为1不断读取图像
#     ret, frame = cap.read()            #视频捕获帧
#
#     # cv.imshow('Photo_Detect', frame)   # 显示窗口 查看实时图像
#     if not frame:
#         continue
#     qrcode = cv.QRCodeDetector()       # 载入文件库
#     result, points, code = qrcode.detectAndDecode(frame)  # 对二维码进行解码，返回二维码的信息
#
#     if code is None:                                     #如果code返回值是none说明没有识别到二维码
#         print("扫码失败，请确认是否放置二维码！！！")
#     if code is not None:                                 #如果code有返回值说明识别到二维码
#         print("扫码成功！")
#         cap.release()  # 释放捕获
#         cv.destroyAllWindows()  # 摧毁全部窗体
#         break
if __name__ == '__main__':
    rospy.init_node("CykaBlyat",anonymous=False)
    rospy.Subscriber("/usb_cam/image_raw",Image,cv_cb)
    rospy.spin()
