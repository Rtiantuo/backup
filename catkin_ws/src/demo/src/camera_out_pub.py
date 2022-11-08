#!/usr/bin/env python
'''camera_out ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Image

def talker():
    '''camera_out Publisher'''
    pub = rospy.Publisher('cyka', Image, queue_size=10)
    rospy.init_node('camera_out', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        msg_ = QuaternionStamped()
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
