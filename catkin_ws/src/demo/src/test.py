#!/usr/bin/env python
# coding: UTF-8
import rospy
from geometry_msgs.msg import Point

pt_msg = Point()
pt_pub = rospy.Publisher("/topic_1",Point,queue_size=1)
def std_step(e):
    pt_msg.x = 1
    pt_msg.y = 2
    pt_msg.z = 3
    pt_pub.publish(pt_msg)
if __name__ == '__main__':
    rospy.init_node("node_a",anonymous=False)
    std_loop = rospy.Timer(rospy.Duration(0,10000000),std_step)
    rospy.spin()