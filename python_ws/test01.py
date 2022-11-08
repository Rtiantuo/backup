import pygame
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cmd_node',anonymous=True)
pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)

pygame.init()
pygame.joystick.init()
done = False

while(done != True ):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    joystick_count = pygame.joystick.get_count()
    for i  in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        axes = joystick.get_numaxes()
        axis1 = joystick.get_axis(1)  #左手前后  前 -1  后 +1
        axis2 = joystick.get_axis(0)   #右手左右  左-1  右+1 
        vx = -1.0*axis1
        vy = -1.0*axis2 
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        pub.publish(cmd)
        print(vx)
        print(vy)
