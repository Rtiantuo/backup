# turtlebot手柄
wzc的电脑要先执行
```
source activate
conda activate ros
```

**运行环境：Python3**

```py
import pygame
import rospy
from geometry_msgs.msg import Twist

pub=rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
# pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

pygame.init()
pygame.joystick.init()
done=False


while (done != True):
    rospy.init_node('cmd',anonymous=True)
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        axes = joystick.get_numaxes()
        axis1 = joystick.get_axis(1)# 左手前后，前-1，后+1
        axis2 = joystick.get_axis(3)# 右手左右，左-1，右+1
        front_and_back=-0.3*axis1
        left_and_right=-0.6*axis2

        cmd = Twist()
        cmd.linear.x=front_and_back
        cmd.angular.z=left_and_right
        pub.publish(cmd)
        print("===============")
        print(cmd)

        exit = joystick.get_axis(2) + joystick.get_axis(5)
        if (exit > 1.5):
            break
    if (exit > 1.5):
        break

```