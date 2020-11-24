#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.1415926535897

x = -0.11
y = 0.0
theta = 0.0
g = 0
j = 0
if g == 0:
    print(5.54, 5.54)
    j = 1

rospy.init_node('robot_nti', anonymous=True)
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()


def update_pose(data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    global x
    x = round(data.x, 2)

    global y
    y = round(data.y, 2)
    global theta
    theta = data.theta
    if (theta < 0):
        theta = 2 * PI + theta


rospy.Subscriber('/turtle1/pose', Pose, update_pose)
pose = Pose()
rate = rospy.Rate(10)


def move(speed, distance, anglespeed, angle):
    global vel_msg
    global velocity_publisher
    global x
    global y
    global theta

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = anglespeed

    while not rospy.is_shutdown():

        current_distance = 0
        current_angle = 0
        flg = True
        while (flg):
            flg = (x < 0)
        x0 = x
        y0 = y
        theta0 = theta

        endangle = theta0 + angle
        while (endangle > 2 * PI):
            endangle = endangle - 2 * PI

        while ((abs(current_distance - distance) > 0.001) or (abs(current_angle - endangle) > 0.01)):

            if (abs(current_distance - distance) > 0.001):
                g = 1
            if (abs(current_angle - endangle) > 0.01):
                g = 2

            velocity_publisher.publish(vel_msg)

            current_distance = ((((x0 - x) ** 2) + ((y0 - y) ** 2)) ** 0.5)
            current_angle = theta

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        velocity_publisher.publish(vel_msg)
        if g == 1:
            print(x, y)

        break


if __name__ == 'main':
    try:

        for i in range(4):
            spd = 0.5
            dist = 3.0
            anglspd = 0
            angl = 0
            move(spd, dist, anglspd, angl)

            spd = 0.0
            dist = 0.0
            anglspd = 0.5
            angl = 0.5 * PI
            move(spd, dist, anglspd, angl)


    except rospy.ROSInterruptException:
        pass