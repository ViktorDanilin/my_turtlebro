#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('onti')
ang_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)


lenn = ['inf'] * 360
flag = True
ends = False

def date_work(msg):
    global lenn
    lenn = msg.ranges

rospy.Subscriber("/scan", LaserScan, date_work)


def rotate(ang):
    ang_c = Twist()
    ang_c.angular.z = ang
    ang_vel_pub.publish(ang_c)
    print('ok2')

def prog():
    global flag
    global lenn
    print('ok3')
    if flag:
        for i in range(2, 40, 2):
            print(lenn[i], lenn[360 - i])
            if lenn[i] != float("inf") and lenn[i] != 'inf':
                if round(lenn[i], 2) == round(lenn[360 - i], 2):
                    flag = False
                    break
    if flag:
        rotate(0.1)
        print('move')
    else:
        rotate(0)
        print('stop')

def move_perp(veloc):
    vel = Twist()
    vel.linear.x = veloc
    vel.angular.z = 0
    ang_vel_pub.publish(vel)
    print('lad')


def proc():
    global lenn
    print('Yeah')
    if lenn[0] > 0.60 and lenn[0] != float("inf"):
        move_perp(0.05)
        print('yeah2')
    else:
        move_perp(0)
        print('yeah3')


while not rospy.is_shutdown():
    print('OK')
    if flag:
        prog()
    else:
        proc()
    rospy.sleep(2)