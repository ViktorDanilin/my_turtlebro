#!/usr/bin/env python3
import rospy
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rospy.init_node('onti')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
res = 1

global length, x, y, move, ugol
length = 1
x = 0
y = []
ugol = []
move = []

def re_msg(msg):
    global length, y, move, ugol
    y = msg.ranges
    ind = 10
    while ind < 16:
        if y[len(y) - ind] == float('inf') or y[ind] == float('inf'):
            move.append(ind)
            move.append(len(y) - ind)
        else:
            move.append(round(y[ind], 2))
            move.append(round(y[len(y) - ind], 2))
        ind += 1

    ugol = move
    move = []
    if msg.ranges[0] == float('inf'):
        pass
    else:
        length = msg.ranges[0]

sub = rospy.Subscriber('/scan', LaserScan, re_msg)

def main():
    pub_velos = Twist()
    global length, x
    if length > 0.5:

        pub_velos.linear.x = 0.1
        pub.publish(pub_velos)

        if length < 0.7:
            pub_velos.linear.x = 0.09 * (length - 0.5) * 1.4
            pub.publish(pub_velos)
    else:
        pub_velos.linear.x = 0
        pub.publish(pub_velos)
        time.sleep(0.2)
        print('Расстояние до стенки: ', round(length, 3), 'm', sep='')
        exit()

def start():
    rospy.sleep(0.3)
    pub_vel = Twist()

    while True:
        if ugol[0] == ugol[1] or ugol[2] == ugol[3] or ugol[4] == ugol[5]:
            break
        print(ugol[0], ugol[1])

        pub_vel.angular.z = 0.1
        pub.publish(pub_vel)

    print(ugol)
    pub_vel.angular.z = 0
    pub.publish(pub_vel)

while not rospy.is_shutdown():
    if res == 1:
        start()
        res += 1
    else:
        main()
    rospy.sleep(0.2)