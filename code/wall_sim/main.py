#!/usr/bin/env python3
import rospy
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('onti')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
c = 1
global l,x,y,m,angle
l = 1
x = 0
y = []
angle = []
m = []
def callback(msg):
    global l,y,m,angle
    y = msg.ranges
    n = len(y)
    for i in range (10,16):
        if (y[i]== float('inf')) or (y[n-i] == float('inf')):
            m.append(i)
            m.append(n - i)
        else:
            m.append(round(y[i],2))
            m.append(round(y[n-i],2))
    angle = m
    m = []
    if msg.ranges[0] == float('inf'): pass
    else:
        l = msg.ranges[0]

sub = rospy.Subscriber('/scan', LaserScan, callback)

def move(vel):
    pub_vel = Twist()
    pub_vel.linear.x = vel
    pub.publish(pub_vel)

def main():
    global l, x
    if l > 0.5:
        move(0.1)
        if l < 0.7:
            p = 1.4
            move(0.09 * (l-0.5) * p)
    else:
        move(0)
        time.sleep(0.2)
        print('расстояние до стенки: ',round(l,3),'m', sep='')
        exit()
def start():
    rospy.sleep(0.3)
    pub_vel = Twist()
    while True:
        if (angle[0]==angle[1])or(angle[2]==angle[3])or(angle[4]==angle[5]):
            break
        #print(angle[0],angle[1])
        # p = 6
        pub_vel.angular.z = 0.1
        # * (l-angle) * p
        pub.publish(pub_vel)
    #print(angle)
    pub_vel.angular.z = 0
    pub.publish(pub_vel)
while not rospy.is_shutdown():
    if c == 1:
        start()
        c += 1
    else:
        main()
    rospy.sleep(0.2)