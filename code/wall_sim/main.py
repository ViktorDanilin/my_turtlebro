#!/usr/bin/env python3
import rospy
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

rospy.init_node('onti')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
c = 1
global l,x,y,m,angle
l = 1
x = 0
y = []
angle = 0
m = []
def callback1(msg):
    if msg.ranges[0] == float('inf'): pass
    else:
        global l,y,m,angle
        y = msg.ranges
        for i in range (len(y)-1):
            if y[i] != float('inf'):
                m.append(y[i])
        angle = min(m)
        m = []
        l = msg.ranges[0]
def callback2(msg):
    global x
    x = msg.transforms[0]

sub = rospy.Subscriber('/scan', LaserScan, callback1)
sub2 = rospy.Subscriber('/tf',TFMessage,callback2)

def move(vel):
    pub_vel = Twist()
    pub_vel.linear.x = vel
    pub.publish(pub_vel)

def main():
    global l, x
    if l > 0.52:
        move(0.1)
    else:
        move(0)
        time.sleep(0.2)
        print('расстояние до стенки: ',round(l,3),'m', sep='')
        exit()
def start():
    rospy.sleep(0.3)
    pub_vel = Twist()
    while (l-0.1) > (angle):
        p = 2
        pub_vel.angular.z = 0.2 * (l-angle) * p
        pub.publish(pub_vel)
    pub_vel.angular.z = 0
    pub.publish(pub_vel)
while not rospy.is_shutdown():
    if c == 1:
        start()
        c += 1
    else:
        main()
    rospy.sleep(0.1)