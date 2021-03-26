import rospy
import time
import math
from math import atan, pi
import tf
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
global dist, button,x,z, x_pose, y_pose, angular
odom_xyt = (0, 0, 0)
odom_0_xyt = None
x_pose = float(raw_input())
y_pose = float(raw_input())

angular = 0.0
c = 1
x = 0
z = 0
dist = 0
button = False


def turn_around():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular
    c = 0
    if x_pose>0 and y_pose>0:
        #1chetvert
        c = 1
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = -1*(pi-atan((x_pose/y_pose)))
    if x_pose < 0 and y_pose > 0:
        #2chetvert
        c = 2
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = (pi - atan((x_pose / y_pose)))
    if x_pose < 0 and y_pose < 0:
        #3chetvert
        c = 3
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = (pi + atan((x_pose / y_pose)))
    if x_pose > 0 and y_pose < 0:
        #4chetvert
        c = 4
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = -1 * (pi + atan((x_pose / y_pose)))
    print(c, angular)
    while(True):
        print(odom_xyt, angular)
        if angular>=0 and angular<=odom_xyt[2]:
            if odom_xyt[2]<=angular:
                x = 0
                z = 0.1
                move(x,z)
            else:
                x = 0
                z = 0
                move(x,z)
                break
        else:
            if odom_xyt[2]>angular:
                x = 0
                z = -0.1
                move(x,z)
            else:
                x = 0
                z = 0
                move(x,z)
                break

def turn_forward():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular
    l = math.sqrt(pow(x_pose,2)+pow(y_pose,2))
    print(l)
    while(True):
        print(odom_0_xyt[0],l)
        if odom_xyt[0]<=l:
            x = 0.2
            z = 0
            move(x,z)
        else:
            x = 0
            z = 0
            move(x, z)
            break

def move(x,z):
    pub_1_vel = Twist()
    pub_1_vel.linear.x = x
    pub_1_vel.angular.z = z
    pub_1.publish(pub_1_vel)

def range_cb(msg):
    global dist
    dist = msg.range

def start_cb(msg):
    global button
    button = msg.data

def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom_updated
    odom_yaw = tf.transformations.euler_from_quaternion([
mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], (odom_yaw-odom_0_xyt[2]))


sub_1 = rospy.Subscriber('/ultrasound', Range, range_cb)
sub_2 = rospy.Subscriber('/pushed', Bool, start_cb)
sub_3 = rospy.Subscriber('/odom', Odometry, odom_cb)
def start():
    global dist, button, x, z
    while(True):
        if button:
            time.sleep(7)
            if dist>50 or dist==0:
                x = 0.5
                z = 0
                move(x,z)
                time.sleep(2)
                x = 0
                z = 0
                move(x, z)
                break
def main():
    global odom_xyt, odom_0_xyt
    turn_around()
    turn_forward()
    # print(odom_xyt, " // ", odom_0_xyt)

while not rospy.is_shutdown():
    if c == 1:
        start()
        c += 1
    else:
        main()
    rospy.sleep(0.1)


# x-0.1
# y0.4
# (2, 2.896613990462929)
# 0.412310562562
# (1, -2.896613990462929)
