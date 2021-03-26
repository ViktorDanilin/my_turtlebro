import rospy
import math
import time
from math import atan, pi
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

global x,z, x_pose, y_pose, angular
odom_xyt = (0, 0, 0)
odom_0_xyt = None
ta = 1
x_pose = float(raw_input("x: "))
y_pose = float(raw_input("y: "))
angular = 0.0
x = 0
z = 0

def turn_around():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular
    c = 0
    if x_pose>0 and y_pose>0:
        #1chetvert
        c = 1
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = -1*((pi/2)-atan((x_pose/y_pose))/2)
    if x_pose < 0 and y_pose > 0:
        #2chetvert
        c = 2
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = ((pi/2) - atan((x_pose / y_pose))/2)
    if x_pose < 0 and y_pose < 0:
        #3chetvert
        c = 3
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = ((pi/2) + atan((x_pose / y_pose))/2)
    if x_pose > 0 and y_pose < 0:
        #4chetvert
        c = 4
        x_pose = abs(x_pose)
        y_pose = abs(y_pose)
        angular = -1 * ((pi/2) + atan((x_pose / y_pose))/2)

    while(True):
        print(c, odom_xyt[2], angular)
        if c==1 or c==4:
            if odom_xyt[2]>angular:
                x = 0
                z = -0.1
                move(x, z)
            else:
                time.sleep(1)
                x = 0
                z = 0
                move(x,z)
                break
        if c==2 or c==3:
            if odom_xyt[2]<angular:
                x = 0
                z = 0.1
                move(x, z)
            else:
                x = 0
                z = 0
                move(x, z)
                print("vse")
                break

def move(x,z):
    pub_1_vel = Twist()
    pub_1_vel.linear.x = x
    pub_1_vel.angular.z = z
    pub_1.publish(pub_1_vel)
    rospy.sleep(0.05)

def odom_cb(mes):
    global odom_xyt, odom_0_xyt
    odom_yaw = tf.transformations.euler_from_quaternion([
mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], (odom_yaw-odom_0_xyt[2]))


sub = rospy.Subscriber('/odom', Odometry, odom_cb)


def main():
    global odom_xyt, odom_0_xyt
    turn_around()
    # turn_forward()

while not rospy.is_shutdown():

    if ta==1:
        turn_around()
        ta+=1
    # if ta==2:
    #     turn_forward()
    #     ta+=1
    rospy.sleep(0.1)