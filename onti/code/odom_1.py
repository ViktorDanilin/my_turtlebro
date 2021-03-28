import rospy
import math
import time
from math import atan, pi
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from std_srvs.srv import Empty

rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
pub_2 = rospy.Publisher('/odom', Odometry, queue_size=10)

global x,z, x_pose, y_pose, angular, odom, a
odom = Odometry()
odom_xyt = (0, 0, 0)
odom_0_xyt = None
ta = 1
l = 0
angular = 0.0
x = 0
z = 0
# fix around!!!!
def turn_around():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular
    c = 0
    if x_pose == 0:
        if y_pose>=0:
            c = 1
            angular = 0.0
        if y_pose<0:
            c = 3
            angular = pi-0.01
    elif y_pose == 0:
        if x_pose>0:
            c = 1
            angular = -1*(pi/2)
        if x_pose<0:
            c = 2
            angular = (pi/2)
        if x_pose==0:
            c = 1
            angular = 0.0
    else:
        if x_pose>0 and y_pose>0:
            #1chetvert
            c = 1
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = -1*((pi/2)-atan((y_pose/x_pose)))
        if x_pose < 0 and y_pose > 0:
            #2chetvert
            c = 2
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = ((pi/2) - atan((y_pose / x_pose)))
        if x_pose < 0 and y_pose < 0:
            #3chetvert
            c = 3
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = ((pi/2) + atan((y_pose / x_pose)))
        if x_pose > 0 and y_pose < 0:
            #4chetvert
            c = 4
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = -1 * ((pi/2) + atan((y_pose / x_pose)))
    angular+=odom_xyt[2]
    ######!!!!!
    if odom_xyt[2]>angular:
        if c==3:
            c+=1
        if c==2:
            c-=1
    print(odom_xyt[2],angular)


    while not rospy.is_shutdown():
        if odom_xyt[2]>(pi+angular):
            while not rospy.is_shutdown():
                move(0,-0.2)
            move(0,0)

        #print(odom_xyt[2], angular)
        if c==1 or c==4:
            if odom_xyt[2]>=angular:
                x = 0
                z = -0.25
                move(x, z)
            else:
                time.sleep(1)
                x = 0
                z = 0
                move(x,z)
                rospy.sleep(0.1)
                print("vse")
                break
        if c==2 or c==3:
            if odom_xyt[2]<=angular:
                x = 0
                z = 0.25
                move(x, z)
            else:
                x = 0
                z = 0
                move(x, z)
                rospy.sleep(0.1)
                print("vse")
                break
#zfebis
def turn_forward():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular, odom
    start_pose = odom

    r = math.sqrt(pow(odom_xyt[0],2)+pow(odom_xyt[0],2))
    l = abs(math.sqrt(pow(x_pose,2)+pow(y_pose,2)))
    v = 0
    if odom_xyt[0]<0:
        v = 1
    #print(odom_xyt[0], l)
    distance = get_distance(start_pose.pose.pose.position, odom.pose.pose.position)
    while not rospy.is_shutdown() and(distance<=(l)):
        distance = get_distance(start_pose.pose.pose.position, odom.pose.pose.position)
        print(distance,l)
        x = 0.2
        z = 0
        move(x, z)
        rospy.sleep(0.05)
    x = 0
    z = 0
    move(x, z)
    rospy.sleep(0.1)
    print("vse2")


def get_distance(start_pose, current_pose):
    return math.sqrt(math.pow(start_pose.x - current_pose.x, 2) + math.pow(start_pose.y - current_pose.y, 2))

def reset_angle():
    pub_2_r = Odometry()
    pub_2_r.pose.pose.orientation.x = 0
    pub_2_r.pose.pose.orientation.y = 0
    pub_2_r.pose.pose.orientation.z = 0
    pub_2_r.pose.pose.orientation.w = 0
    pub_2.publish(pub_2_r)
def move(x,z):
    pub_1_vel = Twist()
    pub_1_vel.linear.x = x
    pub_1_vel.angular.z = z
    pub_1.publish(pub_1_vel)
    rospy.sleep(0.05)

def fix_a(a):
    if a < -math.pi:
        return a + 2*math.pi
    elif a > math.pi:
        return a - 2*math.pi
    else:
        return a

def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom
    odom = mes
    odom_yaw = tf.transformations.euler_from_quaternion([
mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], fix_a(odom_yaw-odom_0_xyt[2]))


sub = rospy.Subscriber('/odom', Odometry, odom_cb)


def main():
    global odom_xyt, odom_0_xyt

    # turn_forward()
while not rospy.is_shutdown():

    while not rospy.is_shutdown():
        # rospy.Service('/reset', Empty,None)
        #time.sleep(2)
        reset_angle()
        x_pose = float(raw_input("x: "))
        y_pose = float(raw_input("y: "))
        print(ta)
        if ta % 2 == 1:
            turn_around()
            ta += 1
        if ta % 2 == 0:
            turn_forward()
            ta += 1
            rospy.sleep(0.1)

    rospy.sleep(0.1)