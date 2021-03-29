import rospy
import math
import time
from math import atan, pi, degrees
import tf
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


global x,z, x_pose, angular, odom, odom_xyt, odom_0_xyt
prev_orientation = Odometry()
odom = Odometry()
odom_xyt = (0, 0, 0)
odom_0_xyt = None
ta = 1
l = 0
angular = 0.0
x = 0
z = 0

def fix_a(a):
    if a < -math.pi:
        return a + 2*math.pi
    elif a > math.pi:
        return a - 2*math.pi
    else:
        return a

def fix_a2(a):
    if a < 0:
        return a + 2*math.pi
    elif a > 2*math.pi:
        return a - 2*math.pi
    else:
        return a

def turn_angle():
    global odom_0_xyt, odom_xyt, odom
    move(0,0)
    prev_orientation = Odometry()
    angle = int(raw_input("ang: "))
    angle = math.radians(angle)
    # odom_angle = odom_xyt[2] - odom_0_xyt[2]
    # v=s/t => t=s/v
    if angle>0:
        t = angle / 0.25
        print(t)
        move(0,-0.25)
        time.sleep(t+ 0.1*t)
        move(0,0)
    if angle<0:
        t = angle / -0.25
        print(t)
        move(0, 0.25)
        time.sleep(t+ 0.1*t)
        move(0, 0)
def turn_forward():
    global odom_xyt, odom_0_xyt, odom
    move(0,0)
    start_pose = odom
    l = float(raw_input("l: "))

    distance = get_distance(start_pose.pose.pose.position, odom.pose.pose.position)
    while not rospy.is_shutdown() and(distance<=(l)):
        distance = get_distance(start_pose.pose.pose.position, odom.pose.pose.position)
        x = 0.2
        z = 0
        move(x, z)
        rospy.sleep(0.05)
    move(0, 0)
    rospy.sleep(0.1)
    print("vse2")

def get_degree_diff(prev_orientation, current_orientation):
    prev_q = [prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w]
    current_q = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

    delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))
    (_, _, yaw) = euler_from_quaternion(delta_q)

    return degrees(yaw)

def get_distance(start_pose, current_pose):
    return math.sqrt(math.pow(start_pose.x - current_pose.x, 2) + math.pow(start_pose.y - current_pose.y, 2))

def move(x,z):
    pub_1_vel = Twist()
    pub_1_vel.linear.x = x
    pub_1_vel.angular.z = z
    pub_1.publish(pub_1_vel)

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
    turn_angle()
while not rospy.is_shutdown():
    main()
    time.sleep(2)
    rospy.sleep(0.1)