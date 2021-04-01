import rospy
import time
import serial
import binascii
import math
from math import atan, pi
import tf
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
ser = serial.Serial('/dev/ttyUSB0', 19200)
print('init')

global dist, button,x,z, x_pose, y_pose, angular, odom, a,m,dat
out = 0
m = []
dat = []
odom = Odometry()
odom_xyt = (0, 0, 0)
odom_0_xyt = None
# x_pose = float(raw_input())
# y_pose = float(raw_input())

ta = 1
angular = 0.0
c = 1
x = 0
z = 0
dist = 0
button = False

#zbs rabotaet
def turn_around():
    global odom_xyt, odom_0_xyt, x_pose,y_pose, angular
    c = 0
    if x_pose == 0:
        if y_pose>=0:
            c = 1
            angular = 0.0
        if y_pose<0:
            c = 3
            angular = pi
    elif y_pose == 0:
        if x_pose>0:
            c = 1
            angular = (pi/2)
        if x_pose<0:
            c = 2
            angular = -1*(pi/2)
        if x_pose==0:
            c = 1
            angular = 0.0
    else:
        if x_pose>0 and y_pose>0:
            #1chetvert
            c = 1
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = ((pi/2)-atan((y_pose/x_pose)))
        if x_pose < 0 and y_pose > 0:
            #2chetvert
            c = 2
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = -1*((pi/2) - atan((y_pose / x_pose)))
        if x_pose < 0 and y_pose < 0:
            #3chetvert
            c = 3
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = -1 * ((pi/2) + atan((y_pose / x_pose)))
        if x_pose > 0 and y_pose < 0:
            #4chetvert
            c = 4
            x_pose = abs(x_pose)
            y_pose = abs(y_pose)
            angular = ((pi/2) + atan((y_pose / x_pose)))
    if angular>0:
        t = angular / 0.25
        print(t)
        move(0,-0.25)
        time.sleep(t+ 0.1*t)
        move(0,0)
    if angular<0:
        t = angular / -0.25
        print(t)
        move(0, 0.25)
        time.sleep(t+ 0.1*t)
        move(0, 0)

#zbs
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
        x = 0.18
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

def move(x,z):
    pub_1_vel = Twist()
    pub_1_vel.linear.x = x
    pub_1_vel.angular.z = z
    pub_1.publish(pub_1_vel)

def fix_a(a):
    if a < -math.pi:
        return a + 2*math.pi
    elif a > math.pi:
        return a - 2*math.pi
    else:
        return a

def range_cb(msg):
    global dist
    dist = msg.range

def start_cb(msg):
    global button
    button = msg.data

def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom
    odom = mes
    odom_yaw = tf.transformations.euler_from_quaternion([
mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z, mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (mes.pose.pose.position.x-odom_0_xyt[0], mes.pose.pose.position.y-odom_0_xyt[1], fix_a(odom_yaw-odom_0_xyt[2]))


sub_1 = rospy.Subscriber('/ultrasound', Range, range_cb)
sub_2 = rospy.Subscriber('/pushed', Bool, start_cb)
sub_3 = rospy.Subscriber('/odom', Odometry, odom_cb)


def str2hex(s):
    return binascii.hexlify(bytes(str.encode(s)))

def reader(out,m):
    inp = ser.read()
    v = str2hex(inp)
    m.append(int(v))
    inp = ser.read()
    v = str2hex(inp)
    m.append(int(v))
    inp = ser.read()
    v = str2hex(inp)
    m.append(int(v))
    return m

def start():
    global dist, button, x, z
    while(True):
        if button:
            time.sleep(7)
            if dist>50 or dist==0:
                x = 0.2
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
count = 0
while not rospy.is_shutdown():
    if c == 1:
        start()
        c += 1
    else:
        while not rospy.is_shutdown():
            print("wait")
            m = reader(out, m)
            if len(m) == 3:
                dat.append(m)
                m = []
            x_pose = float(dat[count][1]) / float(10)
            y_pose = float(dat[count][2]) / float(10)
            print(dat[count][1], dat[count][2])
            d = dat[count][0]
            if d == 1:
                pass
            elif d == 2:
                x_pose = x_pose * -1
            elif d == 3:
                x_pose = x_pose * -1
                y_pose = y_pose * -1
            else:
                y_pose = y_pose * -1
            print(ta)
            if ta%2==1:
                turn_around()
                ta += 1
            if ta%2==0:
                turn_forward()
                ta += 1
                rospy.sleep(0.1)
    rospy.sleep(0.1)
