import rospy
import cv2
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
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32


rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
pub_2 = rospy.Publisher("/servo_1",UInt16,queue_size=5)
pub_3 = rospy.Publisher("/servo_2",UInt16,queue_size=5)
pub_5 = rospy.Publisher("/servo_3",UInt16,queue_size=5)
pub_6 = rospy.Publisher("/servo_4",UInt16,queue_size=5)
pub_4 = rospy.Publisher("/slider",UInt32,queue_size=5)

ser = serial.Serial('/dev/ttyUSB1', 19200)
print('init')

global dist, button,x,z, x_pose, y_pose, angular, odom, a,m,dat,pose,state,camera_state
camera_state = 0
pose = 0
state = 0
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

def servo1(pose):
    pub_2_d = UInt16()
    pub_2_d.data = pose
    pub_2.publish(pub_2_d)
    rospy.sleep(0.01)

def servo2(pose):
    pub_3_d = UInt16()
    pub_3_d.data = pose
    pub_3.publish(pub_3_d)
    rospy.sleep(0.01)

def slider(state):
    pub_4_d = UInt32()
    pub_4_d.data = state
    pub_4.publish(pub_4_d)
    rospy.sleep(0.01)

def servo3(pose):
    pub_5_d = UInt16()
    pub_5_d.data = pose
    pub_5.publish(pub_5_d)
    rospy.sleep(0.01)

def servo4(pose):
    pub_6_d = UInt16()
    pub_6_d.data = pose
    pub_6.publish(pub_6_d)
    rospy.sleep(0.01)



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

def camera():
    cap = cv2.VideoCapture(2)
    print('OK')
    print(cap.isOpened())
    for i in range(20):
        ret, frame = cap.read()
    cap.release()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(gray_image)
    cv2.imwrite('test.jpg', gray_image)
    with open('test.jpg', 'rb') as file:
        img = file.read()
        pack_size = 1024
        pack_num = int(len(img) / pack_size) + 1
        for i in range(0, pack_num):
            serial_pack = img[i * pack_size:(i + 1) * pack_size]
            send_bytes = ser.write(serial_pack)
            print(send_bytes)
            time.sleep(0.5)
        time.sleep(4)
        ser.write('S')

def start():
    global dist, button, x, z
    servo1(170)
    servo2(130)
    servo3(0)
    servo4(0)
    while(True):
        if button:
            time.sleep(7)
            if dist>50 or dist==0:
                x = 0.2
                z = 0
                move(x,z)
                time.sleep(6)
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
            m = reader(out, m)
            if len(m) == 3:
                dat.append(m)
                m = []
            m = reader(out, m)
            if len(m) == 3:
                dat.append(m)
                m = []
            x_pose = float(dat[count][1]) / float(10)
            y_pose = float(dat[count][2]) / float(10)
            servo_num = dat[count+1][0]
            pose = str(dat[count+1][1])+str(dat[count+1][2])
            pose = int(pose)
            print(pose, "angle")
            state = dat[count+2][0]
            camera_state = dat[count+2][1]
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
            if state==1:
                slider(state)
            if servo_num==1:
                servo1(pose)
            if servo_num==2:
                servo2(pose)
            if servo_num==3:
                servo3(0)
                servo4(180)
            if servo_num==4:
                servo3(90)
                servo4(130)
            if camera_state==1:
                camera()
            m = []
            dat = []
    rospy.sleep(0.1)