import rospy
import time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

rospy.init_node('onti')
pub_1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

global dist, button,x,z
x = 0
z = 0
dist = 0
button = False

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

sub_1 = rospy.Subscriber('/ultrasound', Range, range_cb)
sub_2 = rospy.Subscriber('/pushed', Bool, start_cb)

def main():
    global dist, button,x,z
    c = 0
    if button:
        time.sleep(7)
        c+=1
        if c==1:
            if dist>50 and dist!=0:
                x = 0.5
                z = 0
                move(x,z)
                time.sleep(2)
        x = 0
        z = 0
        move(x,z)
        print(dist, button)

while not rospy.is_shutdown():
    main()
    rospy.sleep(0.1)
