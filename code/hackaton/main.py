import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('hack')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

global odom_xyt
odom_xyt = (0, 0, 0)
odom_0_xyt = None

def odom_cb(msg):
    global odom_xyt, odom_0_xyt
    odom_yaw = tf.transformations.euler_from_quaternion([
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (msg.pose.pose.position.x, msg.pose.pose.position.y, odom_yaw)
    odom_xyt = (msg.pose.pose.position.x-odom_0_xyt[0], msg.pose.pose.position.y-odom_0_xyt[1], odom_yaw-odom_0_xyt[2])

rospy.Subscriber('/odom',Odometry, odom_cb)
def main():
    global odom_xyt,odom_0_xyt
    if odom_xyt > -1*(math.pi/2):
        move_right(-0.3)
    else:
        move_right(0)
    print(odom_xyt)
def move_right(vel):
    pub_vel = Twist()
    pub_vel.angular.z = vel
    pub.publish(pub_vel)

while not rospy.is_shutdown():
    main()
    rospy.sleep(0.3)