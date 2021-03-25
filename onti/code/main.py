import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

rospy.init_node('onti')

global dist, button
dist = 0
button = False
def range_cb(msg):
    global dist
    dist = msg.range
def start_cb(msg):
    global button
    button = msg.data

sub = rospy.Subscriber('/ultrasound', Range, range_cb)
sub = rospy.Subscriber('/pushed', Bool, start_cb)
def main():
    global dist, button
    print(dist, button)


while not rospy.is_shutdown():
    main()
    rospy.sleep(0.1)
