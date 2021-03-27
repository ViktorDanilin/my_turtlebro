import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class Mover(object):

    def __init__(self):
        rospy.init_node('mover', log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.subscriber_odometry_cb)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)

    def subscriber_odometry_cb(self, msg):
        self.odom = msg

    def move(self):

        start_pose = self.odom

        while not rospy.is_shutdown():

            cmd_vel = Twist()
            distance = self.get_distance(start_pose.pose.pose.position, self.odom.pose.pose.position)

            if distance <= 0.1:
                rospy.loginfo("Move: %s", distance)
                cmd_vel.linear.x = 0.2

            self.pub.publish(cmd_vel)

            self.rate.sleep()

    def get_distance(self, start_pose, current_pose):
        return math.sqrt(math.pow(start_pose.x - current_pose.x, 2) + math.pow(start_pose.y - current_pose.y, 2))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")


if __name__ == '__main__':
    server = Mover()
    rospy.sleep(1)

    server.move()