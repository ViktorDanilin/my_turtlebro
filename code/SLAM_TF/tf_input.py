import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class Final():

    def __init__(self):
        self.x = -1000
        self.y = -1000
        self._pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        self._pub_2 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self._sub_3 = rospy.Subscriber('/odom', Odometry, self.callback3, queue_size=10)
    def callback3(self, odometry):
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        if (self.position_now != None) and (self.x == -1000) and (self.y == -1000):
            self.x = float(input('Send x' + '\n'))
            self.y = float(input('Send y' + '\n'))
            ass = PoseStamped()
            ass2 = PoseWithCovarianceStamped()

            ass.header.seq = 1
            ass.header.stamp = rospy.Time.now()
            ass.header.frame_id = "base_footprint"

            ass.pose.orientation.x = 0
            ass.pose.orientation.y = 0
            ass.pose.orientation.z = 0
            ass.pose.position.x = self.x
            ass.pose.position.y = self.y
            ass.pose.position.z = 0
            ass.pose.orientation.w = 1

            ass2.header.seq = 1
            ass2.header.stamp = rospy.Time.now()
            ass2.header.frame_id = "base_link"

            ass2.pose.pose.orientation.x = 0
            ass2.pose.pose.orientation.y = 0
            ass2.pose.pose.orientation.z = 0
            ass2.pose.pose.position.x = self.position_now[0]
            ass2.pose.pose.position.y = self.position_now[1]
            ass2.pose.pose.position.z = 0
            ass2.pose.pose.orientation.w = 1
            rospy.sleep(1)
            self._pub_2.publish(ass)
            self._pub.publish(ass2)

    def main(self):

        rospy.rostime.wallsleep(10)

if __name__ == '__main__':
    rospy.init_node('Final')
    node = Final()
    node.main()