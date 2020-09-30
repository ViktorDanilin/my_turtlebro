#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    x = float(input('Send x' + '\n'))
    y = float(input('Send y' + '\n'))
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0
    pub.publish(goal)
    rospy.sleep(1)
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('final')
        main()
    except rospy.ROSInterruptException:pass