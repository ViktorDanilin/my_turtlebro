#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Turtle(object):
    def __init__(self):
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.last_pose = Pose()
        self.turtle_state = 'START'
        self.pose = Pose()
        self.angles = 0

    def move(self):
        if self.turtle_state == 'FORWARD':
            if (self.needMove()):
                self.turtleForward()
            else:
                self.turtle_state = 'TURN'
                self.last_pose = self.pose
                self.turtleStop()

        if self.turtle_state == 'TURN':
            if (self.needRotate()):
                time.sleep(1)
                self.turtleRotate()

            else:
                self.turtle_state = 'Forward'
                self.last_pose = self.pose
                self.turtleStop()
                self.angles += 1

    def turtleForward(self):
        pub_twist = Twist()
        pub_twist.angular.x = 3
        self.pub.publish(pub_twist)

    def turtleRotate(self):
        pub_twist = Twist()
        pub_twist.angular.z = 0.5
        self.pub.publish(pub_twist)

    def turtleStop(self):
        pub_twist = Twist()
        self.pub.publish(pub_twist)

    def needMove(self):
        dist = math.sqrt(math.pow(self.pose.x - self.last_pose.x, 2) + math.pow(self.pose.y - self.last_pose.y, 2))
        if dist < 2:
            return True
        else:
            return False

    def needRotate(self):
        deg = self.pose.theta - self.last_pose.theta
        if (deg <= 0) and (deg < (math.pi / 2)):
            return True
        else:
            return False


def pose_callback(pose, turtle):
    if turtle.turtle_state == 'START':
        turtle.turtle_state = 'FORWARD'
        turtle.last_pose = pose
    turtle.pose = pose
    turtle.move()
    if turtle.angles == 4:
        rospy.signal_shutdown('STOP')


if __name__ == 'main':
    try:
        turtle = Turtle()
        rospy.init_node('robot', anonymous=True)
        rospy.loginfo("StartNode")
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback, turtle)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
