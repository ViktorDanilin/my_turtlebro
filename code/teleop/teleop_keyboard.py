#!/usr/bin/env python3

import rospy
from pynput.keyboard import Key, Listener, KeyCode, Key
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32


def on_press(key):
    global forvard, back, left, right, vel_msg, serv1_msg, serv2_msg, motor_msg, slider_msg, pub1, pub2, pub3, pub4, pub5
    if not rospy.is_shutdown():
        if (key == KeyCode(char='n')):
            motor_msg = 0
            pub4.publish(motor_msg)
        elif (key == KeyCode(char='b')):
            motor_msg = 1
            pub4.publish(motor_msg)
        elif (key == KeyCode(char='m')):
            motor_msg = 2
            pub4.publish(motor_msg)
        if (key == KeyCode(char='z')):
            slider_msg = 0
            pub5.publish(slider_msg)
        elif (key == KeyCode(char='x')):
            slider_msg = 1
            pub5.publish(slider_msg)
        elif (key == KeyCode(char='c')):
            slider_msg = 2
            pub5.publish(slider_msg)
        if (key == KeyCode(char='p')):
            serv1_msg = 140
            pub2.publish(serv1_msg)
        elif (key == KeyCode(char='l')):
            serv1_msg = 70
            pub2.publish(serv1_msg)
        elif (key == KeyCode(char='k')):
            serv1_msg = 10
            pub2.publish(serv1_msg)
        if (key == KeyCode(char='i')):
            serv2_msg = 170
            pub3.publish(serv2_msg)
        elif (key == KeyCode(char='j')):
            serv2_msg = 60
            pub3.publish(serv2_msg)
        elif (key == KeyCode(char='h')):
            serv2_msg = 10
            pub3.publish(serv2_msg)
        if (key == KeyCode(char='w')):
            vel_msg.linear.x = forvard
            pub1.publish(vel_msg)
        elif (key == KeyCode(char='a')):
            vel_msg.angular.z = left
            pub1.publish(vel_msg)
        elif (key == KeyCode(char='d')):
            vel_msg.angular.z = right
            pub1.publish(vel_msg)
        elif (key == KeyCode(char='s')):
            vel_msg.linear.x = back
            pub1.publish(vel_msg)
        elif (key == KeyCode(char='f')):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            pub1.publish(vel_msg)
        elif (key == KeyCode(char='y')):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            pub1.publish(vel_msg)
            exit()


def main(key):
    global forvard, back, left, right, vel_msg, motor_msg, slider_msg, serv1_msg, serv2_msg, pub1, pub2, pub3, pub4, pub5
    forvard = 0.2
    back = -0.2
    right = -0.5
    left = 0.5

    rospy.init_node("joystick", anonymous=True)
    pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()
    pub2 = rospy.Publisher("/arduino/servo1", UInt16, queue_size=10)
    serv1_msg = UInt16
    pub3 = rospy.Publisher("/arduino/servo2", UInt16, queue_size=10)
    serv2_msg = UInt16
    pub4 = rospy.Publisher("/arduino/motor1", UInt32, queue_size=10)
    motor_msg = UInt32
    pub5 = rospy.Publisher("/arduino/slider", UInt32, queue_size=10)
    slider_msg = UInt32
    with Listener(on_press=on_press) as listener:
        listener.join()


if __name__ == '__main__':
    try:
        main(Key)
    except rospy.ROSInterruptException:
        pass