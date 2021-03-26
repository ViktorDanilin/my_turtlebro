import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray, Pose, Point
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math
import time
from PID import PID

rospy.init_node("v_and_v_node")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
left_points_pub = rospy.Publisher("/left_points", PoseArray, queue_size=1)

odom_xyt = (0, 0, 0)
odom_0_xyt = None

lidar_corordinates = []

# pid = PID(kP=2, kD=0.3)
# pid = PID(kP=1.8, kD=0.1)
pid = PID(kP=1.87, kD=0.1)

SPEED_LOW = 0.07
SPEED_HIGH = 0.15


def offset_yaw(yaw, zero_yaw):
    itog = yaw
    itog = yaw - zero_yaw
    if (itog > 1.0 * math.pi):
        itog -= 2.0 * math.pi
    if (itog < -1.0 * math.pi):
        itog += 2.0 * math.pi
    return itog


def fix_a(a):
    if a < -math.pi:
        return a + 2 * math.pi
    elif a > math.pi:
        return a - 2 * math.pi
    else:
        return a


def fix_a2(a):
    if a < 0:
        return a + 2 * math.pi
    elif a > 2 * math.pi:
        return a - 2 * math.pi
    else:
        return a


def proc_ranges(rng):
    crds = np.array(
        [np.array([math.cos(math.radians(i) - math.pi) * j, math.sin(math.radians(i) - math.pi) * j]) for i, j in
         enumerate(rng) if abs(j) != float('inf')])
    return crds


def scan_cb(mes):
    global lidar_corordinates
    lidar_corordinates = proc_ranges(mes.ranges)


def get_left_points(lidar_corordinates):
    return np.array([p for p in lidar_corordinates if p[1] < 0 and -0.1 <= p[0] <= 0.1])


def get_forward_points(lidar_corordinates):
    return np.array([p for p in lidar_corordinates if p[0] < 0 and -0.05 <= p[1] <= 0.05])


def get_forward_wall_dist(points):
    if len(points) > 0:
        return abs(points.mean(axis=0)[0])
    else:
        return float('nan')


def get_follow_data(points, target_l=0.3):
    if len(points) > 0:
        mean_dist = abs(points.mean(axis=0)[1])
        coef = np.polyfit(points[:, 0], points[:, 1], 1)
        poly1d_fn = np.poly1d(coef)
        x1 = -0.1;
        y1 = poly1d_fn(x1)
        x2 = 0.1;
        y2 = poly1d_fn(x2)

        wall_angle = math.atan2(y2 - y1, x2 - x1)
        # print("wall_angle", wall_angle, "mean_dist", mean_dist)
        return (mean_dist - target_l) / 0.3, wall_angle / (math.pi / 4)
    else:
        return None


def get_error(fl_d):
    return fl_d[0] * 0.61 + fl_d[1] * 0.39


def get_speed(fl_d, f_d):
    if abs(fl_d[0]) > 1 or f_d < 0.57:
        return SPEED_LOW
    else:
        return SPEED_HIGH


def follow(lidar_corordinates, target_l=0.3):  # v, a, d
    global pid, left_points_pub
    left_points = get_left_points(lidar_corordinates)
    forward_points = get_forward_points(lidar_corordinates)
    # left_points_pub.publish(PoseArray(poses=[Pose(position=Point(x=i[0], y=i[1], z=0)) for i in list(left_points)+list(forward_points)], header=Header(frame_id="base_scan")))
    d = get_forward_wall_dist(forward_points)
    fl_d = get_follow_data(left_points, target_l)
    if (fl_d is None):
        return (0, 0, d)
    l_v = get_speed(fl_d, d)
    e = get_error(fl_d)
    a_v = 0
    if not (e == float('nan')):
        a_v = pid.calc(e)
    # print("FL_D", fl_d, "FORWARD_D", d, "ERR", e, "OUT_A", a_v, "OUT_L", l_v)
    return (l_v, a_v, d)


def odom_cb(mes):
    global odom_xyt, odom_0_xyt, odom_updated
    odom_yaw = tf.transformations.euler_from_quaternion([
        mes.pose.pose.orientation.x, mes.pose.pose.orientation.y, mes.pose.pose.orientation.z,
        mes.pose.pose.orientation.w])[2]
    if odom_0_xyt is None:
        odom_0_xyt = (mes.pose.pose.position.x, mes.pose.pose.position.y, odom_yaw)
    odom_xyt = (
    mes.pose.pose.position.x - odom_0_xyt[0], mes.pose.pose.position.y - odom_0_xyt[1], fix_a(odom_yaw - odom_0_xyt[2]))


rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/odom", Odometry, odom_cb)


def vel_right(vel):
    pub_vel = Twist()
    pub_vel.angular.z = vel
    pub.publish(pub_vel)


def move_right():
    global odom_xyt, odom_0_xyt
    # st = odom_xyt[2]
    # t = fix_a(odom_xyt[2] - (math.pi/2)+0.05)
    # print("TURN START", t, odom_xyt[2])
    st = odom_xyt[2]
    target = fix_a2((-odom_xyt[2] + math.pi) + math.pi / 2)
    vel_right(-0.18)
    vel_right(-0.18)
    while not rospy.is_shutdown() and fix_a((st - math.pi / 2) - (odom_xyt[2])) < -0.1:
        # print(offset_yaw(fix_a2(target - (-odom_xyt[2]+math.pi)), math.pi))
        # print(st, odom_xyt[2],  (st-math.pi/2)-(odom_xyt[2]), fix_a((st-math.pi)-(odom_xyt[2])))
        # print("TURN ",t,  odom_xyt[2],  abs(fix_a(t - odom_xyt[2])), abs(fix_a(st - odom_xyt[2])))
        rospy.sleep(0.01)
    vel_right(0)


def stop():
    global pub
    out = Twist()
    out.linear.x = 0
    out.angular.z = 0
    pub.publish(out)


def break_s():
    global pub
    stop()
    out = Twist()
    out.linear.x = -0.12
    out.angular.z = 0
    pub.publish(out)
    time.sleep(0.11)
    stop()


turn_c = 0
ffff = True
ffff3 = True
while not rospy.is_shutdown():
    target_l = 0.3 - 0.0006
    if turn_c == 0:
        lp = get_left_points(lidar_corordinates)
        if len(lp) > 0 and ffff:
            mean_dist = abs(lp.mean(axis=0)[1])
            target_l = (mean_dist + 0.5) / 2
            ffff = False
        else:
            target_l = 0.5
    v, a, d = follow(lidar_corordinates, target_l)
    if turn_c == 1 and d < (0.4 + 0.07 + 0.007) and ffff3:
        print("DDDDD wait")
        break_s()
        raw_input()
        ffff3 = False
        # break
    elif turn_c == 5 and d < (0.4 - 0.13 + 0.008):
        print("DDDDD")
        break_s()
        break
    elif turn_c != 5 and d < 0.30 + 0.008:
        break_s()
        if turn_c < 5:
            print("wait")
            time.sleep(1)
            print("wait_for_enter")
            raw_input()
            # odom_0_xyt = odom_xyt
            move_right()
            stop()
            pid.zero()
            turn_c += 1
        else:
            break
        continue
        # break

    out = Twist()
    out.linear.x = v
    out.angular.z = a
    pub.publish(out)

    # left_points_pub.publish(PoseArray(poses=[Pose(position=Point(x=i[0], y=i[1], z=0)) for i in list(left_points)+list(forward_points)], header=Header(frame_id="base_scan")))
    rospy.sleep(0.001)
stop()
# pub.publish(out)