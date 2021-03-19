import rospy
import actionlib
import tf_conversions
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import threading

rospy.init_node("HELLO_SENDER")

odom_x = 0
odom_y = 0
odom_yaw = 0
odom_z = 0


def offset_yaw(yaw, zero_yaw):
    itog = yaw
    itog = yaw - zero_yaw
    if (itog > 1.0 * math.pi):
        itog -= 2.0 * math.pi
    if (itog < -1.0 * math.pi):
        itog += 2.0 * math.pi
    return itog


def odom_clb(data):
    global odom_x, odom_y, odom_yaw, odom_z
    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y
    odom_z = data.pose.pose.position.z
    odom_yaw = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
        data.pose.pose.orientation.w])[2]


odom_sub = rospy.Subscriber("/odom", Odometry, odom_clb)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
print("CONNECTED")


# def yaw_to_quat(yaw):
# tf_conversions.transformations.
class Goaler:
    def __init__(self, client):
        self.client = client
        # pass

    def send_goal(self, frame="base_link", x=0, y=0, yaw=0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        self.client.send_goal(goal)

        t1 = threading.Thread(target=self.wait_for_ans)
        t1.daemon = True
        t1.start()

    def wait_for_ans(self):
        wait = self.client.wait_for_result()
        if not wait:
            print("PROBLEMS :)")
        else:
            print("OKOKOKOKOK\ncmd: ")
            # print("inp: ", end="")

    # client.
    # wait = client.wait_for_result()
    # if not wait:
    # rospy.logerr("Action server not available!")
    # rospy.signal_shutdown("Action server not available!")
    # else:
    # return client.get_result()


goaler = Goaler(client)
inp = ""
while inp != "q":
    inp = input("cmd: ")
    if inp == "q":
        break
    if inp == "c":
        client.cancel_all_goals()
        continue
    inps = ["nan" for i in range(4)]
    inpss = inp.split()
    for i in range(4):
        inps[i] = inpss[i]
    frame = "base_link"
    yaw = float(inps[3])
    if inps[0] == "b":
        frame = "base_link"
    elif inps[0] == "m":
        frame = "map"
    elif inps[0] == "o":
        frame = "odom"
    if inps[3] == "n":
        yaw = str(odom_yaw)
    if len(inpss) > 4 and inpss[4] == "f":
        yaw = str(offset_yaw(odom_yaw + float(inps[3]), 0))
    goaler.send_goal(frame=frame, x=float(inps[1]), y=float(inps[2]), yaw=float(yaw))
# send_goal(frame="map")