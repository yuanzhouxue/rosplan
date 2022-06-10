

from torch import sign
import rospy
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import JointTrajectoryControllerState
import time
import signal

total_time = 0.0

def signal_handler(signal, frame):
    rospy.logerr("Total time: %f", total_time)
    exit(0)

signal.signal(signal.SIGINT, signal_handler)


def in_hand(msg, param):
    global total_time
    # rospy.loginfo("(%s): In hand: %s", rospy.get_name(), msg.joint_names)
    start = time.time()
    ret = msg.actual.positions[0] - msg.desired.positions[0] + msg.actual.positions[1] - msg.desired.positions[1] > 0.01
    end = time.time()
    total_time += end - start
    return ret

def emptyhand(msg, param):
    global total_time
    # rospy.loginfo("(%s): In hand: %s", rospy.get_name(), msg.joint_names)
    start = time.time()
    ret = msg.actual.positions[0] - msg.desired.positions[0] + msg.actual.positions[1] - msg.desired.positions[1] < 0.01
    end = time.time()
    total_time += end - start
    return ret