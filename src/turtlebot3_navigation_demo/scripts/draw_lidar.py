#!/usr/bin/env python3
from sensor_msgs.msg import ChannelFloat32
import rospy
import matplotlib
# matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import numpy as np
from queue import Queue
import tf2_py

q = Queue(10)

# ax = plt.subplot(111, projection='polar')
# ax.set_theta_zero_location("N")

laserScan = LaserScan()

def scan_callback(laserscan: LaserScan):
    global laserScan
    # plt.ion()
    # rospy.loginfo(laserscan.scan_time)
    laserScan = laserscan
    # rospy.loginfo(len(q))
    

if __name__ == "__main__":
    # global laserScan
    rospy.init_node("draw_laser_scan", anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rate = rospy.Rate(200)
    plt.ion()
    while not rospy.is_shutdown():
        rospy.loginfo(laserScan.scan_time)
        x = np.linspace(laserScan.angle_min, laserScan.angle_max, len(laserScan.ranges)).tolist()
        ax = plt.subplot(111, projection="polar")
        plt.ylim(laserScan.range_min, laserScan.range_max)
        ax.plot(x, laserScan.ranges)
        plt.pause(0.001)
        plt.clf()
        rate.sleep()
    rospy.spin()

