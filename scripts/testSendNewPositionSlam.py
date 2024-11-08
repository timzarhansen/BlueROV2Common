#!/usr/bin/env python3
import time

from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np


def main():
    rospy.init_node('publishTestSlamResults')
    publisher_leakageStatus = rospy.Publisher('slam_results', PoseStamped, queue_size=10)
    time.sleep(3)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    angle = 0.0 / 180.0 * np.pi

    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = np.sin(angle / 2.0)
    msg.pose.orientation.w = np.cos(angle / 2.0)
    time.sleep(1)#simulating time of processing
    publisher_leakageStatus.publish(msg)




if __name__ == '__main__':
    main()
