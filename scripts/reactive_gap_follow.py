#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import rospy

from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String

import numpy as np


def callback(data):
    gp=0
    for j in range(405,675): 
        if ((data.ranges[j]>2)&(data.ranges[j+1]>2)):
            gp=gp+1
            if (gp>=15):
                jj=round((j+j+gp)/2)
                ang=data.angle_min+jj*data.angle_increment
                break
    for k in range(0,404):
         if (data.ranges[k]<1):
             ang=0.785


    for l in range(676,1079):
         if (data.ranges[l]<1):
             ang=-0.785

    drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "laser"
    drive_msg.drive.steering_angle = ang #-0.785+(jj-405)*0.0058
    drive_msg.drive.speed = 0.75
    drive_pub.publish(drive_msg)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
