#!/usr/bin/env python2.7
from __future__ import print_function
import sys
import math
import rospy

from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String, Float32

import numpy as np




def pid(dist):
    Kp = 1 # Proportional gain;
    Ki = 0   # Integral gain
    Kd = 0.0005   # Derivative gain;
    dt = 1  # Time constant;

# Variables initialization;
    cp = 0
    current_error = 0
    previous_error = 0
    sum_error = 0
    previous_error_derivative = 0
    current_error_derivative = 0
    dist_to_wall = 0.85

# Loop execution (nested while statement);
    while True:

        current_error = dist_to_wall - dist

        sum_error += sum_error + current_error*dt


        current_error_derivative = (current_error - previous_error) / dt

        previous_error = current_error

# PID Controller 
        cp += Kp * current_error + Ki * sum_error + Kd * current_error_derivative
        return(cp)



def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.angle_min+540*data.angle_increment) ### 0 degree
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.angle_min+675*data.angle_increment) ### 45 degree
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.ranges[540]) ### 0 degree
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.ranges[675]) ### 45 degree
    a=data.ranges[675] # a --> 45 deg from lef
    b=data.ranges[810] # b --> direct left
    c=data.ranges[540] # straight direction
    alpha=np.arctan((a*np.cos(.7854)-b)/a*np.sin(.7854)) # 45 deg in rad
    D_t = b*np.cos(alpha)
    print(D_t)
    dt_pub = rospy.Publisher("aaa", Float32, queue_size=10)
    dt_pub.publish(D_t)
    D_t1 = D_t + 0.75*np.sin(alpha) # L
    err = D_t1 - 0.85
    u=pid(D_t1)
    #ud=(u*180)/3.1416
    if (err>0.45):
        ut=.7854
    elif (err<0.375):
        ut=-3.14
    else:
        ut=0
    if ((ut>=0) and (ut<0.18)): # 10 deg
        sp=1.5
    elif ((ut>0.19) and (ut<0.35)): # 20 deg
        sp=1
    else:
        sp=0.5
    ###print('a:',a,'b:',b,'Dt+1:',D_t1,'error:',err,'angle:',ud)
    
    drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
    #drive = AckermannDrive(steering_angle=u)
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "laser"
    drive_msg.drive.steering_angle = ut
    drive_msg.drive.speed = sp
    drive_pub.publish(drive_msg)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
