#!/usr/bin/env python
# Code adapted from: Weihang Guo [https://www.whguo.me/posts/f110_ttc/]


import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.THRESHOLD = 0.4 
        self.ranges = []
        self.angles = []
        self.angles_cos = []
        self.speed = 0
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        
        self.initialized = False

        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        # TODO: publish brake message and publish controller bool
        self.ranges =np.array(scan_msg.ranges)
        # rospy.loginfo(np.min(self.ranges))
        if self.initialized == False:
        # read and save angle_min, angle_max, angle_increment
            self.angle_min = scan_msg.angle_min
            self.angle_max = scan_msg.angle_max
            self.angle_increment = scan_msg.angle_increment
            self.angles = np.linspace(self.angle_min,self.angle_max,len(self.ranges))
            # rospy.loginfo(len(self.ranges))
            self.angles_cos = np.cos(self.angles)
            # self.angles_cos = np.where(self.angles_cos>0, self.angles_cos, 0)
            self.initialized = True
        # rospy.loginfo(self.angles_cos)
        self.calculate_TTC() 
    def calculate_TTC(self):
        if self.speed < 0.1 and self.speed > -0.1:
            return 
        subspeeds = self.angles_cos*self.speed
        # TTCs = np.divide(self.ranges, subspeeds,out=np.zeros_like(self.ranges), where=subspeeds!=0)
        TTCs = np.divide(self.ranges, subspeeds)
        # TTC = np.min(TTCs[np.nonzero(TTCs)])
        lower_than_threhold = False
        for TTC in TTCs:
            if TTC > 0 and TTC < self.THRESHOLD:
                lower_than_threhold = True
                break
        # rospy.loginfo(TTC)
        if lower_than_threhold:
            brake_bool_pub = rospy.Publisher("brake_bool", Bool, queue_size=10)
            brake_bool_msg = Bool(data=True)
            brake_bool_pub.publish(brake_bool_msg)

            brake_pub = rospy.Publisher("vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
            drive = AckermannDrive(speed=0)
            brake_msg = AckermannDriveStamped(drive=drive)
            brake_pub.publish(brake_msg)
            self.speed = 0
            # rospy.loginfo("warning")
    
def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
