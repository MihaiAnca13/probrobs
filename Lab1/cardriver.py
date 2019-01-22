#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

odometry = None
laser_fwd = 0
laser_right = 0
laser_left = 0


def odometry_data(data):
    odometry = data.pose


def laser_data_fwd(data):
    global laser_fwd
    laser_fwd = data.ranges[0]


def laser_data_right(data):
    global laser_right
    laser_right = data.ranges[0]


def laser_data_left(data):
    global laser_left
    laser_left = data.ranges[0]


def driver():
    global laser_left
    global laser_fwd
    global laser_right

    rospy.init_node('driver', anonymous=True)

    rospy.Subscriber('odom', Odometry, odometry_data)
    rospy.Subscriber('base_scan_0', LaserScan, laser_data_fwd)
    rospy.Subscriber('base_scan_2', LaserScan, laser_data_right)
    rospy.Subscriber('base_scan_1', LaserScan, laser_data_left)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    while not rospy.is_shutdown():
        movement = Twist()
        print(laser_fwd)
        # if laser_fwd > 0.4:
        #     movement.linear.x = 1
        #     movement.angular.z = 0
        # else:
        #     movement.linear.x = 0
        #     movement.angular.z = -2
        # if laser_right < 0.15:
        #     movement.angular.z = 0.3
        # elif laser_left < 0.15:
        #     movement.angular.z = -0.3
        # elif movement.angular.z != -2:
        #     movement.angular.z = 0
        print(laser_left)
        print(laser_right)
        movement.linear.x = 1
        if laser_right < 2 and laser_left < 2:
            movement.angular.z = (laser_left-laser_right)
        else:
            movement.angular.z = 0
        if laser_fwd < 0.4:
            movement.linear.x = 0.3
            movement.angular.z = -2
        pub.publish(movement)
        import time
        time.sleep(0.2)

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
