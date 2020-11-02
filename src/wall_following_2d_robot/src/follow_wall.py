#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback_laser(msg):

def follow_wall():

def main():
    rospy.init_node('Read LIDAR')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('[robot]/laser/scan', LaserScan, callback_laser)

    while not rospy.is_shutdown():
        follow_wall()
        msg = Twist()

        

if __name__ == '__main__':
    main()