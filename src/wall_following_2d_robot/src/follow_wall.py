#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback_laser(data):
    print('first beam value :', data.ranges[0])


def follow_wall():
    msg.linear.x = 1
    msg.angular.z = 1

def main():
    rospy.init_node('ReadLIDAR')

    global msg

    pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('robot0/laser0', LaserScan, callback_laser)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():      
        msg = Twist()
        follow_wall()
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass