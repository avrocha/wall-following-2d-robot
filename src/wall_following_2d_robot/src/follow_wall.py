#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# import math
from math import pi as pi

#PD constants
kp_d = 3                # Proportional constant distance error
kd_d = 0.1              # Derivative constant distance error

kp_a = 2                # Proportional constant angle error
kd_a = 0.1              # Derivative constant angle error

#PD references
ref_dist = 0.15          # Reference distance
ref_angle = -pi/2        # Reference angle

closest_beam_dist = 0   # Minimum distance to the wall
closest_beam_angle = 0  # Minimum distance beam angle

error_dist = 0          # Difference between reference and measured distance
error_angle = 0         # Angle deviation

direction = -1           # still deciding (now 1 is right & -1 is left)

def callback_laser(data):

    global closest_beam_dist, closest_beam_angle

    min_index = 0
    for i in range(0, len(data.ranges)):
        if data.ranges[i] < data.ranges[min_index]:
            min_index = i

    closest_beam_dist = data.ranges[min_index]
    closest_beam_angle = (min_index - len(data.ranges)/2) * data.angle_increment


def follow_wall():

    global error_dist, error_angle

    msg.linear.x = 0.2
    
    delta_error_dist = (ref_dist - closest_beam_dist) - error_dist
    delta_error_angle = (direction*ref_angle - closest_beam_angle) - error_angle

    error_dist = (ref_dist - closest_beam_dist)
    error_angle = (direction*ref_angle - closest_beam_angle)

    #debug
    print("error_dist =", error_dist, "error_angle =", error_angle)

    msg.angular.z = max(min(direction*(kp_d*error_dist+kd_d*delta_error_dist) - (kp_d*error_angle+kd_d*delta_error_angle), 2.0), -2.0)

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