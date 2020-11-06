#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 

# import math
from math import pi as pi

#PD constants
kp_d = 5                # Proportional constant distance error
kd_d = 0.01             # Derivative constant distance error

kp_a = 2               # Proportional constant angle error
kd_a = 0.01              # Derivative constant angle error

#PD references
ref_dist = 0.15                         # Reference distance
ref_angle =  (270*pi/180)/(7-1)         # Reference angle (alpha)

measured_dist = 0   # Minimum distance to the wall
measured_angle = 0

error_dist = 0          # Difference between reference and measured distance
error_angle = 0         # Angle deviation
previous_error_dist =0 
previous_error_angle = 0

R_beam = 0 
FR_beam = 0

#direction = -1           # still deciding (now 1 is right & -1 is left)

def callback_laser(data):

    global measured_dist, R_beam , FR_beam , measured_angle

    measured_dist = data.range_min ### ATENÇÃO NÃO USAR ISTO 
    R_beam = data.ranges[1]            ## no caso de mudar o numero de sensores, mudar aqui  
    FR_beam = data.ranges[2]
    if (FR_beam <= R_beam ):
        FR_beam = R_beam
    if (FR_beam >= 1 ):
        FR_beam = 1
    if (R_beam >= 1 ):
        R_beam = 1
    print("R_beam" ,R_beam , "FR_beam" ,FR_beam) 
    measured_angle = np.arccos(R_beam/FR_beam)



def follow_wall():

    global previous_error_dist , previous_error_angle , error_dist, error_angle

    msg.linear.x = 0.1
    
    error_dist = ref_dist - measured_dist
    error_angle = ref_angle - measured_angle

    delta_error_dist = error_dist - previous_error_dist
    delta_error_angle = error_angle - previous_error_angle

    previous_error_dist = error_dist
    previous_error_angle = error_angle

    #debug
    print("error_dist =", error_dist, "error_angle =", error_angle)

    #control equation
    msg.angular.z = max(min( (kp_d * error_dist + kd_d * delta_error_dist) +  (kp_a * error_angle + kd_a * delta_error_angle) , 2.0), -2.0)

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
