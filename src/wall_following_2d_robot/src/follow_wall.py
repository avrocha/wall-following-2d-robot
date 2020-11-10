#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# import math
from math import pi as pi
from circular_queue import CircularQueue

direction = 1           # 1:right // -1:left
sample_time = 0.05

#PD constants
kp_d =  5               # Proportional constant distance error
kd_d = 0.015            # Derivative constant distance error
ki_d = 1.5              # Integrative constant distance error 

kp_a = 2                # Proportional constant angle error
kd_a = 0.06             # Derivative constant angle error

#PD references
ref_dist = 0.25          # Reference distance
ref_angle = -pi/2        # Reference angle

closest_beam_dist = 0    # Minimum distance to the wall
closest_beam_angle = 0   # Minimum distance beam angle

error_dist = 0          # Difference between reference and measured distance
error_angle = 0         # Angle deviation

beam_fr = 0             # front-right beam
beam_fl = 0             # front-left beam
beam_f = 0              # front beam

buffer_I_size = 10
buffer_I = CircularQueue(buffer_I_size)       
counter_buffer_I = 0       

count = 0 
lin_vel_wander = 0.2    # initial value of wander linear velocity 

state = 0               # initial state -> 0: wander // 1: follow_wall

def callback_laser(data):

    global closest_beam_dist, closest_beam_angle , beam_fr, beam_fl, beam_f

    min_index = 0
    for i in range(0, len(data.ranges)):
        if data.ranges[i] < data.ranges[min_index]:
            min_index = i

    closest_beam_dist = data.ranges[min_index]
    closest_beam_angle = (min_index - len(data.ranges)/2) * data.angle_increment

    states(data,min_index)

    beam_f = data.ranges[len(data.ranges)/2-1]
    beam_fr = data.ranges[len(data.ranges)/2-4]
    beam_fl = data.ranges[len(data.ranges)/2+2]


def follow_wall():

    global error_dist, error_angle , counter_buffer_I, buffer_I_size , buffer_I, closest_beam_angle
    global sample_time
    #inner corner detection

    if (beam_fr < ref_dist and beam_f < ref_dist) or (beam_fl < ref_dist and beam_f < ref_dist):
        msg.linear.x = 0.05
    else : 
        msg.linear.x = 0.25

    # security condition 

    if closest_beam_dist < 0.65*ref_dist :
        msg.linear.x = 0.04
    
    delta_error_dist = (ref_dist - closest_beam_dist) - error_dist
    delta_error_angle = (direction*ref_angle - closest_beam_angle) - error_angle

    error_dist = (ref_dist - closest_beam_dist)
    error_angle = (direction*ref_angle - closest_beam_angle)

    #filling buffer up (initialize)

    if counter_buffer_I < buffer_I_size:
        buffer_I.enqueue(error_dist)
        int_error_dist = 0
        counter_buffer_I = counter_buffer_I +1 
       
    else : 
        buffer_I.dequeue()
        buffer_I.enqueue(error_dist)
        int_error_dist = buffer_I.sum

    msg.angular.z = max(min(direction*(kp_d*error_dist + ki_d * int_error_dist + kd_d*delta_error_dist/sample_time) - \
        (kp_d*error_angle+kd_d*delta_error_angle/sample_time), 2.0), -2.0)

def wander():
    global count, lin_vel_wander
    
    count += 1

    if (count%100)==0:
        lin_vel_wander = lin_vel_wander +0.05
    
    msg.linear.x = lin_vel_wander
    msg.angular.z= 0.2

def states(data,min_index):
    global state, direction
    if closest_beam_dist <= data.range_max and state == 0: # wander -> follow_wall
        state = 1
        if min_index >= len(data.ranges)/2 : # wall detected at left 
            direction = -1
        else: # wall detected at right
            direction = 1

    elif closest_beam_dist > data.range_max and state == 1 : # follow_wall -> wander 
        state = 0

def main():
    rospy.init_node('ReadLIDAR')
    print("Robot initialized!\n")

    global msg

    pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('robot0/laser0', LaserScan, callback_laser)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = Twist()
        if state == 1 : 
            follow_wall()
        elif state == 0:
            wander()
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass