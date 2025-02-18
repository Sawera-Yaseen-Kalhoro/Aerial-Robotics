#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

if __name__ == '__main__':
    rospy.init_node('test_height_attitude')
    pos_pub = rospy.Publisher('/ardrone/pos_ref', Vector3, queue_size=1)
    pos_msg = Vector3(0,0,0)
    euler_pub = rospy.Publisher('/ardrone/euler_ref', Vector3, queue_size=1)
    euler_msg = Vector3(0,0,0)
    yaw_pub = rospy.Publisher('/ardrone/yaw_ref', Float32, queue_size=1)
    yaw_msg = Float32(0)

    rospy.sleep(1)

    t_tot = 190.0
    dt = 0
    perc_fact = 1
    pos_msg.z = 1
    pos_pub.publish(pos_msg)
    euler_pub.publish(euler_msg)
    yaw_pub.publish(yaw_msg)
    rospy.sleep(10)
    dt += 10
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 2
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 0
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.y = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.y = 2
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.y = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.y = 0
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    yaw_msg.data = 47.5/180.0 * 3.14
    yaw_pub.publish(yaw_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    pos_msg.x = 1
    pos_msg.y = 0
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 0
    pos_msg.y = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.x = 0
    pos_msg.y = 0
    pos_pub.publish(pos_msg)
    rospy.sleep(15)
    dt += 15

    print('progress = ', 100, '%')
