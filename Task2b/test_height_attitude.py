#!/usr/bin/env python3

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
    rospy.init_node('test_height_attitude')
    pos_pub = rospy.Publisher('/ardrone/pos_ref', Vector3, queue_size=1)
    pos_msg = Vector3(0,0,0)
    euler_pub = rospy.Publisher('/ardrone/euler_ref', Vector3, queue_size=1)
    euler_msg = Vector3(0,0,0)

    rospy.sleep(1)

    t_tot = 245
    dt = 0
    perc_fact = 1.3
    pos_msg.z = 1
    pos_pub.publish(pos_msg)
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.z = 2
    pos_pub.publish(pos_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    pos_msg.z = 1
    pos_pub.publish(pos_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.x = 0.1
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.x = 0
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.x = -0.1
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.x = 0
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.y = 0.1
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact / t_tot * 100, '%')

    euler_msg.y = 0
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.y = -0.1
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.y = 0
    euler_pub.publish(euler_msg)
    rospy.sleep(6)
    dt += 6
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.z = 1
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.z = 3
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.z = -3
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.z = -1
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', dt * perc_fact /  t_tot * 100, '%')

    euler_msg.z = 0
    euler_pub.publish(euler_msg)
    rospy.sleep(20)
    dt += 20
    print('progress = ', 100, '%')
