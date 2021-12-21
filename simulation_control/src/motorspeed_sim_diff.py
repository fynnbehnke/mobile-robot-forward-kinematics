#!/usr/bin/env python
import rospy
import math
import time
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel_right = Wheel()
    wheel_left = Wheel()

    wheel_right.radius = 0.1
    wheel_right.angular_velocity = 0.3
    wheel_right.steering_angle = 0

    wheel_left.radius = 0.1
    wheel_left.angular_velocity = 0.15
    wheel_left.steering_angle = 0

    sim_data.robot_type = 1
    sim_data.width = 0.25
    sim_data.length = 0
    sim_data.number_wheels = 2
    sim_data.wheels.append(wheel_right)
    sim_data.wheels.append(wheel_left)

    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=10)
    rospy.init_node("diff_motor_sim")
    r = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        #rospy.loginfo(sim_data)
        data_pub.publish(sim_data)
        r.sleep()