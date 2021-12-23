#!/usr/bin/env python
import rospy
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel_right = Wheel()
    wheel_left = Wheel()

    wheel_right.radius = 0.1
    wheel_right.angular_velocity = 1

    wheel_left.radius = 0.1
    wheel_left.angular_velocity = 2

    sim_data.robot_type = 1
    sim_data.width = 0.25
    sim_data.length = 0
    sim_data.wheels.append(wheel_right)
    sim_data.wheels.append(wheel_left)

    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=10)
    rospy.init_node("diff_motor_sim")
    r = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        data_pub.publish(sim_data)
        r.sleep()