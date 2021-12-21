#!/usr/bin/env python
import rospy
import math
import time
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel1 = Wheel()
    wheel2 = Wheel()
    wheel3 = Wheel()
    wheel4 = Wheel()

    wheel1.radius = 0.1
    wheel1.angular_velocity = -0.3
    wheel1.steering_angle = 0

    wheel2.radius = 0.1
    wheel2.angular_velocity = 0.3
    wheel2.steering_angle = 0

    wheel3.radius = 0.1
    wheel3.angular_velocity = 0.3
    wheel3.steering_angle = 0

    wheel4.radius = 0.1
    wheel4.angular_velocity = -0.3
    wheel4.steering_angle = 0

    sim_data.robot_type = 2
    sim_data.width = 0.2
    sim_data.length = 0.4
    sim_data.number_wheels = 4
    sim_data.wheels.append(wheel1)
    sim_data.wheels.append(wheel2)
    sim_data.wheels.append(wheel3)
    sim_data.wheels.append(wheel4)

    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=10)
    rospy.init_node("mecanum_motor_sim")
    r = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        #rospy.loginfo(sim_data)
        data_pub.publish(sim_data)
        r.sleep()