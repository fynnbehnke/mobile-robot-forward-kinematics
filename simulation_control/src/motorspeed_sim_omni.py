#!/usr/bin/env python
import rospy
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel1 = Wheel()
    wheel2 = Wheel()
    wheel3 = Wheel()

    wheel1.radius = 0.1
    wheel1.angular_velocity = 1

    wheel2.radius = 0.1
    wheel2.angular_velocity = 1

    wheel3.radius = 0.1
    wheel3.angular_velocity = 1

    sim_data.robot_type = 3
    sim_data.width = 0.4
    sim_data.length = 0.4
    sim_data.wheels.append(wheel1)
    sim_data.wheels.append(wheel2)
    sim_data.wheels.append(wheel3)

    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=10)
    rospy.init_node("omni_motor_sim")
    r = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        data_pub.publish(sim_data)
        r.sleep()