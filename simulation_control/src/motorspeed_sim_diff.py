#!/usr/bin/env python
import rospy
import math
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel_right = Wheel()
    wheel_left = Wheel()

    wheel_right.radius = 0.07
    wheel_right.angular_velocity = 0

    wheel_left.radius = 0.07
    wheel_left.angular_velocity = 0

    sim_data.robot_type = 1
    sim_data.width = 0.2
    sim_data.length = 0
    sim_data.wheels.append(wheel_right)
    sim_data.wheels.append(wheel_left)

    timePoint = 0

    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=1)
    rospy.init_node("diff_motor_sim")
    
    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(10.0))
        beginTime = rospy.Time.now()
        secondsToSend = rospy.Duration(5.0)
        endTime = beginTime + secondsToSend
        while(rospy.Time.now() < endTime):
            sim_data.wheels[0].angular_velocity = ((-0.5*timePoint**2)+(2.5*timePoint))
            sim_data.wheels[1].angular_velocity = (1.7-(0.7*math.sin(2.5*timePoint+1.6)))
            timePoint += 0.1

            data_pub.publish(sim_data)
            rospy.sleep(rospy.Duration(0.1))
        
        sim_data.wheels[0].angular_velocity = 0
        sim_data.wheels[1].angular_velocity = 0

        data_pub.publish(sim_data)

        exit(-666)