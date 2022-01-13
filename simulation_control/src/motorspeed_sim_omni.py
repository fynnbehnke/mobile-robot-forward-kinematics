#!/usr/bin/env python
import rospy
import math
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel_1 = Wheel()
    wheel_2 = Wheel()
    wheel_3 = Wheel()

    wheel_1.radius = 0.1
    wheel_1.angular_velocity = 0

    wheel_2.radius = 0.1
    wheel_2.angular_velocity = 0

    wheel_3.radius = 0.1
    wheel_3.angular_velocity = 0

    sim_data.robot_type = 3
    sim_data.width = 0.4
    sim_data.length = 0.4
    sim_data.wheels.append(wheel_1)
    sim_data.wheels.append(wheel_2)
    sim_data.wheels.append(wheel_3)

    time = 5.0
    timePoint = 0
    timestep = 0.1

    rospy.init_node("omni_motor_sim")
    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=1)
    
    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(10.0))
        beginTime = rospy.Time.now()
        secondsToSend = rospy.Duration(time)
        endTime = beginTime + secondsToSend
        while(rospy.Time.now() < endTime):
            sim_data.wheels[0].angular_velocity = ((-0.5*timePoint**2)+(2.5*timePoint))     #wheelspeed function for wheel 1
            sim_data.wheels[1].angular_velocity = 2.5*math.cos(timePoint)                   #wheelspeed function for wheel 2
            sim_data.wheels[2].angular_velocity = 2.5*math.sin(timePoint)                   #wheelspeed function for wheel 3
            timePoint += timestep

            data_pub.publish(sim_data)
            rospy.sleep(rospy.Duration(timestep))
        
        sim_data.wheels[0].angular_velocity = 0
        sim_data.wheels[1].angular_velocity = 0
        sim_data.wheels[2].angular_velocity = 0

        data_pub.publish(sim_data)

        exit(-666)