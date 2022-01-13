#!/usr/bin/env python
import rospy
import math
from simulation_control.msg import InputValues
from simulation_control.msg import Wheel



if __name__ == '__main__':
    sim_data = InputValues()
    wheel_fl = Wheel()
    wheel_fr = Wheel()
    wheel_rl = Wheel()
    wheel_rr = Wheel()

    wheel_fl.radius = 0.1
    wheel_fl.angular_velocity = 0

    wheel_fr.radius = 0.1
    wheel_fr.angular_velocity = 0

    wheel_rl.radius = 0.1
    wheel_rl.angular_velocity = 0

    wheel_rr.radius = 0.1
    wheel_rr.angular_velocity = 0

    sim_data.robot_type = 2
    sim_data.width = 0.2
    sim_data.length = 0.4
    sim_data.wheels.append(wheel_fl)
    sim_data.wheels.append(wheel_fr)
    sim_data.wheels.append(wheel_rl)
    sim_data.wheels.append(wheel_rr)

    time = 5.0
    timePoint = 0
    timestep = 0.1

    rospy.init_node("mecanum_motor_sim")
    data_pub = rospy.Publisher("vmr_input_data",InputValues,queue_size=10)
    
    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(10.0))
        beginTime = rospy.Time.now()
        secondsToSend = rospy.Duration(time)
        endTime = beginTime + secondsToSend
        while(rospy.Time.now() < endTime):
            sim_data.wheels[0].angular_velocity = ((-0.5*timePoint**2)+(2.5*timePoint))     #wheelspeed function for front left wheel
            sim_data.wheels[1].angular_velocity = math.cos(2.5*timePoint)                   #wheelspeed function for front right wheel
            sim_data.wheels[2].angular_velocity = 1                                         #wheelspeed function for rear left wheel
            sim_data.wheels[3].angular_velocity = (1.7-(0.7*math.sin(2.5*timePoint+1.6)))   #wheelspeed function for rear right wheel
            timePoint += timestep

            data_pub.publish(sim_data)
            rospy.sleep(rospy.Duration(timestep))
        
        sim_data.wheels[0].angular_velocity = 0
        sim_data.wheels[1].angular_velocity = 0
        sim_data.wheels[2].angular_velocity = 0
        sim_data.wheels[3].angular_velocity = 0

        data_pub.publish(sim_data)

        exit(-666)