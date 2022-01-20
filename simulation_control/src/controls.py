#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from simulation_control.msg import InputValues

TIMESTEP = 0.05

def get_jacobian_and_wheel_speed(robot_data):
    
    if(robot_data.robot_type == 1):

        l_diff = robot_data.width/2

        diff_J_1 = np.array([[1,0,l_diff],
                             [1,0,-l_diff],
                             [0,1,0]])
        
        diff_J_2 = np.array([[robot_data.wheels[0].radius*robot_data.wheels[0].angular_velocity],
                             [robot_data.wheels[1].radius*robot_data.wheels[1].angular_velocity],
                             [0]])

        return diff_J_1, diff_J_2
    elif(robot_data.robot_type == 2):

        l_mecanum = math.sqrt((robot_data.width/2)**2 + (robot_data.length/2)**2)
        alpha_1 = math.atan(robot_data.width/robot_data.length)

        mecanum_J_1 = np.array([[math.sqrt(2)/2, -math.sqrt(2)/2, -l_mecanum*math.cos((math.pi/4)-alpha_1)],
                                [math.sqrt(2)/2, math.sqrt(2)/2, -l_mecanum*math.cos((3*math.pi/4)+alpha_1)],
                                [math.sqrt(2)/2, math.sqrt(2)/2, -l_mecanum*math.cos((-math.pi/4)+alpha_1)],
                                [math.sqrt(2)/2, -math.sqrt(2)/2, -l_mecanum*math.cos((-3*math.pi/4)-alpha_1)]])

        mecanum_J_2 = np.array([[robot_data.wheels[0].radius*robot_data.wheels[0].angular_velocity*math.cos(-math.pi/4)],
                                [robot_data.wheels[1].radius*robot_data.wheels[1].angular_velocity*math.cos(math.pi/4)],
                                [robot_data.wheels[2].radius*robot_data.wheels[2].angular_velocity*math.cos(math.pi/4)],
                                [robot_data.wheels[3].radius*robot_data.wheels[3].angular_velocity*math.cos(-math.pi/4)]])
        
        return mecanum_J_1, mecanum_J_2
    elif(robot_data.robot_type == 3):
        
        l_omni = robot_data.width/2

        omni_J_1 = np.array([[math.sin(math.pi/3),-0.5,-l_omni],
                             [0,1,-l_omni],
                             [-math.sin(math.pi/3),-0.5,-l_omni]])

        omni_J_2 = np.array([[robot_data.wheels[0].radius*robot_data.wheels[0].angular_velocity],
                             [robot_data.wheels[1].radius*robot_data.wheels[1].angular_velocity],
                             [robot_data.wheels[2].radius*robot_data.wheels[2].angular_velocity]])

        return omni_J_1, omni_J_2
    else:
        rospy.loginfo('%d is not a valid Robot Type!\nRobot Types:\n1 = Differential Drive Mobile Robot\n2 = Omnidirectional Drive Mobile Robot\n3 = Tricycle Drive Mobile Robot', robot_data.robot_type)

def calculate_odom_values(robot_type, jacobian, wheel_speed):
    values = Twist()

    values.linear.z = 0
    values.angular.x = 0
    values.angular.y = 0

    if(robot_type == 2):
        j_1p = np.dot(np.linalg.inv(np.dot(np.transpose(jacobian),jacobian)),np.transpose(jacobian))
        temp_values = np.dot(j_1p,wheel_speed)
    else:
        temp_values = np.dot(np.linalg.inv(jacobian),wheel_speed)

    values.linear.x = temp_values[0]
    values.linear.y = temp_values[1]
    values.angular.z = temp_values[2]

    return values


def callback(data):

    control_data = Twist()

    j_1, j_2 = get_jacobian_and_wheel_speed(data)
    control_data = calculate_odom_values(data.robot_type,j_1,j_2)

    odom_pub.publish(control_data)
    rospy.sleep(rospy.Duration(TIMESTEP))

if __name__ == '__main__':
    
    rospy.init_node("odometry_sim", anonymous=True)
    rospy.Subscriber("vmr_input_data",InputValues,callback, queue_size=1)
    odom_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    rospy.spin()

