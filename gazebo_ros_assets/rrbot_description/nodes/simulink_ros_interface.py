#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64,Float64MultiArray

#Variable definitions
pub = []

#Parameters 
max_rpm = 400 
min_rpm = 0
max_pwm = 1600
min_pwm = 1000
publish_rate_motor_setpoints = 100 #hz


class MotorPwmValues():
    def __init__(self):
        self.pwm_motor0 = 0
        self.pwm_motor1 = 0
        self.pwm_motor2 = 0
        self.pwm_motor3 = 0
        return

motor_pwm_values = MotorPwmValues()

def process_simulink_motor0_pwm_msg(msg):
    motor_pwm_values.pwm_motor0 = ((max_rpm-min_rpm)/(max_pwm-min_pwm))*(msg.data-min_pwm)+min_rpm #conversion from pwm (range 1000 to 2000) to rpm (range 0 to rpm_max)
    return

def process_simulink_motor1_pwm_msg(msg):
    motor_pwm_values.pwm_motor1 = ((max_rpm-min_rpm)/(max_pwm-min_pwm))*(msg.data-min_pwm)+min_rpm #conversion from pwm (range 1000 to 2000) to rpm (range 0 to rpm_max)
    return

def process_simulink_motor2_pwm_msg(msg):
    motor_pwm_values.pwm_motor2 = ((max_rpm-min_rpm)/(max_pwm-min_pwm))*(msg.data-min_pwm)+min_rpm #conversion from pwm (range 1000 to 2000) to rpm (range 0 to rpm_max)
    return

def process_simulink_motor3_pwm_msg(msg):
    motor_pwm_values.pwm_motor3 = ((max_rpm-min_rpm)/(max_pwm-min_pwm))*(msg.data-min_pwm)+min_rpm #conversion from pwm (range 1000 to 2000) to rpm (range 0 to rpm_max)
    return




if __name__ == '__main__':
    rospy.init_node('simulink_ros_interface')
    pub.append(rospy.Publisher('/rrbot/motor0_velocity_controller/command',Float64, queue_size=3))
    pub.append(rospy.Publisher('/rrbot/motor1_velocity_controller/command',Float64, queue_size=3))
    pub.append(rospy.Publisher('/rrbot/motor2_velocity_controller/command',Float64, queue_size=3))
    pub.append(rospy.Publisher('/rrbot/motor3_velocity_controller/command',Float64, queue_size=3))
    rospy.Subscriber('/rrbot/simulink_ros_interface/pwm/motor0',Float64,process_simulink_motor0_pwm_msg)
    rospy.Subscriber('/rrbot/simulink_ros_interface/pwm/motor1',Float64,process_simulink_motor1_pwm_msg)
    rospy.Subscriber('/rrbot/simulink_ros_interface/pwm/motor2',Float64,process_simulink_motor2_pwm_msg)
    rospy.Subscriber('/rrbot/simulink_ros_interface/pwm/motor3',Float64,process_simulink_motor3_pwm_msg)
    

    rate = rospy.Rate(publish_rate_motor_setpoints)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        pub[0].publish(motor_pwm_values.pwm_motor0)
        pub[1].publish(motor_pwm_values.pwm_motor1)
        pub[2].publish(motor_pwm_values.pwm_motor2)
        pub[3].publish(motor_pwm_values.pwm_motor3)
        rate.sleep()

    



