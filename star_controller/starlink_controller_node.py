#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

#Define your class - use package name with camel case capitalization
class StarlinkPID(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('starlink_controller_node')
        self.actual_heading = 0.0
        self.desired_heading = 0.0
        #Define the publishers here
        self.wrench_pub_ = self.create_publisher(WrenchStamped, 'wrench_cmd', 10)
        #PARAMS HERE
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 0.001)
        self.Kp = self.get_parameter('Kp')
        self.Ki = self.get_parameter('Ki')
        self.Kd = self.get_parameter('Kd')
        #Define the subscribers here
        self.imu_sub_ = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10) #Imu is message type and imu/data is topic and 10 is que size
        self.setpoint_sub_ = self.create_subscription(Vector3, 'setpoint', self.setpoint_callback, 10)
        self.d_time = 10
        self.desired_heading = 0.0 #yaw
        self.desired_speed =  0.0
        
        #Variable to track the current time
        self.current_time = self.get_clock().now()

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.time = self.create_timer(time_period, self.timer_callback)

# This is the timer function that runs at the desired rate from above
    def timer_callback(self):
    #     #This is an example on how to create a message, fill in the header information and add some data, then publish
        # self.twist_msg.twist.linear.x = 10.0
        # self.twist_pub_.publish(self.twist_msg)
        error_output = self.PID()
        
        #publish cmd
        msg_out = WrenchStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.wrench.force.x = self.desired_speed
        msg_out.wrench.torque.z = error_output
        self.wrench_pub_.publish(msg_out) #publishes the message
        # #This is how to keep track of the current time in a ROS2 node
        self.current_time = self.get_clock().now()

    #Put your callback functions here - these are run whenever the node loops and when there are messages available to process.
    def imu_callback(self, msg):
        imu = msg.data
        self.actual_heading = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) #heading
        #this is an example on how to compare the current time to the time in a message. This next line subtracts the two times, converts to nanoseconds then puts it in seconds and checks if its greater than 5 seconds
        self.d_time = (self.get_clock().now().to_msg()-self.current_time).nanoseconds*(10**-9) #.
        

    def setpoint_callback(self, msg):
        self.desired_heading = msg.z #yaw
        self.desired_speed =  msg.x #velocity in x

    def PID(self):
        error = self.desired_heading - self.actual_heading #i think we want to control the aacceleration but I am not sure about attitude
        self.Kp = self.get_parameter('Kp').value
        self.Kd = self.get_parameter('Kd').value
        self.Ki = self.get_parameter('Ki').value
        p = self.Kp*error
        i = self.Ki*error*(self.d_time)
        d = self.Kd*error/(self.d_time)
        controller = p + i + d
        return controller

#This is some boiler plate code that you slap at the bottom to make the node work.
#Make sure to update the name of the function and package to match your node
def main(args=None):
    rclpy.init(args=args)

    starlink_controller = StarlinkPID()
    rclpy.spin(starlink_controller)
    starlink_controller.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()