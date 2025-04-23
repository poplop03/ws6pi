#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# Callback function to process /cmd_vel messages
def cmdvel_callback(msg: Twist):
    rospy.loginfo("Received /cmd_vel: Xvel = {:.2f}, Yvel = {:.2f}".format(msg.linear.x, msg.linear.y))
    
    # Initialize setpoints
    v1 = v2 = 0.0 #perpendicular velocity
    setpoint1 = 0.0 #left
    setpoint2 = 0.0 #right
    L = 0.4 #distance between left anf right wheel
    R = 0.1651/2 #wheel radius
    
    # Logic to determine setpoints based on linear velocities
    if msg.linear.x != 0 or msg.angular.z != 0 :
        v1 = (msg.linear.x - (msg.angular.z*L)/2)   #m/s
        v2 = (msg.linear.x + (msg.angular.z*L)/2)   #m/s
        setpoint1 = (v1/R)/(2*3.1416)   
        setpoint2 = (v2/R)/(2*3.1416) 
    else:
        rospy.logwarn("Unknown /cmd_vel command. Defaulting setpoints to 0.")
    
    # Prepare the message to publish setpoints
    setpoint_msg = Float32MultiArray()
    setpoint_msg.data = [setpoint1, setpoint2]
    
    # Publish the setpoints
    setpoint_pub.publish(setpoint_msg)
    rospy.loginfo("Published setpoints: [Setpoint1: {:.2f}, Setpoint2: {:.2f}]".format(setpoint1, setpoint2))


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("subcmdtoArduino")
    rospy.loginfo("subcmd node started")
    
    # Define the publisher for /setpoint
    setpoint_pub = rospy.Publisher("/setpoint", Float32MultiArray, queue_size=10)
    
    # Define the subscriber for /cmd_vel
    sub = rospy.Subscriber("/cmd_vel", Twist, callback=cmdvel_callback)
    
    # Spin to keep the script running
    rospy.spin()
