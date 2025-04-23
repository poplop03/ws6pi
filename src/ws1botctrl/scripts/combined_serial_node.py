#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rosserial_python import SerialClient
from serial import SerialException
from time import sleep
import sys

# Initialize global variables for serial communication
serial_client = None  # Will hold the SerialClient instance

# Callback function to process /cmd_vel messages
def cmdvel_callback(msg: Twist):
    rospy.loginfo("Received /cmd_vel: Xvel = {:.2f}, Yvel = {:.2f}".format(msg.linear.x, msg.linear.y))

    # Initialize setpoints
    v1 = v2 = 0.0  # Perpendicular velocity
    setpoint1 = 0.0  # Left wheel
    setpoint2 = 0.0  # Right wheel
    L = 0.4  # Distance between left and right wheel
    R = 0.1651 / 2  # Wheel radius

    # Logic to determine setpoints based on linear and angular velocities
    if msg.linear.x != 0 or msg.angular.z != 0:
        v1 = (msg.linear.x - (msg.angular.z * L) / 2)  # Velocity for left wheel (m/s)
        v2 = (msg.linear.x + (msg.angular.z * L) / 2)  # Velocity for right wheel (m/s)
        setpoint1 = (v1 / R) / (2 * 3.1416)  # Convert to rotations per second
        setpoint2 = (v2 / R) / (2 * 3.1416)  # Convert to rotations per second
    else:
        rospy.logwarn("Unknown /cmd_vel command. Defaulting setpoints to 0.")

    # Prepare the message to publish setpoints
    setpoint_msg = Float32MultiArray()
    setpoint_msg.data = [setpoint1, setpoint2]
    setpoint_pub.publish(setpoint_msg)
    rospy.loginfo("Published setpoints: [Setpoint1: {:.2f}, Setpoint2: {:.2f}]".format(setpoint1, setpoint2))

    # Send setpoints to Arduino via serial communication
    if serial_client is not None:
        try:
            # Send the setpoints in a specific format, e.g., "S1:0.5,S2:0.6\n"
            command = "S1:{:.2f},S2:{:.2f}\n".format(setpoint1, setpoint2)
            serial_client.write(command.encode('utf-8'))  # Send the command to Arduino
            rospy.loginfo("Sent to Arduino: {}".format(command.strip()))
        except SerialException as e:
            rospy.logerr("Failed to send data to Arduino: {}".format(e))


if __name__ == "__main__":
    rospy.init_node("combined_serial_node")
    rospy.loginfo("Combined Serial and ROS Node Started")

    # Get the serial port and baudrate parameters from the launch file or default values
    port_name = rospy.get_param("~port", "/dev/ttyUSB1")
    baud = int(rospy.get_param("~baud", 57600))

    # Initialize the serial connection
    rospy.loginfo("Connecting to {} at {} baud".format(port_name, baud))
    try:
        serial_client = SerialClient(port_name, baud)
        rospy.loginfo("Serial connection established on port: {} with baudrate: {}".format(port_name, baud))
    except SerialException as e:
        rospy.logerr("Failed to open serial port: {}".format(e))
        exit(1)

    # Define the publisher for /setpoint
    setpoint_pub = rospy.Publisher("/setpoint", Float32MultiArray, queue_size=10)

    # Define the subscriber for /cmd_vel
    rospy.Subscriber("/cmd_vel", Twist, callback=cmdvel_callback)

    try:
        # Spin to keep the script running
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ROS node")

    # Close the serial connection when the node shuts down
    if serial_client is not None:
        serial_client.close()
        rospy.loginfo("Serial connection closed.")
