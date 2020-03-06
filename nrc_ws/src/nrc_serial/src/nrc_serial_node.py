#! /usr/bin/env python

import json
import rospy
import serial


from nrc_msgs.msg import DriveStatus, motors

# Global robot objects
robot_serial = None
status_pub = None


def get_drivetrain_status(timer_event):
    global robot_serial
    global status_pub

    # Read the status packet
    status = robot_serial.read_until()

    # Load status into JSON object
    try:
        status_json = json.loads(status)

        # Package the status into a message
        status_msg = DriveStatus()

        status_msg.device_id = status_json['id']
        status_msg.right_speed = status_json['right_vel']
        status_msg.left_speed = status_json['left_vel']
        status_msg.acceleration = status_json['acceleration']
        status_msg.yaw = status_json['yaw']

        # Publish the status
        status_pub.publish(status_msg)

    # Sometimes a ValueError occurs when you try to loads(). In this situation just try again next time
    except ValueError:
        pass



def send_drivetrain_command(motor_cmd):
    global robot_serial

    # Form the command packet
    command = {"target_left_speed": motor_cmd.left, "target_right_speed": motor_cmd.right}

    # Convert the packet to a string
    command_json = json.dumps(command)

    # Write the command to serial
    robot_serial.write(command_json)




if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("nrc_serial_node")

    # Set up the serial communication to the robot
    robot_serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

    # Set up a publisher for publishing the status
    status_pub = rospy.Publisher("/nrc/sensor_data", DriveStatus, queue_size=1)

    # Set up a subscriber for getting commands
    command_sub = rospy.Subscriber("/nrc/motors", motors, send_drivetrain_command, queue_size=1)

    # Set up a timer to read the sensor data at 200 Hz
    sensor_timer = rospy.Timer(rospy.Duration(secs=0.005), get_drivetrain_status)

    # Pump callbacks
    rospy.spin()

    # Close the serial port when the program ends
    robot_serial.close()
