
import serial
import json

if __name__ == "__main__":
    # Set up the serial communication to the robot
    robot_serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

    # Run the program for as long as the port is open
    while robot_serial.is_open:
        # Read the status packet
        status = robot_serial.read_until()

        # Load status into JSON object
        try:
            status_json = json.loads(status)

            # Print the status
            print("Status: " + str(status_json['id']))
            print("Right Velocity: " + str(status_json['right_vel']))
            print("Left Velocity: " + str(status_json['left_vel']))
            print("Acceleration: " + str(status_json['acceleration']))
            print("Yaw: " + str(status_json['yaw']))
            print("\n\n")
        except ValueError:
            print("JSON loads() Error")

    # Close the serial port when the program ends
    robot_serial.close()
