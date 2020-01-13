
import serial

if __name__ == "__main__":
    # Set up the serial communication to the robot
    robot_serial = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

    # Open the serial port
    robot_serial.open()

    # Run the program forever
    while robot_serial.is_open():
        # Read the status packet
        status = robot_serial.read_until()

        # Print the status
        print(status)

    robot_serial.close()
