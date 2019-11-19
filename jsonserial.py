import serial
import json

#the default baudrate is 9600
#Parameters must be name of the serial port
ser = serial.Serial("/dev/ttyUSB0")

commands = {}
#default properties
commands["leftMotor"] = True
commands["rightMotor"] = True

while True:
	control = input("How to move the robot?")
	if cmd == "w":
		commands["leftMotor"] = True
		commands["rightMotor"] = True
 	elif cmd == "a":
		commands["leftMotor"] = True
		commands["rightMotor"] = False
	elif cmd == "d":
		commands["leftMotor"] = False
		commands["rightMotor"] = True
	elif cmd == "s":
		commands["leftMotor"] = False
		commands["rightMotor"] = False
	else
		break
	ser.write(json.dumps(commands).encode("utf-8"))
