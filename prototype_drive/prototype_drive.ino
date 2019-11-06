/*
 * You may need to install RobotLib from
 * Tools>Manage Libraries in the Arduino IDE.
 */
#include <RobotLib.h>

/*
 * This can be set in the range 0.0 to 1.0.
 * Include a decimal point so the floating point
 * arithmetic is used.
 */
#define MAX_DRIVE .8

enum Direction {
	STOP,
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};

Motor leftMotor, rightMotor;
	
void setup()
{
	leftMotor.begin(4,5,6);
	rightMotor.begin(8,7,9);
}

void loop()
{
	/* Drive forward for 2 seconds */
	setDrive(FORWARD);
	delay(2000);
	/* Drive backward for 2 seconds */
	setDrive(BACKWARD);
	delay(2000);
	/* Drive right for 4 seconds */
	setDrive(RIGHT);
	delay(4000);
	/* Drive left for 4 seconds */
	setDrive(LEFT);
	delay(4000);
	/* Stop for 5 seconds */
	setDrive(STOP);
	delay(5000);
}

void setDrive(int direction)
{
	/* Prevent instantaneous reversal of motors */
	leftMotor.output(0);
	rightMotor.output(0);
	delay(100);

	switch (direction) {
	case STOP:
		leftMotor.output(0);
		rightMotor.output(0);
		break;
	case FORWARD:
		leftMotor.output(MAX_DRIVE);
		rightMotor.output(MAX_DRIVE);
		break;
	case LEFT:
		leftMotor.output(MAX_DRIVE/2);
		rightMotor.output(MAX_DRIVE);
		break;
	case BACKWARD:
		leftMotor.output(-MAX_DRIVE/2);
		rightMotor.output(-MAX_DRIVE/2);
		break;
	case RIGHT:
		leftMotor.output(MAX_DRIVE);
		rightMotor.output(MAX_DRIVE/2);
		break;
	}
}
