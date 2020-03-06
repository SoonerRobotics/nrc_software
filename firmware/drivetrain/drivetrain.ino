#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RobotLib.h>


#define DEVICE_ID 1


/////////////////////////////////
// Actuators
/////////////////////////////////

// Drivetrain motors
Motor leftMotor, rightMotor;

// Drivetrain output parameters
float turnPower;
float drivePower;



/////////////////////////////////
// Algorithms
/////////////////////////////////

// Left and right speed control
PIDController leftController;
PIDController rightController;
float targetLeftSpeed = 0.0;
float targetRightSpeed = 0.0;

// Velocity Estimators
TrackingLoop left_tracking(0.5, 5), right_tracking(0.5, 5);
float currentSpeed;



/////////////////////////////////
// Communications
/////////////////////////////////

// Serial Send/Receive
StaticJsonDocument<128> recv_pkt;
StaticJsonDocument<128> send_pkt;



/////////////////////////////////
// Sensors
/////////////////////////////////
#define RIGHT_ENC_A 2
#define RIGHT_ENC_B 12
#define RIGHT_TICK_CONST (float)((0.3 / 178.0))  // Rolled robot 30cm (0.3m) and got 178 ticks average, so this is meters per tick
#define LEFT_ENC_A 3
#define LEFT_ENC_B 11
#define LEFT_TICK_CONST (float)(-(0.3 / 178.0))// Rolled robot 30cm (0.3m) and got 178 ticks average, so this is meters per tick

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler, accel;
sensors_event_t event;

// Encoders
QuadratureEncoder leftEncoder, rightEncoder;



/////////////////////////////////
// Timing
/////////////////////////////////
#define LOOP_RATE   200
#define LOOP_PERIOD (float)(1.0f / (float)LOOP_RATE)
#define MILLIS_PER_SECOND 1000

unsigned long last_loop_time;



/////////////////////////////////
// Angle Helper Functions
/////////////////////////////////

float constrainAngle(float x)
{
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float angleDiff(float a,float b)
{
    float dif = fmod(b - a + 180,360);
    if (dif < 0)
        dif += 360;
    return dif - 180;
}



/////////////////////////////////
// Interrupt Handlers
/////////////////////////////////

// Left Encoder
void left_encoder_isr()
{
    leftEncoder.process();
}

// Right Encoder
void right_encoder_isr()
{
    rightEncoder.process();
}



/////////////////////////////////
// Arduino Functions
/////////////////////////////////

void setup()
{
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);

    // Enable the external crystal on the IMU
    bno.setExtCrystalUse(true);

    // Set up the left and right PID controllers
    leftController.begin(0, 0.01, 0, 0);
    rightController.begin(0, 0.01, 0, 0);

    // Set up the motors
    leftMotor.begin(4,5,6);
    rightMotor.begin(8,7,9);

    // Set up the encoders
    leftEncoder.begin(LEFT_ENC_A, LEFT_ENC_B, LEFT_TICK_CONST);
    rightEncoder.begin(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_TICK_CONST);

    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), &left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), &right_encoder_isr, CHANGE);

    // Initialize loop timer
    last_loop_time = millis();

    // Fully initialize tracking loops
    left_tracking.reset();
    right_tracking.reset();
}



void loop()
{
    // BNO055 sensor data
    bno.getEvent(&event);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


    // Tracking loops for wheel velocities
    left_tracking.update(leftEncoder.getValue());
    right_tracking.update(rightEncoder.getValue());


    // Calculate current speed
    currentSpeed = 0.5 * (left_tracking.getVelocityEstimate() + right_tracking.getVelocityEstimate());


    // Sensor data update
    send_pkt["id"] = DEVICE_ID;
    send_pkt["yaw"] = euler.x();
    send_pkt["left_vel"] = left_tracking.getVelocityEstimate();
    send_pkt["right_vel"] = right_tracking.getVelocityEstimate();
    send_pkt["left_error"] = left_tracking.getPositionEstimate() - leftEncoder.getValue();
    send_pkt["right_error"] = right_tracking.getPositionEstimate() - rightEncoder.getValue();
    send_pkt["acceleration"] = accel.x();
    send_pkt["speed"] = currentSpeed;

    // Send data over serial
    serializeJson(send_pkt, Serial);
    Serial.println();


    ////////////////////////////////////////////////////////////////////////////////////////////


    // Get commands from serial
    if(Serial.available())
    {
        deserializeJson(recv_pkt, Serial);

        targetLeftSpeed = recv_pkt["target_left_speed"];
        targetRightSpeed = recv_pkt["target_right_speed"];
    }

    // Achieve the current target left and right motor speeds
    leftSpeed = leftSpeed + leftSpeedController.update(targetLeftSpeed, left_tracking.getVelocityEstimate());
    rightSpeed = rightSpeed + rightSpeedController.update(targetRightSpeed, right_tracking.getVelocityEstimate());

    // Output to the motors
    leftMotor.output(leftSpeed);
    rightMotor.output(rightSpeed);

    ////////////////////////////////////////////////////////////////////////////////////////////


    // Wait for loop update time to elapse
    while((millis() - last_loop_time) * MILLIS_PER_SECOND < LOOP_PERIOD){}

    // Update timing tracker
    last_loop_time = millis();
}