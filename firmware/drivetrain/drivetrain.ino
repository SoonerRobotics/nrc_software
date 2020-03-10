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
PIDController leftSpeedController;
PIDController rightSpeedController;
float targetLeftSpeed = 0.0;
float targetRightSpeed = 0.0;
float leftCmdSpeed = 0.0;
float rightCmdSpeed = 0.0;

// Velocity Estimators
float leftSpeedEstimate = 0.0;
float rightSpeedEstimate = 0.0;
float currentSpeed;

// Low Pass IIR decay rate (smooths out velocity estimate)
#define LPIIR_DECAY 0.1f



/////////////////////////////////
// Communications
/////////////////////////////////

// Serial Send/Receive
StaticJsonDocument<128> recv_pkt;
StaticJsonDocument<128> send_pkt;



/////////////////////////////////
// Sensors
/////////////////////////////////
#define LEFT_ENC_A 2
#define LEFT_ENC_B 12
#define LEFT_TICK_CONST (float)((0.3 / 178.0))  // Rolled robot 30cm (0.3m) and got 178 ticks average, so this is meters per tick
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 11
#define RIGHT_TICK_CONST (float)(-(0.3 / 178.0))// Rolled robot 30cm (0.3m) and got 178 ticks average, so this is meters per tick

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler, accel;
sensors_event_t event;

// Encoders
QuadratureEncoder leftEncoder, rightEncoder;



/////////////////////////////////
// Timing
/////////////////////////////////
#define LOOP_RATE  100
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
    Serial.begin(115200);
    Serial.setTimeout(10);

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
    leftSpeedController.begin(0, 0.09, 0, 0.005);
    rightSpeedController.begin(0, 0.09, 0, 0.005);

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
}



void loop()
{
    // BNO055 sensor data
    bno.getEvent(&event);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


    // Get instantaneous speed for each wheel
    float instLeftSpeed = leftEncoder.getValue() / LOOP_PERIOD;
    leftEncoder.reset();
    float instRightSpeed = rightEncoder.getValue() / LOOP_PERIOD;
    rightEncoder.reset();

    // Get estimate using low pass IIR
    leftSpeedEstimate += (1.0f - LPIIR_DECAY) * (instLeftSpeed - leftSpeedEstimate);
    rightSpeedEstimate += (1.0f - LPIIR_DECAY) * (instRightSpeed - rightSpeedEstimate);

    // Calculate current speed
    currentSpeed = 0.5 * (leftSpeedEstimate + rightSpeedEstimate);


    // Sensor data update
    send_pkt["id"] = DEVICE_ID;
    send_pkt["yaw"] = euler.x();
    send_pkt["left_vel"] = leftSpeedEstimate;
    send_pkt["right_vel"] = rightSpeedEstimate;
    send_pkt["acceleration"] = accel.x();
    send_pkt["speed"] = currentSpeed;

    // Send data over serial
    serializeJson(send_pkt, Serial);
    Serial.println();


    ////////////////////////////////////////////////////////////////////////////////////////////


    // Get commands from serial
    if(Serial.available())
    {
        // deserializeJson(recv_pkt, Serial);

        // Wait for entire packet. Timeout set in setup() to only wait 10ms at worst
        String rawSerial = Serial.readStringUntil('\n');
        deserializeJson(recv_pkt, rawSerial);

        if (recv_pkt.containsKey("target_left_speed")) {
          targetLeftSpeed = recv_pkt["target_left_speed"];
        }

        if (recv_pkt.containsKey("target_right_speed")) {
          targetRightSpeed = recv_pkt["target_right_speed"];
        }
    }

    // Achieve the current target left and right motor speeds
    leftCmdSpeed -= leftSpeedController.update(targetLeftSpeed, leftSpeedEstimate);
    rightCmdSpeed -= rightSpeedController.update(targetRightSpeed, rightSpeedEstimate);

    // Clamp speed to prevent it from diverging crazily (particularly when stalled)
    leftCmdSpeed = RLUtil::clamp(leftCmdSpeed, -1, 1);
    rightCmdSpeed = RLUtil::clamp(rightCmdSpeed, -1, 1);

    // Output to the motors
    // We only output when there is enough command to actually make the wheels turn
    // Further, we rescale the output to always be >0.05 and <1.0
    leftMotor.output(abs(leftCmdSpeed) > 0.1 ? leftCmdSpeed / abs(leftCmdSpeed) * 0.05 + leftCmdSpeed * 0.95 : 0);
    rightMotor.output(abs(rightCmdSpeed) > 0.1 ? rightCmdSpeed / abs(rightCmdSpeed) * 0.05 + rightCmdSpeed * 0.95 : 0);

    ////////////////////////////////////////////////////////////////////////////////////////////


    // Wait for loop update time to elapse
    while((millis() - last_loop_time) * MILLIS_PER_SECOND < LOOP_PERIOD){}

    // Update timing tracker
    last_loop_time = millis();
}
