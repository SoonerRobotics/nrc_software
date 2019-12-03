#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RobotLib.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

PIDController yawController;
float targetAngle = 0.0;
Motor leftMotor, rightMotor;
float offset = 0.0;

imu::Vector<3> euler;
sensors_event_t event;
float turnPower;
float drivePower = 0.5;

StaticJsonDocument<256> recv_pkt, send_pkt;

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

void setup()
{
    Serial.begin(9600);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);

    bno.setExtCrystalUse(true);

    yawController.begin(0, 0.01, 0, 0);

    leftMotor.begin(4,5,6);
    rightMotor.begin(8,7,9);
}




void loop()
{
    // Get commands from serial
    if(Serial.available())
    {
        deserializeJson(recv_pkt, Serial);

        drivePower = recv_pkt["drive_power"];   // TODO: replace with target velocity and then make a PID for drive power
        targetAngle = recv_pkt["target_yaw"];
    }


    // BNO055 sensor data
    bno.getEvent(&event);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    send_pkt["yaw"] = euler.x();

    // Tracking loop for velocity
    // TODO

    // Send data over serial
    serializeJson(send_pkt, Serial);



    turnPower = yawController.update(0, angleDiff(constrainAngle(euler.x()), targetAngle));


    leftMotor.output(drivePower - turnPower);
    rightMotor.output(drivePower + turnPower);
}