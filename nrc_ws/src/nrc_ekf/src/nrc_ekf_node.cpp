#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <fstream>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

#include "nrc_ekf/ekf.h"
#include "nrc_msgs/LocalizationVector.h"
#include "nrc_msgs/DriveStatus.h"
#include "nrc_msgs/motors.h"

// #include "igvc_msgs/EKFState.h"
// #include "igvc_msgs/EKFConvergence.h"
// #include "igvc_msgs/gps.h"
// #include "igvc_msgs/velocity.h"
// #include "igvc_msgs/EKFService.h"
// #include "igvc_msgs/motors.h"

// Init the EKF
EKF ekf;

// State and control vectors
Eigen::VectorXd x(8), u(2);

// Measurement vector
Eigen::VectorXd z(8);
double last_heading = -1010; // TODO: What?
double x_coord, y_coord;

ros::Publisher output_pub;

uint16_t data_init = 0;

nrc_msgs::LocalizationVector encodeEKFState(Eigen::VectorXd x)
{
    nrc_msgs::LocalizationVector ekf_state;

    ekf_state.x = x(0);
    ekf_state.y = x(1);

    return ekf_state;
}


void updateEKF(const ros::TimerEvent& time_event)
{
    // If all sensor data has been received once
    if (data_init == 1)
    {
        // EKF state message
        nrc_msgs::LocalizationVector state;

        // Update the extended kalman filter
        x = ekf.run_filter(z, u);

        // Publish predicted state
        output_pub.publish(encodeEKFState(x));
    }
}

/******
 * Helper functions
 ******/

float constrainAngle(float x){
    x = std::fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float angleDiff(float a,float b){
    float dif = std::fmod(b - a + 180,360);
    if (dif < 0)
        dif += 360;
    return dif - 180;
}



/******
 * Measurement callbacks
 ******/

void updateMeasurements(const nrc_msgs::DriveStatus::ConstPtr& vel_msg)
{
    // Update measurement
    z(2) = vel_msg->left_speed;
    z(3) = vel_msg->right_speed;
    z(4) = vel_msg->yaw;
    z(6) = 0.5 * WHEEL_RADIUS * (z(2) + z(3));
    z(7) = vel_msg->acceleration;

    // Show that the velocity has been updated
    data_init = 1;
}

void updateControlSignal(const nrc_msgs::motors::ConstPtr& motors)
{
    u(0) = motors->left;
    u(1) = motors->right;
}


void updatePosition(const ros::TimerEvent& timer_event)
{
    // // position initialization flag
    // data_init = 1;

    // // Compute time difference
    // double dt = (timer_event.current_real - timer_event.last_real).toSec();

    // // X component velocity and acceleration
    // double vx = z(6) * cos(z(5));
    // double ax = z(10) * cos(z(5));
    // x_coord += (vx * dt) + (ax * pow(dt, 2));

    // // Y component velocity and acceleration
    // double vy = z(6) * sin(z(5));
    // double ay = z(10) * sin(z(5));
    // y_coord += (vy * dt) + (ay * pow(dt, 2));

    // // Update the measurement vector
    // z(0) = x_coord;
    // z(1) = y_coord;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nrc_ekf_node");

    ros::NodeHandle ekf_node;

    // Initialize the measurement vector
    z.resize(8);
    z.setZero(8);

    // Initialize the x and y coordinates to 0 (the starting position)
    x_coord = 0;
    y_coord = 0;

    // Initialize the subscribers
    ros::Subscriber sensor_sub = ekf_node.subscribe(ekf_node.resolveName("/nrc/sensor_data"), 1, &updateMeasurements);
    ros::Subscriber motor_sub = ekf_node.subscribe(ekf_node.resolveName("/nrc/motors"), 1, &updateControlSignal);

    // Initialize the state and control vectors
    x.resize(8);
    u.resize(2);
    x << x_coord, y_coord, 0, 0, 0, 0, 0, 0;
    u << 0, 0;

    // Initialize the EKF
    ekf.init(x);

    // Publishers
    output_pub = ekf_node.advertise<nrc_msgs::LocalizationVector>(ekf_node.resolveName("/nrc/ekf_robot_state"), 10);
    // convergence_pub = ekf_node.advertise<igvc_msgs::EKFConvergence>(ekf_node.resolveName("/nrc_ekf/EKFConvergence"), 10);

    // Timers
    ros::Timer dead_reckon_update = ekf_node.createTimer(ros::Duration(0.01), &updatePosition, false);
    ros::Timer ekf_update = ekf_node.createTimer(ros::Duration(0.02), &updateEKF, false);
    // ros::Timer converge_update = ekf_node.createTimer(ros::Duration(0.025), &updateConvergence, false);

    ros::spin();

    return 0;
}