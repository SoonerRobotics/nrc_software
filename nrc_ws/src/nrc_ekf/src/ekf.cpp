#include "nrc_ekf/ekf.h"


EKF::EKF()
{
    // set the covariance matrix to the identity matrix
    this->P_k.setIdentity(8, 8);
    this->P_k *= 1.0; // TODO: what good here?

    // Define the process noise
    this->Q_k.setIdentity(8, 8);
    this->Q_k *= 0.2; // TODO: what good here?
    this->Q_k(0, 0) = 5; // x and y are basically garbage lol
    this->Q_k(1, 1) = 5;

    // Define the measurement noise
    this->R_k.setIdentity(8, 8);
    this->R_k(0, 0) = 2;
    this->R_k(1, 1) = 2;
    this->R_k(4, 4) = 2;

    // Define the measurement model
    this->H_k.setIdentity(8, 8);

    // Initialize the kalman gain
    this->K_k.setZero(8, 8); // TODO: Why was this 11, 9 before instead of 11, 11?

    // Setup identity matrix
    this->I.setIdentity(8, 8);
}


void EKF::init(Eigen::VectorXd x0)
{
    // set the initial state
    this->x_k = x0;

    // init the timer
    this->last_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
}


Eigen::VectorXd EKF::run_filter(Eigen::VectorXd sensors, Eigen::VectorXd u_k)
{
    // local vars
    double cur_time;
    double dt;

    // find time delta
    cur_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
    dt = cur_time - last_time;
    last_time = cur_time;

    // Run prediction
    this->predict(u_k, dt);

    // Update from prediction
    this->update(sensors);

    // Return the new state estimate
    return this->x_k;
}


double EKF::get_convergence()
{
    return this->convergence;
}




void EKF::calculate_dynamics(Eigen::VectorXd u_k, double dt)
{
    // Velocity calculations
    double velocity = 0.5 * WHEEL_RADIUS * (x_k(2) + x_k(3));
    double x_dot    = velocity * cos(x_k(4));
    double y_dot    = velocity * sin(x_k(4));
    double theta_dot  = (WHEEL_RADIUS / WHEELBASE_LEN) * (x_k(3) - x_k(2));

    // Position Calculations
    double x     = x_k(0) + x_dot * dt;
    double y     = x_k(1) + y_dot * dt;
    double theta = x_k(4) + theta_dot * dt;

    // Local Orientation
    x_k(0) = x;
    x_k(1) = y;

    // Wheel velocities
    x_k(2) = u_k(0);
    x_k(3) = u_k(1);

    // Rotation
    x_k(4) = theta;
    x_k(5) = theta_dot;

    // Velocities
    x_k(6) = velocity;
    x_k(7) = x_k(7);
}


void EKF::linear_dynamics(Eigen::VectorXd u_k, double dt)
{
    double velocity = x_k(6);
    double left_vel = x_k(2);
    double right_vel = x_k(3);

    double sin_theta = sin(x_k(4));
    double cos_theta = cos(x_k(4));

    this->F_k.setIdentity(8, 8);

    // x
    F_k(0, 2) = 0.5 * WHEEL_RADIUS  * dt * cos_theta;
    F_k(0, 3) = 0.5 * WHEEL_RADIUS  * dt * cos_theta;
    F_k(0, 4) = -dt * (0.5 * WHEEL_RADIUS * (left_vel + right_vel)) * sin_theta;

    // y
    F_k(1, 2) = 0.5 * WHEEL_RADIUS  * dt * sin_theta;
    F_k(1, 3) = 0.5 * WHEEL_RADIUS  * dt * sin_theta;
    F_k(1, 4) = dt * (0.5 * WHEEL_RADIUS * (left_vel + right_vel)) * cos_theta;

    // theta
    F_k(4, 2) = -(WHEEL_RADIUS / WHEELBASE_LEN) * dt;
    F_k(4, 3) = (WHEEL_RADIUS / WHEELBASE_LEN) * dt;

    // theta_dot
    F_k(5, 2) = -(WHEEL_RADIUS / WHEELBASE_LEN);
    F_k(5, 3) = (WHEEL_RADIUS / WHEELBASE_LEN);

    // velocity
    F_k(6, 2) = 0.5 * WHEEL_RADIUS;
    F_k(6, 3) = 0.5 * WHEEL_RADIUS;
}


void EKF::predict(Eigen::VectorXd u_k, double dt)
{
    // Predict current state from past state and control signal
    this->calculate_dynamics(u_k, dt);

    // Linearize the dynamics using a jacobian
    this->linear_dynamics(u_k, dt);

    // Update the covariance matrix
    this->P_k = (F_k * P_k * F_k.transpose()) + this->Q_k;
}




Eigen::VectorXd EKF::get_measurement_model()
{
    // We compare the measurements, z_k, to the current state as a one-to-one mapping
    return this->x_k;
}



void EKF::update(Eigen::VectorXd z_k)
{
    // Calculate the innovation (the difference between predicted measurements and the actual measurements)
    this->yk = z_k - get_measurement_model();

    // Find the covariance of the innovation
    this->Sk = (this->H_k * this->P_k * this->H_k.transpose()) + this->R_k;

    // Convergence calculation
    this->convergence = this->yk.transpose() * this->Sk.inverse() * this->yk;

    // Compute Kalman gain
    this->K_k = this->P_k * H_k.transpose() * Sk.inverse();

    // Use the kalman gain to update the state estimate
    this->x_k = this->x_k + this->K_k * yk;

    // Likewise, update the covariance for the state estimate
    this->P_k = (this->I - this->K_k * this->H_k) * this->P_k;
}