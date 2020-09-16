#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix- laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Process noise
  noise_ax = 9;
  noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // Initialise the state ekf_.x_ with the first measurement
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // State covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert from polar radar coordinates to cartesian coordinates for kalman filter
      ekf_.x_(0) = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_(1) = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      ekf_.x_(2) = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_(3) = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);

      // measurement matrix- radar
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      // Measurement covariance matrix
      ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;

      // Measurement matrix- laser
      ekf_.H_ = H_laser_;
      // Measurement covariance matrix
      ekf_.R_ = R_laser_;
    }

    // the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    size_t dim = ekf_.x_.size();
    ekf_.I_ = MatrixXd::Identity(dim, dim);

    // Initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "EKF successfully initialised." << endl;
    return;
  }

  /**
   * Prediction
   */
  // Compute time elapsed between measurements in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F according to the new elapsed time
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Update the process noise covariance matrix.
  float c1 = pow(dt, 2.0);
  float c2 = pow(dt, 3.0) / 2.0;
  float c3 = c1 * c1 / 4.0;

  ekf_.Q_ = MatrixXd(4,4); 
  ekf_.Q_ << c3 * noise_ax, 0, c2 * noise_ax, 0,
            0, c3 * noise_ay, 0, c2 * noise_ay,
            c2 * noise_ax, 0, c1 * noise_ax, 0,
            0, c2 * noise_ay, 0, c1 * noise_ay;

  // Predict the next state
  ekf_.Predict();

  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // If measurement is radar, calculate the Jacobian to linearise the measurement function
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // Update Kalman Filter to use radar covariance matrix
    ekf_.R_ = R_radar_;
    // Update state using extended Kalman Filter
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // If laser measurment, update measurement and covariance matrix
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // Update state using normal Kalman Filter
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
