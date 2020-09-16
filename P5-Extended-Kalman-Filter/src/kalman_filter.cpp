#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * Predict the state at time k+1 using the state transition function (i.e. linear motion model).
 * Uncertainty about the state is modelled in the process covariance matrix, P.
 */
void KalmanFilter::Predict() {
  // State transition
  x_ = F_ * x_;
  // State covariance
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Update the state by using Kalman Filter equations
 */
void KalmanFilter::Update(const VectorXd &z) {
  // Error given new measurement
  VectorXd y = z - H_ * x_;
  UpdateKF(y);
}
#include <iostream>
/**
 * Update the state by using Extended Kalman Filter equations
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Convert x' to polar coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px);

  // Normalise theta to between -pi and pi
  while (theta > M_PI || theta < -M_PI) {
    std::cout << theta << std::endl;
    if (theta > M_PI) {
      theta -= 2 * M_PI;
    } else if (theta < -M_PI) {
      theta += 2 * M_PI;
    }
  }

  // Check for division by zero
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0.0;
  } else {
    rho_dot = (px * vx + py * vy) / rho;
  }

  VectorXd z_pred(3);
  z_pred << rho, theta, rho_dot;
  // Calculate error given measurement
  VectorXd y = z - z_pred;

  // Calculate rest of the equations as normal
  UpdateKF(y);
}

void KalmanFilter::UpdateKF(const Eigen::VectorXd &y) {
  // Measurement uncertainty
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  // Kalman gain (weighted combination of prediction and measurement uncertainty)
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // Updated state and covariance
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
}
