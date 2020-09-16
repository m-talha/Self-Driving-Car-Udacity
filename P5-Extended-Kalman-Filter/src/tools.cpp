#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Validate input
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground truth data." << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (auto i=0; i < estimations.size(); ++i) {
    VectorXd residual = ground_truth[i] - estimations[i];
    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse /= estimations.size();
  // Calculate the square root
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd jacobian(3,4);

  // recover state parameters
  float px = x_state(0);   
  float py = x_state(1);   
  float vx = x_state(2);   
  float vy = x_state(3);

  // pre-compute repeated values
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  // check division by 0
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() - Error: Division by zero" << endl;
    return jacobian;
  }

  // compute the jacobian
  jacobian << px/c2, py/c2, 0, 0,
      -py/c1, px/c1, 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return jacobian;    
}
