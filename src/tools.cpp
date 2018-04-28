#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse;
  if (estimations.size() != ground_truth.size()) {
    cout << "Tools::CalculateRMSE: Estimation and ground_truth vectors are of different sizes\n";
    return rmse;
  }
  if (estimations.size() == 0) {
    cout << "Tools::CalculateRMSE: Invalid Estimation vector\n";
    return rmse;
  }
  if (ground_truth.size() == 0) {
    cout << "Tools::CalculateRMSE: Invalid Ground truth vector\n";
    return rmse;
  }

  // Initialize sum of squared diffs
  int rows = estimations[0].size();
  VectorXd sum_diffs(4);
  sum_diffs << 0, 0, 0, 0;
  for (unsigned int i = 0u; i != estimations.size(); i++) {
    // Calculate diff
    VectorXd diff = estimations[i] - ground_truth[i];
    // Add to sum of squared diffs
    VectorXd diff_sqrd = diff.array() * diff.array();
    sum_diffs += diff_sqrd;
  }
  // Calculate mean
  VectorXd mean = sum_diffs/estimations.size();
  // Calculate and return RMSE
  rmse = mean.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  if (x_state.size() != 4) {
    cout << "Tools::CalculateJacobian: Invalid x_state\n";
    return Hj;
  }
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if ((px == 0) && (py == 0)) {
      cout << "Tools::CalculateJacobian: Error - Divide by 0\n";
      return Hj;
  }
  float px2_py2 = px*px + py*py;
  //compute the Jacobian matrix
    Hj << (px/sqrt(px2_py2)), (py/sqrt(px2_py2)), 0, 0,
          (-py/(px2_py2)), (px/px2_py2), 0 , 0,
          py*(vx*py - vy*px)/(pow(px2_py2, 1.5)), px*(vy*px - vx*py)/(pow(px2_py2, 1.5)), (px/sqrt(px2_py2)), (py/sqrt(px2_py2));
  return Hj;
}
