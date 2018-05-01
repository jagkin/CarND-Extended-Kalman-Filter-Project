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
  unsigned int num_entries = estimations.size();
  if (num_entries == 0) {
    cout << "Tools::CalculateRMSE: Invalid Estimation vector\n";
    return rmse;
  }

  if (num_entries != ground_truth.size()) {
    cout << "Tools::CalculateRMSE: Estimation and ground_truth vectors are of different sizes\n";
    return rmse;
  }

  // Initialize sum of squared diffs
  unsigned int dims = estimations[0].size();
  VectorXd sum_diffs(dims);
  sum_diffs = VectorXd::Zero(dims);
  for (unsigned int i = 0u; i != num_entries; i++) {
    // Calculate diff
    VectorXd diff = estimations[i] - ground_truth[i];
    // Add to sum of squared diffs
    VectorXd diff_sqrd = diff.array() * diff.array();
    sum_diffs += diff_sqrd;
  }
  // Calculate mean
  VectorXd mean = sum_diffs/num_entries;

  // Calculate and return RMSE
  rmse = mean.array().sqrt();

#if VERBOSE_PRINTS
  // Dump latest estimations/ground truth and RMSE to console
  cout << "Estimation:   ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << estimations[num_entries-1](i) << "\t";
  }
  cout << endl;

  cout << "Ground truth: ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << ground_truth[num_entries-1](i) << "\t";
  }
  cout << endl;

  cout << "RMSE:         ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << rmse(i) << "\t";
  }
  cout << endl << endl;
#endif

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  if (x_state.size() != 4) {
    cout << "ERROR in Tools::CalculateJacobian: Invalid x_state\n";
    return Hj;
  }
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if ((px == 0) && (py == 0)) {
      cout << "ERROR in Tools::CalculateJacobian: Error - Divide by 0\n";
      return Hj;
  }
  float px2_py2 = px*px + py*py;
  //compute the Jacobian matrix
    Hj << (px/sqrt(px2_py2)),                     (py/sqrt(px2_py2)),                     0,                  0,
          (-py/(px2_py2)),                        (px/px2_py2),                           0,                  0,
          py*(vx*py - vy*px)/(pow(px2_py2, 1.5)), px*(vy*px - vx*py)/(pow(px2_py2, 1.5)), (px/sqrt(px2_py2)), (py/sqrt(px2_py2));
  return Hj;
}
