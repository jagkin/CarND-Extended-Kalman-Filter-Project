#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  I_ = MatrixXd::Identity(x_.size(), x_.size());
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

#define DISABLE_LASER 0
void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
#if DISABLE_LASER
  return;
#endif

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  /* update state */
  x_ = x_ + (K * y);

  /* update state covariance matrix */
  P_ = (I_ - K * H_) * P_;
}

#define DISABLE_RADAR 0
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
#if DISABLE_RADAR
  return;
#endif

  VectorXd Hofx(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = hypot(px, py);
  float rho_dot = (px*vx + py*vy)/rho;
  float theta = atan2(py, px);
  Hofx << rho, theta, rho_dot;
  VectorXd y = z - Hofx;

  // Normalize phi to be in -PI to PI range
  float phi = y(1);
  if (phi < -M_PI) {
    y(1) = phi + 2 * M_PI;
  } else if (phi > M_PI) {
    y(1) = phi - 2 * M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  /* update state */
  x_ = x_ + (K * y);

  /* update state covariance matrix */
  P_ = (I_ - K * H_) * P_;
}
