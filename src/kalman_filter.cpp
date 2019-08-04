#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  Eigen::VectorXd Y = z - Tools::TransformStateToLaserMeasurement(x_);
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * Y);
  P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  Eigen::VectorXd Y = z - Tools::TransformStateToRadarMeasurement(x_);
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * Y);
  P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
}
