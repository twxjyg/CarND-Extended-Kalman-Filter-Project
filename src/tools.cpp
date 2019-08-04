#include "tools.h"
#include <iostream>

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.empty()) {
    return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
    return rmse;
  }

  Eigen::VectorXd error2_sum(4);
  error2_sum << 0, 0, 0, 0;
  for (int i = 0; i < estimations.size(); ++i) {
    // ... your code here
    Eigen::VectorXd error = estimations[i] - ground_truth[i];
    Eigen::VectorXd error2 = error.array() * error.array();
    error2_sum += error2;
  }

  error2_sum = error2_sum / estimations.size();

  rmse = error2_sum.array().sqrt();
  return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  Eigen::MatrixXd Hj = Eigen::MatrixXd::Zero(3, 4);
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);
  auto vxpy_sub_vypx = vx * py - vy * px;
  auto vypx_sub_vxpy = vy * px - vx * py;
  auto px2_add_py2 = px * px + py * py;
  if (std::abs(px2_add_py2) < 0.00001) {
    std::cerr << "Error: px2_add_py2 is zero" << std::endl;
    throw std::runtime_error("(px*px + py*py) equals Zero");
  }

  Hj(0, 0) = px / sqrt(px2_add_py2);
  Hj(0, 1) = py / sqrt(px2_add_py2);
  Hj(0, 2) = 0.0;
  Hj(0, 3) = 0.0;
  Hj(1, 0) = -py / px2_add_py2;
  Hj(1, 1) = px / px2_add_py2;
  Hj(1, 2) = 0.0;
  Hj(1, 3) = 0.0;
  Hj(2, 0) = (py * vxpy_sub_vypx) / pow(px2_add_py2, 1.5);
  Hj(2, 1) = (px * vypx_sub_vxpy) / pow(px2_add_py2, 1.5);
  Hj(2, 2) = px / sqrt(px2_add_py2);
  Hj(2, 3) = py / sqrt(px2_add_py2);
  return Hj;
}

Eigen::MatrixXd Tools::CalculateLinearMeasurementMatrix() {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  return H;
}

Eigen::MatrixXd Tools::CalculateLinearStateTransform(const double &det_t) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(4, 4);
  F(0, 2) = det_t;
  F(1, 3) = det_t;
  F(0, 0) = 1.0;
  F(1, 1) = 1.0;
  F(2, 2) = 1.0;
  F(3, 3) = 1.0;
  return F;
}

Eigen::MatrixXd Tools::CalculateProcessNoise(const double &det_t, const double &noise_ax, const double &noise_ay) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  auto det_t_4 = std::pow(det_t, 4);
  auto det_t_3 = std::pow(det_t, 3);
  auto det_t_2 = std::pow(det_t, 2);
  Q(0, 0) = (det_t_4 / 4) * noise_ax;
  Q(0, 2) = (det_t_3 / 2) * noise_ax;
  Q(1, 1) = (det_t_4 / 4) * noise_ay;
  Q(1, 3) = (det_t_3 / 2) * noise_ay;
  Q(2, 0) = (det_t_3 / 2) * noise_ax;
  Q(2, 2) = det_t_2 * noise_ax;
  Q(3, 1) = (det_t_3 / 2) * noise_ay;
  Q(3, 3) = det_t_2 * noise_ay;
  return Q;
}

Eigen::VectorXd Tools::CalculateMotionNoise(const double &det_t, const double &noise_ax, const double &noise_ay) {
  Eigen::VectorXd u = Eigen::VectorXd::Zero(4, 1);
  auto det_t_2 = std::pow(det_t, 2);
  auto noise_ax_std = std::sqrt(noise_ax);
  auto noise_ay_std = std::sqrt(noise_ay);
  u(0) = noise_ax_std * det_t_2 / 2.0;
  u(1) = noise_ay_std * det_t_2 / 2.0;
  u(2) = noise_ax_std * det_t;
  u(3) = noise_ay_std * det_t;
  return u;
}

Eigen::VectorXd Tools::TransformRadarMeasurementToState(const Eigen::VectorXd &radar_measurement) {
  const double &ro = radar_measurement(0);
  const double &theta = radar_measurement(1);
  const double &ro_dot = radar_measurement(2);
  return Eigen::Vector4d(
      {ro * std::sin(theta), ro * std::cos(theta), ro_dot * std::sin(theta), ro_dot * std::cos(theta)});
}

Eigen::VectorXd Tools::TransformLaserMeasurementToState(const Eigen::VectorXd &laser_measurement) {
  return Eigen::Vector4d({laser_measurement(0), laser_measurement(1), 0.0, 0.0});
}

Eigen::VectorXd Tools::TransformStateToRadarMeasurement(const Eigen::VectorXd &state) {
  Eigen::VectorXd radar_measurement = Eigen::VectorXd::Zero(3, 1);
  auto px = state(0);
  auto py = state(1);
  auto vx = state(2);
  auto vy = state(3);
  auto px2_add_py2 = std::pow(state(0), 2) + std::pow(state(1), 2);
  if (std::abs(px2_add_py2) < 0.0001) {
    throw std::runtime_error("(px*px + py*py) equals Zero");
  }
  radar_measurement(0) = std::sqrt(px2_add_py2);
  radar_measurement(1) = std::atan2(py, px);

  if (radar_measurement(1) > M_PI || radar_measurement(1) < -M_PI) {
    throw std::runtime_error("theta out of range");
  }
  radar_measurement(2) = (px * vx + py * vy) / std::sqrt(px2_add_py2);
  return radar_measurement;
}

Eigen::VectorXd Tools::TransformStateToLaserMeasurement(const Eigen::VectorXd &state) {
  Eigen::VectorXd laser_measurement = Eigen::VectorXd::Zero(2, 1);
  laser_measurement(0) = state(0);
  laser_measurement(1) = state(1);
  return laser_measurement;
}
