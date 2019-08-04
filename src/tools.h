#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * A helper method to calculate RMSE.
   */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * A helper method to calcute linear measurement matrix
   */
  static Eigen::MatrixXd CalculateLinearMeasurementMatrix();

  /**
   * A helper methond to calculate linear State Transform Matrix
   */
  static Eigen::MatrixXd CalculateLinearStateTransform(const double& det_t);

  /**
   * A helper method to calculate process noise Matrix
   */
  static Eigen::MatrixXd CalculateProcessNoise(const double& det_t, const double& noise_ax, const double& noise_ay);

  /**
   * A helper method to calculate motion noise Vector
   */
  static Eigen::VectorXd CalculateMotionNoise(const double& det_t, const double& noise_ax, const double& noise_ay);
  /**
   * A helper method to transform radar measurment to EKF state vector
   */
  static Eigen::VectorXd TransformRadarMeasurementToState(const Eigen::VectorXd& radar_measurement);

  /**
   * A helper method to transform laser measurement to EKF state vector
   */
  static Eigen::VectorXd TransformLaserMeasurementToState(const Eigen::VectorXd& laser_measurement);

  /**
   * A helper method to transform state vector to radar measurement space
   */
  static Eigen::VectorXd TransformStateToRadarMeasurement(const Eigen::VectorXd& state);

  /**
   * A helper method to transform state vector to laser measurement space
   */
  static Eigen::VectorXd TransformStateToLaserMeasurement(const Eigen::VectorXd& state);

};

#endif  // TOOLS_H_
