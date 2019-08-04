#include "fusion_ekf.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_radar_ = Eigen::MatrixXd(3, 3);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // accelaration noise
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  /**
   * Initialization
   */
  std::cout << measurement_pack.ToString() << std::endl;
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    std::cout << "EKF: " << std::endl;
    ekf_.x_ = Eigen::VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.P_ = Eigen::MatrixXd::Zero(4, 4);
    ekf_.P_(0, 0) = 3;
    ekf_.P_(1, 1) = 3;
    ekf_.P_(2, 2) = 3;
    ekf_.P_(3, 3) = 3;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      std::cout << "initialize with radar measurement" << std::endl;
      ekf_.x_ = Tools::TransformRadarMeasurementToState(measurement_pack.raw_measurements_);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      std::cout << "initialize with laser measurement" << std::endl;
      ekf_.x_ = Tools::TransformLaserMeasurementToState(measurement_pack.raw_measurements_);
    } else {
      std::cerr << "Unknown sensor type:" << measurement_pack.sensor_type_ << std::endl;
      return;
    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    // print the output
    std::cout << "x_ = " << ekf_.x_.transpose() << std::endl;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double det_t = static_cast<double>(measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  ekf_.F_ = Tools::CalculateLinearStateTransform(det_t);
  ekf_.Q_ = Tools::CalculateProcessNoise(det_t, noise_ax_, noise_ay_);

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = Tools::CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = Tools::CalculateLinearMeasurementMatrix();
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    std::cerr << "Unknown sensor type:" << measurement_pack.sensor_type_ << std::endl;
    return;
  }
  // Update previous processed timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  std::cout << "x_ = " << ekf_.x_.transpose() << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
Eigen::VectorXd FusionEKF::GetLatestEstimate() { return ekf_.x_; }

Eigen::MatrixXd FusionEKF::GetLatestCovariance() { return ekf_.P_; }