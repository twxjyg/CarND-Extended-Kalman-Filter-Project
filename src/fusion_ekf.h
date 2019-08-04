#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  Eigen::VectorXd GetLatestEstimate();

  Eigen::MatrixXd GetLatestCovariance();

 private:
  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  float noise_ax_;
  float noise_ay_;
};

#endif  // FUSION_EKF_H_
