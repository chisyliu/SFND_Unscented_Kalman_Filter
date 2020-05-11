#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // Set UKF parameters

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Radar measurement dimension
  n_z_radar_ = 3;

  // Lidar measurement dimension
  n_z_lidar_ = 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Augmented sigma point spreading parameter
  lambda_aug_ = 3 - n_aug_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial augmented state vector
  x_aug_ = VectorXd(n_aug_);

  // initial predicted radar measurement
  z_radar_pred_ = VectorXd(n_z_radar_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // initial augmented covariance matrix
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  // initial predicted radar measurement covariance matrix
  S_radar_pred_ = MatrixXd(n_z_radar_, n_z_radar_);

  // initial sigma point matrix
  Xsigma_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  // initial augmented sigma point matrix
  Xsigma_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // initial predicted sigma point matrix
  Xsigma_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial radar measurement sigma points matrix
  Zsigma_radar_ = MatrixXd(n_z_radar_, Xsigma_pred_.cols());

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  GenerateWeights();
  GenerateMeasurementNoiseCovarianceMatrices();
}

UKF::~UKF() {}

void UKF::GenerateWeights()
{
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_aug_ / (lambda_aug_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++)
  {
    weights_(i) = 1.0 / (2 * (lambda_aug_ + n_aug_));
  }
}

void UKF::GenerateMeasurementNoiseCovarianceMatrices()
{
  // Generate noise covariance matrix for Radar
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  // Generate noise covariance matrix for Lidar
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
}

void UKF::GenerateSigmaPoints()
{
  MatrixXd A = P_.llt().matrixL();
  Xsigma_.col(0) = x_;
  for (int i = 0; i < n_x_; ++i)
  {
    Xsigma_.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsigma_.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
}

void UKF::GenerateAugmentedSigmaPoints()
{
  // add augmented part to mean state
  x_aug_.head(n_x_) = x_;
  x_aug_(n_x_) = std_a_;
  x_aug_(n_x_ + 1) = std_yawdd_;

  // add augmented part to covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_, n_x_) = std_a_ * std_a_;
  P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd A_aug = P_aug_.llt().matrixL();
  Xsigma_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsigma_aug_.col(i + 1) = x_aug_ + sqrt(lambda_aug_ + n_aug_) * A_aug.col(i);
    Xsigma_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_aug_ + n_aug_) * A_aug.col(i);
  }
}

void UKF::PredictSigmaPoints(const double dt)
{
  double epsilon = 1e-5; // equal to zero threshold for yaw_rate
  for (int i = 0; i < Xsigma_pred_.cols(); i++)
  {
    // Read data from augmented sigma point
    double pos_x = Xsigma_aug_(0, i);
    double pos_y = Xsigma_aug_(1, i);
    double vel_abs = Xsigma_aug_(2, i);
    double yaw_angle = Xsigma_aug_(3, i);
    double yaw_rate = Xsigma_aug_(4, i);
    double std_a = Xsigma_aug_(5, i);
    double std_yawdd = Xsigma_aug_(6, i);

    // Sigma point that will be predicted
    double pos_x_pred;
    double pos_y_pred;
    double vel_abs_pred;
    double yaw_angle_pred;
    double yaw_rate_pred;

    // Apply CTRV motion model
    if (std::abs(yaw_rate) < epsilon) // if yaw_rate is 0
    {
      pos_x_pred = pos_x + vel_abs * dt * std::cos(yaw_angle);
      pos_y_pred = pos_y + vel_abs * dt * std::sin(yaw_angle);
    }
    else
    {
      pos_x_pred = pos_x + (vel_abs / yaw_rate) * (std::sin(yaw_angle + yaw_rate * dt) - std::sin(yaw_angle));
      pos_y_pred = pos_y + (vel_abs / yaw_rate) * (std::cos(yaw_angle) - std::cos(yaw_angle + yaw_rate * dt));
    }
    vel_abs_pred = vel_abs;
    yaw_angle_pred = yaw_angle + yaw_rate * dt;
    yaw_rate_pred = yaw_rate;

    // Add noise
    pos_x_pred += 0.5 * dt * dt * std::cos(yaw_angle) * std_a;
    pos_y_pred += 0.5 * dt * dt * sin(yaw_angle) * std_a;
    vel_abs_pred += std_a * dt;
    yaw_angle_pred += 0.5 * dt * dt * std_yawdd;
    yaw_rate_pred += std_yawdd * dt;

    // Write the predicted sigma point
    Xsigma_pred_(0, i) = pos_x_pred;
    Xsigma_pred_(1, i) = pos_y_pred;
    Xsigma_pred_(2, i) = vel_abs_pred;
    Xsigma_pred_(3, i) = yaw_angle_pred;
    Xsigma_pred_(4, i) = yaw_rate_pred;
  }
}

void UKF::PredictMeanState()
{
  VectorXd x_pred = VectorXd(n_x_);
  x_pred.fill(0.0);
  for (int i = 0; i < weights_.size(); i++)
  {
    x_pred += weights_(i) * Xsigma_pred_.col(i);
  }
  x_ = x_pred;
}

void UKF::PredictCovarianceMatrix()
{
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);
  P_pred.fill(0.0);
  VectorXd x_diff;
  for (int i = 0; i < weights_.size(); i++)
  {
    x_diff = Xsigma_pred_.col(i) - x_;

    // Normalize yaw angle
    while (x_diff(3) > M_PI)
    {
      x_diff(3) -= 2. * M_PI;
    }
    while (x_diff(3) < -M_PI)
    {
      x_diff(3) += 2. * M_PI;
    }

    P_pred += weights_(i) * (x_diff) * (x_diff).transpose();
  }
  P_ = P_pred;
}

void UKF::TransformSigmaPointsToRadarSpace()
{
  for (int i = 0; i < Xsigma_pred_.cols(); i++)
  {
    // Read needed sigma point info
    double pos_x = Xsigma_pred_(0, i);
    double pos_y = Xsigma_pred_(1, i);
    double vel_abs = Xsigma_pred_(2, i);
    double yaw_angle = Xsigma_pred_(3, i);

    double v_x = vel_abs * std::cos(yaw_angle);
    double v_y = vel_abs * std::sin(yaw_angle);

    // Transform sigma point into measurement space
    double tho = std::sqrt(pos_x * pos_x + pos_y * pos_y);
    double phi = std::atan2(pos_y, pos_x);
    double tho_d = (pos_x * v_x + pos_y * v_y) / std::sqrt(pos_x * pos_x + pos_y * pos_y);

    // Write transformed sigma point
    Zsigma_radar_(0, i) = tho;
    Zsigma_radar_(1, i) = phi;
    Zsigma_radar_(2, i) = tho_d;
  }
}

void UKF::PredictRadarMeanState()
{
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i = 0; i < weights_.size(); ++i)
  {
    z_pred += weights_(i) * Zsigma_radar_.col(i);
  }
  z_radar_pred_ = z_pred;
}

void UKF::PredictRadarCovariance()
{
    MatrixXd S_radar_pred = MatrixXd(n_z_radar_, n_z_radar_);
    S_radar_pred.fill(0.0);
    VectorXd z_diff;
    for (int i = 0; i < weights_.size(); i++)
    {
      z_diff = Zsigma_radar_.col(i) - z_radar_pred_;

      // Normalize yaw angle
      while (z_diff(1) > M_PI)
      {
        z_diff(1) -= 2. * M_PI;
      }
      while (z_diff(1) < -M_PI)
      {
        z_diff(1) += 2. * M_PI;
      }

      S_radar_pred += weights_(i) * (z_diff) * (z_diff).transpose();
    }

    // Add measurement noise
    S_radar_pred += R_radar_;

    S_radar_pred_ = S_radar_pred;
}

void UKF::PredictRadarMeasurement()
{
  PredictRadarMeanState();
  PredictRadarCovariance();
}

void UKF::ProcessLidarMeasurement() {}

void UKF::ProcessMeasurement(MeasurementPackage &meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    TransformSigmaPointsToRadarSpace();
    PredictRadarMeasurement();
  }
  else
  {
    std::cout << "Unknown sensor type!" << std::endl;
  }
}

void UKF::Prediction(double dt)
{
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

  GenerateSigmaPoints();
  GenerateAugmentedSigmaPoints();
  PredictSigmaPoints(dt);
  PredictMeanState();
  PredictCovarianceMatrix();
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}