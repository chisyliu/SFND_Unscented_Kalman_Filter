#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <iostream>

class UKF
{
public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * InitializeStates
   * @brief Initialize state x_ and Covariance matrix P_ with the first measurement
   */
  void InitializeStates(const MeasurementPackage &meas_package);

  /**
   * GenerateWeights
   * @brief Generates weights for sigma points and covariance matrix
   */
  void GenerateWeights();

  /**
   * GenerateMeasurementNoiseCovarianceMatrices
   * @brief Generates measurement noise covariance matrices for Radar and Lidar
   */
  void GenerateMeasurementNoiseCovarianceMatrices();

  /**
   * GenerateSigmaPoints
   * @brief Generates sigma points based on posteior distribution(x_ and P_)
   */
  void GenerateSigmaPoints();

  /**
   * GenerateAugmentedSigmaPoints
   * @brief Generates augmented sigma points based on x_aug_ and P_aug_
   */
  void GenerateAugmentedSigmaPoints();

  /**
   * PredictSigmaPoints
   * @brief Predicts sigma points by processing augmented sigma points
   */
  void PredictSigmaPoints(const double dt);

  /**
   * PredictMeanState
   * @brief Predicts mean state with weights
   */
  void PredictMeanState();

  /**
   * PredictCovarianceMatrix
   * @brief Predicts covariance matrix with weights
   */
  void PredictCovarianceMatrix();

  /**
   * TransformSigmaPointsToRadarSpace
   * @brief Transform predicted sigma points into Radar measurement space
   */
  void TransformSigmaPointsToRadarSpace();

  /**
   * TransformSigmaPointsToLidarSpace
   * @brief Transform predicted sigma points into Lidar measurement space
   */
  void TransformSigmaPointsToLidarSpace();

  /**
   * PredictRadarMeanState
   * @brief Predict Radar measurement mean state
   */
  void PredictRadarMeanState();

  /**
   * PredictLidarMeanState
   * @brief Predict Lidar measurement mean state
   */
  void PredictLidarMeanState();

  /**
   * PredictRadarCovariance
   * @brief Predict Radar measurement covariance matrix
   */
  void PredictRadarCovariance();

  /**
   * PredictLidarCovariance
   * @brief Predict Lidar measurement covariance matrix
   */
  void PredictLidarCovariance();

  /**
   * PredictRadarMeasurement
   * @brief Predict Radar measurement with sigma points in radar measurement space
   */
  void PredictRadarMeasurement();

  /**
   * PredictLidarMeasurement
   * @brief Predict Lidar measurement with sigma points in radar measurement space
   */
  void PredictLidarMeasurement();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double &dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage &meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage &meas_package);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos_x pos_y vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // augmented state vector: [pos_x pos_y vel_abs yaw_angle yaw_rate std_a std_yawdd] in SI units and rad
  Eigen::VectorXd x_aug_;

  // predicted radar measurement : [rho phi rho_d]
  Eigen::VectorXd z_radar_pred_;

  // predicted lidar measurement : [pos_x pos_y]
  Eigen::VectorXd z_lidar_pred_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // augmented state covariance matrix
  Eigen::MatrixXd P_aug_;

  // predicted radar measurement covariance matrix
  Eigen::MatrixXd S_radar_pred_;

  // predicted lidar measurement covariance matrix
  Eigen::MatrixXd S_lidar_pred_;

  // radar measurement noise covariance matrix
  Eigen::MatrixXd R_radar_;

  // lidar measurement noise covariance matrix
  Eigen::MatrixXd R_lidar_;

  // sigma point matrix
  Eigen::MatrixXd Xsigma_;

  // augmented sigma point matrix
  Eigen::MatrixXd Xsigma_aug_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsigma_pred_;

  // radar measurement sigma points matrix
  Eigen::MatrixXd Zsigma_radar_;

  // lidar measurement sigma points matrix
  Eigen::MatrixXd Zsigma_lidar_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Radar measurement dimension
  int n_z_radar_;

  // Lidar measurement dimension
  int n_z_lidar_;

  // Sigma point spreading parameter
  double lambda_;

  // Augmented sigma point spreading parameter
  double lambda_aug_;
};

#endif // UKF_H