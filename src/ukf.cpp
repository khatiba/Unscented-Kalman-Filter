#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.39;

  // Timestamp of last measurement
  time_us_ = 0.0;

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

  ///* State dimension
  n_x_ = 5;

  P_ = MatrixXd::Identity(n_x_, n_x_);

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  weights_ = VectorXd(2*n_aug_ + 1);

  double weight = 1 / (2 * (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_+1; i++) {
      weights_(i) = weight;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_pack) {
  if (!is_initialized_) {
    if (meas_pack.sensor_type_ == MeasurementPackage::RADAR) {
      InitRadar(meas_pack);
    }
    if (meas_pack.sensor_type_ == MeasurementPackage::LASER) {
      InitLidar(meas_pack);
    }

    time_us_ = meas_pack.timestamp_;

    is_initialized_ = true;

    return;
  }

  double delta_t = (meas_pack.timestamp_ - time_us_) / 1000000.0; // in seconds
  time_us_ = meas_pack.timestamp_;

  if (delta_t > 0.001) {
    Prediction(delta_t);
  }

  if (meas_pack.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_pack);
  }
  if (meas_pack.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_pack);
  }
}

void UKF::InitLidar(MeasurementPackage meas_pack) {
  float px = meas_pack.raw_measurements_(0);
  float py = meas_pack.raw_measurements_(1);

  x_ << px,
        py,
        0,
        0,
        0;
}

void UKF::InitRadar(MeasurementPackage meas_pack) {
  // convert from polar to cartesian coordinates and initialize state
  float rho     = meas_pack.raw_measurements_(0);
  float phi     = meas_pack.raw_measurements_(1);
  float rho_dot = meas_pack.raw_measurements_(2);

  float vx = rho_dot*cos(phi);
  float vy = rho_dot*sin(phi);
  float v  = sqrt(vx*vx + vy*vy);

  x_ << rho*cos(phi),
        rho*sin(phi),
        v,
        0,
        0;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug.tail(n_aug_-n_x_) << 0, 0;

  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd x_sig = Xsig_aug.col(i);

    double v_k = x_sig(2);
    double psi_k = x_sig(3);
    double psi_dot_k = x_sig(4);
    double delta_t_2 = delta_t * delta_t;

    VectorXd noise = VectorXd(n_x_);
    noise << 0.5 * delta_t_2 * cos(psi_k) * x_sig(5),
             0.5 * delta_t_2 * sin(psi_k) * x_sig(5),
             delta_t * x_sig(5),
             0.5 * delta_t_2 * x_sig(6),
             delta_t * x_sig(6);

    VectorXd f = VectorXd(n_x_);
    if (x_sig(4) == 0) {
      f << v_k * cos(psi_k) * delta_t,
           v_k * sin(psi_k) * delta_t,
           0,
           0,
           0;
    } else {
      f << (v_k/psi_dot_k) * (sin(psi_k + psi_dot_k * delta_t) - sin(psi_k)),
           (v_k/psi_dot_k) * (-cos(psi_k + psi_dot_k * delta_t) + cos(psi_k)),
           0,
           psi_dot_k * delta_t,
           0;
    }

    Xsig_pred_.col(i) = x_sig.head(n_x_) + f + noise;
  }

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {  //iterate over sigma points
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_pack) {
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;

  double nis = UpdateUKF(meas_pack, Zsig, R);
  /* std::cout << nis << std::endl; */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_pack) {
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0,std_radrd_*std_radrd_;

  double nis = UpdateUKF(meas_pack, Zsig, R);
  /* std::cout << nis << std::endl; */
}

double UKF::UpdateUKF(MeasurementPackage meas_pack, MatrixXd Zsig, MatrixXd R) {
  int n_z = Zsig.rows();

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {  //2n+1 simga points
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {  //2n+1 simga points

    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  VectorXd z_diff = meas_pack.raw_measurements_ - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

  // update state mean and covariance matrix
  x_ += K * z_diff;
  P_ += -K * S * K.transpose();

  //calculate NIS
  double nis = z_diff.transpose() * S.inverse() * z_diff;

  return nis;
}

