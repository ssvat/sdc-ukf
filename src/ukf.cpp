
#include "ukf.h"
#include "tools.h"
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

  //Tuning session: the process noise parameters
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.2; //Initial value set 0.2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; //Initial value set 0.2
  // Tuning ended
  
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
  The initialization. Set state dimension
  */
  n_x_ = x_.size();
  
  //set augmented dimension
  n_aug_ = n_x_ + 2;
  
  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  //define spreading parameter
  lambda_ = 3 - n_aug_;
  
  //set vector for weights for 2*n_aug+1 
  weights_ = VectorXd(n_sig_);

  //create example matrix with predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Add measurement noise covariance matrix of radar
  // The radar measurements include r, phi, and r_dot
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<    std_radr_*std_radr_, 0, 0,
                 0, std_radphi_*std_radphi_, 0,
                 0, 0,std_radrd_*std_radrd_;
  
  //add measurement noise covariance matrix of lidar
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ <<    std_laspx_*std_laspx_, 0,
                 0, std_laspy_*std_laspy_;

  // Start time
  time_us_ = 0;

  // Set NIS
  NIS_radar_ = 0;
  NIS_laser_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Switch between lidar and radar measurements.
  */
  if (!is_initialized_) {
    
    // Initialize covariance matrix as identy matrix
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
       float rho     = meas_package.raw_measurements_(0); //range
       float phi     = meas_package.raw_measurements_(1); //bearing
       float rho_dot = meas_package.raw_measurements_(2); //velocity

    //transform sigma points into measurement space
    // extract values for better readibility
       float p_x  = rho*cos(phi);
       float p_y  = rho*sin(phi);
       float v_x  = rho_dot*cos(phi);
       float v_y  = rho_dot*sin(phi);
       float v = sqrt(v_x * v_x + v_y * v_y);

       //       x_ << p_x, p_y, v, 0.0, 0.0;
       // polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
       // ***** Middle value for 'v' can be tuned *****
       x_ << p_x, p_y, 4, v_x, v_y;

       //state covariance matrix to be tuned
       //P_ << std_radr_*std_radr_, 0, 0, 0, 0,
       //     0, 1, 0, 0, 0,
       //    0, 0, 1, 0, 0,
       //    0, 0, 0, 1, 0,
       //    0, 0, 0, 0, 1;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

       x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
       // To avoid divergence
       if (x_(0) <0.0001 and x_(1) <0.0001){
	 x_(0) = 0.0001;
	 x_(1) = 0.0001;
       }
    }

  // Initialize weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    for (int i = 1; i < weights_.size(); i++) {  
      weights_(i) = 0.5/( n_aug_ + lambda_ );
    }
 
    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }
  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_)/1000000.0;//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  // Predict before update
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    UpdateLidar(meas_package);
  }
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Estimate the object's location. Modify the state vector, x_. 
  Predict sigma points, the state, and the state covariance matrix.
  */
  // Create mean predicted measurement
  
  double delta_t2 = delta_t*delta_t;

  // Create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // Create augmented mean covarience matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0)  = x_aug;
  //  for (int i = 0; i< n_aug_; i++){
  //    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
  //    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  //  }
  double sqrt_lambda_n_aug = sqrt(lambda_+n_aug_); // Save some computations
  VectorXd sqrt_lambda_n_aug_L;
  for(int i = 0; i < n_aug_; i++) {
    sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
    Xsig_aug.col(i+1)        = x_aug + sqrt_lambda_n_aug_L;
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug_L;
  }
  
  //predict sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; i++) {
    
  //extract values for better readability
      double p_x = Xsig_aug(0,i);
      double p_y = Xsig_aug(1,i);
      double v = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawd = Xsig_aug(4,i);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

      //predicted state values
      double px_p, py_p;

      //avoid division by zero
      if (fabs(yawd) > 0.0001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
      }
      else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;
      
      //add noise
      px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a*delta_t;

      yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd*delta_t;

      //write predicted sigma point into right column
      Xsig_pred_(0,i) = px_p;
      Xsig_pred_(1,i) = py_p;
      Xsig_pred_(2,i) = v_p;
      Xsig_pred_(3,i) = yaw_p;
      Xsig_pred_(4,i) = yawd_p;

      //predicted state mean
      x_.fill(0.0);
      x_ = Xsig_pred_ * weights_; // vectorised sum
      
      //for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    //x_ = x_+ weights_(i) * Xsig_pred_.col(i);
      //}

      //predicted state covariance matrix
      P_.fill(0.0);
      for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

	// state difference
	VectorXd x_diff = Xsig_pred_.col(i) - x_;
	//angle normalization
	while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
	while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

	P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
      }
  }
}

/**
 *  Angle normalization to [-Pi, Pi]
 */
void UKF::NormAng(double *ang) {
  while (*ang > M_PI) *ang -= 2. * M_PI;
  while (*ang < -M_PI) *ang += 2. * M_PI;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Use lidar data to update the belief about the object's position. 
  */
  // Set measurement dimension (Lidar)
  // Create matrix for sigma points in measurement space
  // Transform sigma points into measurement space
  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);

  UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Use radar data to update with the universal update function
  */
  // Set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    // Measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);          //r
    Zsig(1,i) = atan2(p_y,p_x);                   //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   //r_dot
  }
  UpdateUKF(meas_package, Zsig, n_z);
  
}

// Universal update function
void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z){
  /**
  Update the belief about the object's position. 
  Modify the state vector, x_, and covariance, P_.
   */
  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred  = Zsig * weights_;
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Angle normalization
    NormAng(&(z_diff(1)));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
    R = R_radar_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
    R = R_lidar_;
  }
  S = S + R;

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // Calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
      // Angle normalization
      NormAng(&(z_diff(1)));
    }
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle normalization
    NormAng(&(x_diff(3)));
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // Measurements
  VectorXd z = meas_package.raw_measurements_;

  // Calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // Residual
  VectorXd z_diff = z - z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
    // Angle normalization
    NormAng(&(z_diff(1)));
  }
  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  // Calculate NIS for Radar and Lidar
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
    NIS_radar_ = z.transpose() * S.inverse() * z;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
    NIS_laser_ = z.transpose() * S.inverse() * z;
  }
}
