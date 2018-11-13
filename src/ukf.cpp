#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;//9 0.231 is the average acceleration of a bicycle

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 4;//45 degrees is 0.7
  
  // NONO
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  // END NONO
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // n state x
  n_x_ = 5;

  // n augmented 
  n_aug_ = 7;

  // Lambda (spreading factor)
  lambda_ = 3 - n_aug_;

  // Sigma Weights 
  weights_ = VectorXd(n_aug_ * 2 + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++){
    //set weights
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

   ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_ , n_aug_ * 2 + 1);

  is_initialized_ = false;

  time_us_ = 0.0;

  // NIS radar value 
  NIS_radar_ = 0.0;

  // NIS  laser value
  NIS_laser_ = 0.0;

  // Const matrcies and values

  // Radar  Noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << pow(std_radr_, 2), 0, 0,
  0, pow(std_radphi_, 2), 0,
  0, 0, pow(std_radrd_, 2);

  // Laser Noise covariance matrix
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << pow(std_laspx_, 2), 0,
  0, pow(std_laspy_, 2);

  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_(5, 5) = pow(std_a_, 2);
  P_aug_(6, 6) = pow(std_yawdd_, 2);

//   NIS_laser_vals.open("../NIS_data/NIS_laser_vals.txt", ios::out);
//   NIS_radar_vals.open("../NIS_data/NIS_radar_vals.txt", ios::out);
//
//   // Check for errors opening the files
//   if( !NIS_laser_vals.is_open() )
//   {
//     cout << "Error opening NIS_laser_vals.txt file" << endl;
//     exit(1);
//   }
//
//   if( !NIS_radar_vals.is_open() )
//   {
//     cout << "Error opening NIS_radar_vals.txt file" << endl;
//     exit(1);
//   }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) ||
    (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      if (!is_initialized_){

      // Init State vector and covariance matrix
      x_ << .1, .1, 0., 0., 0.0;
//       P_ = MatrixXd::Identity(n_x_, n_x_);
      P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;




      // init timestamp
      time_us_ = meas_package.timestamp_;

        if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {

          x_(0) =  meas_package.raw_measurements_(0);
          x_(1) = meas_package.raw_measurements_(1);

        }
        else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
          /**
          Convert radar from polar to cartesian coordinates and initialize state.
          */
          float rho = meas_package.raw_measurements_(0);
          float phi = meas_package.raw_measurements_(1);
          // float rho_dot = meas_package.raw_measurements_(2);
          x_(0) = rho * cos(phi);
          x_(1) = rho * sin(phi);
        }
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;

      }
      

      /*****************************************************************************
      *  Prediction
      ****************************************************************************/
      float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
      time_us_ = meas_package.timestamp_;

      Prediction(dt);

      /*****************************************************************************
      *  Update
      ****************************************************************************/
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
      } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
      } 

  }
}

MatrixXd UKF::GenerateSigmaPoint(){
  int n_sig = 2 * n_x_ + 1;
  //sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, n_sig);


  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  Xsig.col(0) = x_;
  //calculate sigma points ...
  //set sigma points as the remaining columns of matrix Xsig
  for (int i = 0; i < n_x_; i++){
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  return Xsig;
}
// GenerateSigmaPoint
MatrixXd UKF::GetAugmentedSigmaPoints(){
  // Init 
  // augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  lambda_ = 3 - n_aug_;
  // Augmanted sate vector
  // x_aug.head(n_x_) = x_;
  // x_aug(n_x_) = 0;
  // x_aug(n_x_ + 1 ) = 0;
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;


  // //Augmented covariance matrix
  // P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  // P_aug_(5, 5) = pow(std_a_, 2);
  // P_aug_(6, 6) = pow(std_yawdd_, 2);

  //Square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

//// PredictSigmaPoint
MatrixXd UKF::PredictSigmaPoints(double delta_t){
  // Get augmented sigma points
  MatrixXd Xsig_aug = GetAugmentedSigmaPoints();
  MatrixXd Xsig_pred = MatrixXd(n_x_ , n_aug_ * 2 + 1);;
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
      // Helpers
      const double p_x = Xsig_aug(0, i);
      const double p_y = Xsig_aug(1, i);
      const double v = Xsig_aug(2, i);
      const double yaw = Xsig_aug(3, i);
      const double yawd = Xsig_aug(4, i);
      const double nu_a = Xsig_aug(5, i);
      const double nu_yawdd = Xsig_aug(6, i);
      
      // State variables pred
      double px_p, py_p;
      // Division by zero check
      if (fabs(yawd) > EPS){
          // If greater than Zero
          px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
          py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
      } else {
          // If Zero
          px_p = p_x + v * delta_t * cos(yaw);
          py_p = p_y + v * delta_t * sin(yaw); 
      }
      
      // Continue initialization
      double v_p = v;
      double yaw_p = yaw + yawd * delta_t;
      double yawd_p = yawd;
      
      
      // Adding Noise 
      px_p += 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a;
      py_p += 0.5 * pow(delta_t, 2) * sin(yaw) * nu_a;
      v_p += delta_t * nu_a;
      yaw_p += 0.5 * pow(delta_t, 2) * nu_yawdd;
      yawd_p += delta_t * nu_yawdd;
      
      // updating Xsig_pred witht eh new values
      Xsig_pred(0, i) = px_p;
      Xsig_pred(1, i) = py_p;
      Xsig_pred(2, i) = v_p;
      Xsig_pred(3, i) = yaw_p;
      Xsig_pred(4, i) = yawd_p;
  }
  return Xsig_pred;
} 

// PredictMeanAndCovariance
void UKF::PredictMeanAndCovariance(){
  //create vector for predicted state
  VectorXd x_pred = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
      x_ += weights_(i) * Xsig_pred_.col(i);
  }
  // Predicted Covariance 
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
      VectorXd diff = Xsig_pred_.col(i) - x_;
      // Normalize the angle between -ve and +ve pi
      while (diff(3) > M_PI) diff(3) -=2. * M_PI;
      while (diff(3) < -M_PI) diff(3)+=2. * M_PI;
      P_ += weights_(i) * diff * diff.transpose();
  }
  // Set the state vector and the covarience matrix 
  // with the predicted values
  // x_ = x_pred;
  // P_ = P_pred;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // Sigma points
  // state vector x
  // State covariance matrix P
  
  // Predict Sigma Point 
  Xsig_pred_ = PredictSigmaPoints(delta_t);
  // Predict Mean and covariance which will set both the x_ state vector 
  // and the P_ covarience matrix 
  PredictMeanAndCovariance();

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // px and py of the Lidar sensor
  int n_z  = 2;
  // //
  const VectorXd z = meas_package.raw_measurements_;
  //
  MatrixXd S = MatrixXd(n_z, n_z);
  // Measurement space sigma points matrix
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
      double p_x = Xsig_pred_(0, i);
      double p_y = Xsig_pred_(1, i); 
      
      // measurement model 
      Zsig(0, i) = p_x;
      Zsig(1, i) = p_y;
  }
  //calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      // Residual 
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
    //   //angle normalization
    // while (z_diff(1) > M_PI) z_diff(1) -= 2. *M_PI;
    // while (z_diff(1) < -M_PI) z_diff(1) += 2. *M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
      
  }

  // Noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << pow(std_laspx_, 2), 0,
  0, pow(std_laspy_, 2);
  
  S = S + R;

  // Update
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
    // Residual 
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    
    // State Difference 
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    Tc += weights_(i) * x_diff * z_diff.transpose();
    
  }

    //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();
    //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ = P_ - K * S * K.transpose();

  

  // cout << "Laser NIS is : "<< NIS_laser_<< endl;
//   NIS_laser_vals << NIS_laser_ << endl;

  //##########################################################
  // EKF implementation

  // MatrixXd H_laser_ = MatrixXd(2, 2);
  // H_laser_ <<  1, 0, 0, 0, 0,
  //              0, 1, 0, 0, 0;

  // MatrixXd R_laser = MatrixXd(2, 2);
  //   //measurement covariance matrix - laser
  // R_laser << 0.0225, 0,
  //             0, 0.0225;
  // VectorXd z_pred = H_laser_ * x_;
  // VectorXd y = z - z_pred;
  // const MatrixXd Ht = H_laser_.transpose();
  // const MatrixXd S = H_laser_ * P_ * Ht + R_laser;
  // const MatrixXd Si = S.inverse();
  // const MatrixXd PHt = P_ * Ht;
  // const MatrixXd K = PHt * Si;
  
  // // // New estimate 
  // x_ = x_ + (K * y);

  // long x_size = x_.size();
  // MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // // 
  // P_ = (I - K * H_laser_) * P_;

  // // NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

}

void UKF::RadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd Zsig_out, int n_z){
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  // VectorXd sensorspace_measurements = VectorXd(n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
      double p_x = Xsig_pred_(0, i);
      double p_y = Xsig_pred_(1, i); 
      double v = Xsig_pred_(2, i);
      double yaw = Xsig_pred_(3, i);
      
      double v1 = cos(yaw) * v;
      double v2 = sin(yaw) * v;
      
      // measurement model 
      Zsig(0, i) = sqrt(pow(p_x, 2) + pow(p_y, 2));
      if (fabs(p_x + p_y) < EPS) {
        Zsig(1, i) = atan2(0.0001, 0.0001);
      } else {
        Zsig(1, i) = atan2(p_y, p_x);
      }
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / std::max(EPS, sqrt(p_x*p_x + p_y*p_y));
//      Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(pow(p_x, 2) + pow(p_y, 2));
      
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      // Residual 
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
      //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. *M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. *M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
      
  }
  
  S = S + R_laser_;

  *z_out = z_pred;
  *S_out = S;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // Number of radar mesurements
  int n_z = 3;

  VectorXd z = meas_package.raw_measurements_;
  // VectorXd z_pred = VectorXd(n_z);
  // MatrixXd S = MatrixXd(n_z, n_z);
  // MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);


  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  // VectorXd sensorspace_measurements = VectorXd(n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
      double p_x = Xsig_pred_(0, i);
      double p_y = Xsig_pred_(1, i); 
      double v = Xsig_pred_(2, i);
      double yaw = Xsig_pred_(3, i);
      
      double v1 = cos(yaw) * v;
      double v2 = sin(yaw) * v;
      
      // measurement model 
      Zsig(0, i) = sqrt(pow(p_x, 2) + pow(p_y, 2));
      Zsig(1, i) = atan2(p_y, p_x);
      Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(pow(p_x, 2) + pow(p_y, 2));
      
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      // Residual 
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
      //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. *M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. *M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
      
  }
  
  S = S + R_radar_;


  // RadarMeasurement(&z_pred, &S, Zsig, n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
    // Residual 
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. *M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. *M_PI;
    
    
    // State Difference 
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    
    Tc += weights_(i) * x_diff * z_diff.transpose();
    
  }
  //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();
    //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ = P_ - K * S * K.transpose();

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  // cout << "Radar NIS is : "<< NIS_radar_<< endl;
//   NIS_radar_vals << NIS_radar_ << endl;

}
