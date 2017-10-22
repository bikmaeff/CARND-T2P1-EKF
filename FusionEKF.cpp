#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  numLidar_ = 0;
  numRadar_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.x_ = VectorXd(4);
  ekf_.Q_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 0,
	  0, 0, 0, 1000;

  //measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Covariance matrices are initialized in FusionEKF Class Constructor above.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
        Convert radar sensor measurement from polar to cartesian coordinates,
        and initialize state of ekf_.x_
       */
      float radarRho = measurement_pack.raw_measurements_[0];
      float radarPhi = measurement_pack.raw_measurements_[1];
      float radarRhoDot = measurement_pack.raw_measurements_[2];
      ekf_.x_(0) = radarRho * cos(radarPhi);
      ekf_.x_(1) = radarRho * sin(radarPhi);
      ekf_.x_(2) = radarRhoDot * cos(radarPhi);
      ekf_.x_(3) = radarRhoDot * sin(radarPhi);
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;  // done initializing, no need to predict or update
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
        Initialize state of ekf_.x_ from LIDAR sensor measurement
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;  // done initializing, no need to predict or update
    }

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix Q.
     * noise_ax = 9 and noise_ay = 9 values are used for setting up Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
	  0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
	  dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
	  0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    float px = ekf_.x_(0);
    float py = ekf_.x_(1);
    float vx = ekf_.x_(2);
    float vy = ekf_.x_(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px + py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if ((fabs(px) < 0.0001) && (fabs(py) < 0.0001)) {
      px = 0.0001; py = 0.0001;
    }
    if (fabs(c1) < 0.0000001) {
      c1 = 0.0000001;
    }      

    //compute the Jacobian matrix
    Hj_ << (px / c2), (py / c2), 0, 0,
	  -(py / c1), (px / c1), 0, 0,
	  py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

    ekf_.R_ = R_radar_;	
    ekf_.H_ = Hj_; // use the Hj matrix in updateEKF() for the H_ matrix
    //numRadar_++;
    //cout << endl << "R" << numRadar_ << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    //numLidar_++;
    //cout << endl << "L" << numLidar_ << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
