#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  // First Calculate hOfXPrime for use in y = z - hOfXPrime	
  double px = x_(0), py = x_(1), vx = x_(2), vy = x_(3);
  double c1 = px*px + py*py;
  double c2 = sqrt(c1);
  VectorXd hOfXPrime = VectorXd(3);
  hOfXPrime(0) = c2;
  hOfXPrime(1) = atan2(py, px);
  hOfXPrime(2) = ((px * vx) + (py * vy)) / c2;
  VectorXd y = z - hOfXPrime;
  // Adjust resulting y(1) angle value to to be within range -PI to PI
  if (y(1) > 3.14159) 
    y(1) = y(1) - (2.0 * 3.14159);
  if (y(1) < -3.14159) 
    y(1) = y(1) + (2.0 * 3.14159);

  // The remaining porting of UpdateEKF() is the same as Update() above
  // but are carried out with different size H_ and R_ matrices
  // set up from calling function.
  MatrixXd Ht = H_.transpose();  // H_ holds Jacobian Matrix for UpdateEKF()
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
