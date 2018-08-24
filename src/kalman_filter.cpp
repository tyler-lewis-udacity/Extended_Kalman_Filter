#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_ ;//(...Lesson 5, Section 8)
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;// (...Lesson 5, Section 7)
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);

   // Cartesian to polar   
   float rho = sqrt(px*px + py*py);
   float phi = atan2(py, px);
   float rho_dot;
   
   // If rho is very near to zero, set rho_dot to zero
   float epsilon = 0.0001;
   if (fabs(rho) < epsilon){
     rho_dot = 0;
   }
   else {
     rho_dot = (px*vx + py*vy) / rho;
   }

   VectorXd z_pred = VectorXd(3);
   z_pred << rho, phi, rho_dot;
   VectorXd y = z - z_pred;

  // Ensure that angle is between -pi and pi (...see project Tips and Tricks)
  /* A big thanks to: https://github.com/mdalai/self-driving-car-Extended-Kalman-Filter
  ...I was stuck on this error until I read this student's notes. */
  float pi = 3.141;
  if (y(1) < -pi) {
    y(1) = y(1) + 2*pi;
  }
  else if (y(1) > pi) {
    y(1) = y(1) - 2*pi;
  }

   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K = PHt * Si;
 
   //new estimate
   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_) * P_;
}
