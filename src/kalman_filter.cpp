#define _USE_MATH_DEFINES
#include <cmath>

#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  P_ = MatrixXd(4,4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  F_ = MatrixXd(4,4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  H_ = MatrixXd(2,4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  Q_ =  MatrixXd(4,4);
}

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
  cout << "KalmanFilter::Predict" << endl;
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  cout << "KalmanFilter::Update" << endl;
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  cout << "KalmanFilter::UpdateEKF" << endl;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px_d = x_(0);
  float py_d = x_(1);
  float vx_d = x_(2);
  float vy_d = x_(3);

  float eps = 0.000001;
  if (fabs(px_d) < eps) {
    px_d = eps;
  }
  if (fabs(py_d)< eps) {
    py_d = eps;
  }

  float norm = sqrtf(powf(px_d,2) + powf(py_d,2));

  float rad = atan2f(py_d, px_d);
  cout << "rad: " << rad << endl;

  VectorXd hx = VectorXd(3);
  hx(0) = norm;
  hx(1) = rad;
  hx(2) = (px_d * vx_d + py_d * vy_d) / norm;

  // VectorXd y = z - z_pred;
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  cout << "Hj: " << Hj << endl;

  VectorXd y = z - hx;
  y[1] -= (2 * M_PI) * floor((y[1] + M_PI) / (2 * M_PI));

  MatrixXd Hjt = Hj.transpose();
	MatrixXd S = Hj * P_ * Hjt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Hjt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;

}
