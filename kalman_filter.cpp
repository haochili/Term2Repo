#include "kalman_filter.h"
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_Rin, MatrixXd &R_Lin, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_R = R_Rin;
  R_L = R_Lin;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y;
  MatrixXd S;
  MatrixXd K;
  long x_size =x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  y = z - H_ * x_;
  S = R_L + H_* P_ * H_.transpose();
  K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd y;
  MatrixXd S;
  MatrixXd K;
  long x_size =x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //y = z - Hj_ * x_;
  float rho = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  if (abs(x_(0)) <=0.0001)
  {
    x_(0) = 0.0001;
  }
  float theta = atan2(x_(1),x_(0));
  float rho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;
  x_polar = VectorXd(3);
  x_polar << rho, theta, rho_dot;
  y = z - x_polar;
  y(1) = atan2(sin(y(1)),cos(y(1)));
  S = R_R + Hj_* P_ * Hj_.transpose();
  K = P_ * Hj_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I - K * Hj_) * P_;
}
