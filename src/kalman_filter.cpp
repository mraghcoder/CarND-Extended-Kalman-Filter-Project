#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}
/*
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
*/
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
  return;
}

void KalmanFilter::Update(const VectorXd &z) {
  // update the state by using Kalman Filter equations

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
  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // update the state by using Extended Kalman Filter equations

  VectorXd h_x(3);

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float c1 = pow((px*px + py*py),0.5);
  float c2 = px*vx + py*vy;
  h_x[0] = c1;
  if(fabs(c1) < 0.0001 ){
		cout << "h(x') Calc - Error - Division by Zero" << endl;
    return;
    //c1 = (c1 > 0 ? 0.0001 : -0.0001);
	}
  if(fabs(px) < 0.0001){
    px = (px > 0 ? 0.0001 : -0.0001);
		//cout << "h(x') Calc - Error - Division by Zero" << endl;
    //return;
	}


  h_x[1] = atan2(py,px);
  h_x[2] = c2/c1;

  // Update
  VectorXd y = z - h_x;
	MatrixXd Ht = H_.transpose();
  MatrixXd S = H_* P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt* Si;

  //new estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  return;
}
