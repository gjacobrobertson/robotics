#include<localization/BallFilter.h>

// function for declaring initial values of state, covariance, A, R, C, and Q
void BallFilter::reinitialize(Eigen::Matrix<float,2,1> observation) {
  
  state(0) = observation(0);
  state(1) = observation(1);
  state(2) = 0.0;
  state(3) = 0.0;

  covariance = Eigen::Matrix<float, 4, 4>::Identity();

}

void BallFilter::initialize() {

  reinitialize(Eigen::Matrix<float,2,1>::Zero());

  A = Eigen::Matrix<float, 4, 4>::Identity();
  A(0,2) = 1;
  A(1,3) = 1;
  
  R = Eigen::Matrix<float,4,4>::Identity();
  C = Eigen::Matrix<float,2,4>::Zero();
  C(0,0) = 1.0;
  C(1,1) = 1.0;

  Q = Eigen::Matrix<float,2,2>::Identity();
}
