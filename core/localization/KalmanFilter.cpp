#include<localization/BallFilter.h>

template<int Dimension, int M_Dim>
void KalmanFilter<Dimension, M_Dim>::update(Eigen::Matrix<float,M_Dim,1> observation) {

  // Predictive step
  Eigen::Matrix<float, Dimension , 1> mu_ = A * state;
  Eigen::Matrix<float, Dimension, 1> sigma_ = A * covariance * Eigen::Matrix<float, Dimension, Dimension>::transpose(A) + R;

  // Compute Kalman gain
  Eigen::Matrix<float, Dimension, M_Dim> gain = C * covariance * Eigen::Matrix<float,M_Dim,Dimension>::transpose(C) + Q;
  gain = gain.inverse(); 
  gain = sigma_ * Eigen::Matrix<float,M_Dim, Dimension>::transpose(C) * gain;
  
  // Corrective step
  state = mu_ + gain * (observation - C * mu_);
  covariance = (Eigen::Matrix<float,Dimension,Dimension>::Identity() - gain * C) * sigma_;  
}
