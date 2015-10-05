#pragma once
#include <Eigen/Eigen>

using namespace Eigen;

template<int Dimension, int M_Dim>
class KalmanFilter {
  
  private:

    Eigen::Matrix<float, Dimension,1> state;
    Eigen::Matrix<float, Dimension,Dimension> covariance;
    Eigen::Matrix<float, Dimension,Dimension> A;

  public:

    virtual void update(Eigen::Matrix<float,M_Dim,1> observation) = 0;
    Eigen::Matrix<float, Dimension, 1> getState() {return state;}
};
