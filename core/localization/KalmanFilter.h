#pragma once
#include <Eigen/Eigen>


template<int Dimension, int M_Dim>
class KalmanFilter {
  
  protected:

    Eigen::Matrix<float, Dimension,1> state;
    Eigen::Matrix<float, Dimension,Dimension> covariance;
    Eigen::Matrix<float, Dimension,Dimension> A;
    Eigen::Matrix<float, Dimension,Dimension> R;
    Eigen::Matrix<float, M_Dim, Dimension> C;
    Eigen::Matrix<float, M_Dim, M_Dim> Q;    

  public:

    void virtual initialize() = 0;
    void virtual reinitialize(Eigen::Matrix<float,M_Dim,1> observation) = 0;
    void update(Eigen::Matrix<float,M_Dim,1> observation);
    Eigen::Matrix<float, Dimension, 1> getState() {return state;}
};
