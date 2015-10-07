#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>


template<int S_Dim, int M_Dim>
class KalmanFilter {
  public:
    typedef Eigen::Matrix<float, S_Dim, 1> StateVector;
    typedef Eigen::Matrix<float, S_Dim, S_Dim> ErrorCovMatrix;
    typedef Eigen::Matrix<float, M_Dim, 1> MeasurementVector;
    typedef Eigen::Matrix<float, S_Dim, S_Dim> ProcessMatrix;
    typedef Eigen::Matrix<float, S_Dim, S_Dim> ProcessCovMatrix;
    typedef Eigen::Matrix<float, M_Dim, S_Dim> MeasurementMatrix; 
    typedef Eigen::Matrix<float, M_Dim, M_Dim> MeasurementCovMatrix;
    typedef Eigen::Matrix<float, S_Dim, M_Dim> GainMatrix;

    StateVector x;
    ErrorCovMatrix P;
    ProcessMatrix A;
    ProcessCovMatrix Q;
    MeasurementMatrix H;
    MeasurementCovMatrix R;
    
    void predict() {
      x = A * x;
      P = A * P * A.transpose() + Q;
    }
   
    void correct(MeasurementVector z) {
      GainMatrix K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
      x = x + K * (z - H * x);
      P = (ErrorCovMatrix::Identity() - K * H) * P;
    }

    void print() {
      cout << "x: " << endl;
      cout << x << endl;
    
      cout << "P: " << endl;
      cout << P << endl;
    
      cout << "A: " << endl;
      cout << A << endl;
    
      cout << "Q: " << endl;
      cout << Q << endl;
    
      cout << "H: " << endl;
      cout << H << endl;
    
      cout << "R: " << endl;
      cout << R << endl;
    }
};
