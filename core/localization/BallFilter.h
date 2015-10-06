#pragma once

#include <localization/KalmanFilter.h>
#include <Eigen/Core>

class BallFilter : public KalmanFilter<4,2> {

  public:
    void initialize() override;
    void reinitialize(Eigen::Matrix<float,2,1> observation) override;
};
