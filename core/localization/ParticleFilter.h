#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math/Pose2D.h>
#include <common/Random.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    Random rand_;
    float reseed_factor;
    int num_particles;

    mutable Pose2D mean_;
    mutable bool dirty_;

    Pose2D meanPoseEstimate() const;
    Pose2D meanShiftPoseEstimate() const;
    double meanShift(Eigen::Vector3d &mean) const;

    void sampleMotion();
    void reseed();
    void resample();
    void weightParticles();
    Particle sample(double seed);
    double get_weight(Particle p);

    template<int N>
    double mvgauss(Eigen::Matrix<double, N, 1> mu, Eigen::Matrix<double, N, N> sigma, Eigen::Matrix<double, N, 1> x) const;
};
