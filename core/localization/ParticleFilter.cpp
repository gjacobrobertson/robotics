#include <localization/ParticleFilter.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>
#include <math.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  particles().resize(100);
  for(auto& p: particles()) {
    p.x = rand_.sampleN(loc.x, 250);
    p.y = rand_.sampleN(loc.y, 250);
    p.t = rand_.sampleN(orientation, M_PI / 4);
    p.w = 1;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  

  //Sample and weight particles
  double sum_w = 0;
  for(auto& p: particles()) {
    p.x = rand_.sampleN(p.x + disp.translation.x * cos(p.t), 10);
    p.y = rand_.sampleN(p.y + disp.translation.x * sin(p.t), 10);
    p.t = fmod( rand_.sampleN(p.t + disp.rotation, M_PI / 64) + M_PI , 2 * M_PI) - M_PI;
    p.w = get_weight(p);
    sum_w += p.w;
  }


  if (sum_w < 0.000001) { 
    for(auto& p: particles()) {
      p.x = rand_.sampleU(-2500, 2500);
      p.y = rand_.sampleU(-1250, 1250);
      p.t = rand_.sampleU(-M_PI, M_PI);
      p.w = 1;
    }
    return;
  }

  //Normalize weights
  for(auto& p: particles()) {
    p.w = p.w / sum_w;
  }

  //Resample
  int n = particles().size();
  vector<Particle> new_particles(n);
  double seed = rand_.sampleU(1.0 / n);
  for (int i=0; i < n; i++) {
    new_particles[i] = sample(seed + i/(double)n);
    new_particles[i].w = 1.0;
  }

  particles() = new_particles;
}

Particle ParticleFilter::sample(double seed)
{
  double cdf = 0;
  for (auto& p: particles()) {
    cdf += p.w;
    if(cdf >= seed) {
      return p;
    }
  }
  return particles().back();
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= particles().size();
    dirty_ = false;
  }
  return mean_;
}

double ParticleFilter::get_weight(Particle p) {
  double weight = 1;
  vector<WorldObjectType> beacon_types{WO_BEACON_BLUE_YELLOW,
                            WO_BEACON_YELLOW_BLUE,
                            WO_BEACON_BLUE_PINK,
                            WO_BEACON_PINK_BLUE,
                            WO_BEACON_PINK_YELLOW,
                            WO_BEACON_YELLOW_PINK};
  for(auto &beacon_type : beacon_types) {
    auto& beacon = cache_.world_object->objects_[beacon_type];
    if (beacon.seen) {
      log(41, "Weighting particle: (%f, %f, %f)", p.x, p.y, p.t);
      Point2D point(p.x, p.y);
      Eigen::Vector2d mu;
      mu << point.getDistanceTo(beacon.loc), 
            point.getBearingTo(beacon.loc, p.t);
      log(41, "Expected beacon: (%f, %f)", mu[0], mu[1]);

      Eigen::Matrix2d sigma;
      sigma << 250, 0, 
               0 , M_PI/16;

      Eigen::Vector2d x;
      x << beacon.visionDistance, beacon.visionBearing;      
      log(41, "Observed beacon: (%f, %f)", x[0], x[1]);

      weight *= mvnpdf<2>(mu, sigma, x);
      log(41, "Assigning weight: %f", weight);
    }
  }
  return weight;
}

template<int N>
double ParticleFilter::mvnpdf(Eigen::Matrix<double, N, 1> mu, Eigen::Matrix<double, N, N> sigma, Eigen::Matrix<double, N, 1> x) {
  return exp(-0.5 * (x - mu).transpose() * sigma.inverse() * (x - mu)) / (sqrt(pow(2 * M_PI, N)) * sigma.determinant());
}
