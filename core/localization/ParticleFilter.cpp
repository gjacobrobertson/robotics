#include <localization/ParticleFilter.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>
#include <math.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), num_particles(200), reseed_factor(0.98), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  particles().resize(num_particles);
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
  sampleMotion();
  resample();
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    mean_ = meanShiftPoseEstimate();
    dirty_ = false;
  }
  return mean_;
}

Pose2D ParticleFilter::meanPoseEstimate() const {
  Pose2D pose;
  using T = decltype(pose.translation);
  for(const auto& p : particles()) {
    pose.translation += T(p.x,p.y);
    pose.rotation += p.t;
  }
  if(particles().size() > 0)
    pose /= (particles().size());
  return pose;
}


Pose2D ParticleFilter::meanShiftPoseEstimate() const{
  Pose2D pose;
  using T = decltype(pose.translation);
  double max_density = 0;
  for(auto& p: particles()) {
    Eigen::Vector3d mean;
    mean << p.x, p.y, p.t;
    double density = meanShift(mean);
    if (density > max_density) {
      max_density = density;
      pose.translation = T(mean[0], mean[1]);
      pose.rotation = mean[2];    
    }
  }
  return pose;
}

double ParticleFilter::meanShift(Eigen::Vector3d &mean) const{
  Eigen::Matrix3d sigma;
  sigma << pow(100, 2), 0          , 0             ,
           0          , pow(100, 2), 0             ,
           0          , 0          , pow(M_PI/8, 2);

  double density;
  Eigen::Vector3d shift;
  int iter = 0;
  int dist_error = 50;
  do {
    density = 0;
    Eigen::Vector3d new_mean = Eigen::Vector3d::Zero();
    for (auto& p: particles()) {
      Eigen::Vector3d x;
      x << p.x, p.y, p.t;
      double w = mvgauss<3>(mean, sigma, x);
      new_mean += w * x;
      density += w;
    }
    new_mean /= density;
    shift = new_mean - mean;
    log(42, "Shifting (%f, %f, %f) to (%f, %f, %f)", mean[0], mean[1], mean[2], new_mean[0], new_mean[1], new_mean[2]);
    mean += shift;
    iter ++;
    if (iter > 100)
      break;
  } while (shift[0] > dist_error || shift[1] > dist_error || shift[2] > M_PI/16);
//  std::cout<<"Iteration: " << iter << std::endl;
  return density;
}
void ParticleFilter::sampleMotion() {
  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  //Sample and weight particles
  for(auto& p: particles()) {
      p.x = rand_.sampleN(p.x + disp.translation.x * cos(p.t), 10);
      p.y = rand_.sampleN(p.y + disp.translation.x * sin(p.t), 10);
      p.t = fmod( rand_.sampleN(p.t + disp.rotation, M_PI / 64) + M_PI , 2 * M_PI) - M_PI;
  }

  reseed();
}

void ParticleFilter::reseed() {
  for (int i=num_particles*reseed_factor; i < num_particles; i++) {
    particles()[i].x = rand_.sampleU(-2500,2500);
    particles()[i].y = rand_.sampleU(-1250,1250);
    particles()[i].t = rand_.sampleU(-M_PI,M_PI);
    particles()[i].w = 1.0;
  }
}

void ParticleFilter::resample() {
  weightParticles();
  vector<Particle> new_particles(num_particles);
  double seed = rand_.sampleU(1.0 / num_particles);
  for (int i=0; i < num_particles; i++) {
    new_particles[i] = sample(seed + i/(double)num_particles);
    new_particles[i].w = 1.0;
  }

  particles() = new_particles;
}

void ParticleFilter::weightParticles() {
  double sum_w = 0;
  for(auto& p: particles()) {
    p.w = get_weight(p);
    sum_w += p.w;
  }
  //Normalize weights
  for(auto& p: particles()) {
    p.w = p.w / sum_w;
    log(41, "Particle (%f, %f, %f) has normalized weight: %f", p.x, p.y, p.t, p.w);
  }
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
      log(42, "Weighting particle: (%f, %f, %f)", p.x, p.y, p.t);
      Point2D point(p.x, p.y);
      Eigen::Vector2d mu;
      mu << point.getDistanceTo(beacon.loc), 
            point.getBearingTo(beacon.loc, p.t);
      log(42, "Expected beacon: (%f, %f)", mu[0], mu[1]);

      Eigen::Matrix2d sigma;
      sigma << pow(200, 2), 0, 
               0 , pow(M_PI/32, 2);

      Eigen::Vector2d x;
      x << beacon.visionDistance, beacon.visionBearing;      
      log(42, "Observed beacon: (%f, %f)", x[0], x[1]);

      weight *= mvgauss<2>(mu, sigma, x);
      log(42, "Assigning weight: %f", weight);
    }
  }
  return weight;
}


template<int N>
double ParticleFilter::mvgauss(Eigen::Matrix<double, N, 1> mu, Eigen::Matrix<double, N, N> sigma, Eigen::Matrix<double, N, 1> x) const {
  return exp(-0.5 * (x - mu).transpose() * sigma.inverse() * (x - mu));
}
