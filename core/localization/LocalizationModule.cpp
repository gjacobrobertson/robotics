#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <Eigen/Core>

#define SIGHT_THRESHOLD 20

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), seen_last_frame_(false) {
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
//  self.loc = cache_.localization_mem->player;
  cache_.localization_mem->player = self.loc;
  std::cout << "Init: " << self.loc.x << std::endl;
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  cache_.localization_mem->player = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
  initBallFilter();
  last_seen_ball.x = 0;
  last_seen_ball.y = 0;
  frames_since_last_seen_ = SIGHT_THRESHOLD + 1;
}

// Initialize the ball filter
void LocalizationModule::initBallFilter() {
  ball_filter_.x.setZero();

  BallFilter::StateVector p;
  p << 0.1, 0.1, 0.1, 0.1;
  ball_filter_.P = p.asDiagonal();

  ball_filter_.A.setIdentity();
  ball_filter_.A.topRightCorner<2,2>().setIdentity();

  BallFilter::StateVector q;
  q << 0.1, 0.1, 0.1, 0.1;
  ball_filter_.Q = q.asDiagonal();

  ball_filter_.H.setIdentity();

  BallFilter::MeasurementVector r;
  r << 5.0, 5.0, 0.5, 0.5; // 0.3 0.3 0.3 0.3 
  ball_filter_.R = r.asDiagonal();
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player;
  self.loc = sloc;
     
  if(ball.seen) {

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);
    auto belief = ball_filter_.x;

    if (ball.visionDistance > 5000) {
      seen_last_frame_ = false;
      frames_since_last_seen_ ++;
      ball_filter_.predict();
    } else {
      

      BallFilter::MeasurementVector observation;
      observation(0) = relBall.x;
      observation(1) = relBall.y;

/*      if (seen_last_frame_)
      {
        observation(2) = (relBall.x - belief(0)) * 1.0 + belief(2) * 0.0;
        observation(3) = (relBall.y - belief(1)) * 1.0 + belief(3) * 0.0;
      }
      else
      {
        observation(2) = belief(2);
        observation(3) = belief(3); 
      }
*/

      if (frames_since_last_seen_ < SIGHT_THRESHOLD)
      {
        observation(2) = (relBall.x - last_seen_ball.x) / (frames_since_last_seen_ + 1);
        observation(3) = (relBall.y - last_seen_ball.y) / (frames_since_last_seen_ + 1);
      } else {
        observation(2) = 0;
        observation(3) = 0;
      }
      
      frames_since_last_seen_ = 0;    
      seen_last_frame_ = true;
      last_seen_ball.x = relBall.x;
      last_seen_ball.y = relBall.y;
  //   std::cout << "Observation: " << observation(0) << ", " << observation(1) << ", " << observation(2) << ", " << observation(3) << std::endl;
      ball_filter_.predict();
      ball_filter_.correct(observation);
    }
  } 
  else {
    frames_since_last_seen_ ++;
    seen_last_frame_ = false;
    ball_filter_.predict();
  }
  

  auto new_belief = ball_filter_.x;
//  std::cout << "New State: " << new_belief(0) << ", " << new_belief(1) << ", " << new_belief(2) << ", " << new_belief(3)<< std::endl;
  auto relBall = Point2D(new_belief(0), new_belief(1));
  auto relVel = Point2D(new_belief(2), new_belief(3));
//  std::cout << self.loc.x << ", " << self.loc.y << std::endl;
  auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);
  auto globalVel = relVel;
  globalVel.rotate(self.orientation);

  // Update the ball in the WorldObject block so that it can be accessed in python
  ball.loc = globalBall;
  ball.relPos = relBall;
  ball.distance = relBall.getMagnitude(); //ball.visionDistance;
  ball.bearing = relBall.getDirection(); //ball.visionBearing;
  ball.absVel = globalVel;
  ball.relVel = relVel;

  // Update the localization memory objects with localization calculations
  // so that they are drawn in the World window
  cache_.localization_mem->state[0] = ball.loc.x;
  cache_.localization_mem->state[1] = ball.loc.y;
  cache_.localization_mem->covariance = ball_filter_.P.topLeftCorner<2, 2>() * 10000;
}
