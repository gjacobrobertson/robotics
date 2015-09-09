#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_, *abs_parts = vblocks_.body_model->abs_parts_;
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  visionLog(30, "Process Frame camera %i", camera_);

  updateTransform();
  
  // Horizon calculation
  visionLog(30, "Calculating horizon line");
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog(30, "Classifying Image", camera_);
  if(!classifier_->classifyImage(color_table_)) return;
  detectBall();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {

  int imageX, imageY;
  if(!findBall(imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
  ball->frameLastSeen = vblocks_.frame_info->frame_id;
}

void ImageProcessor::detectGoal() {

  int imageX, imageY;
  if(!findGoal(imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;

}

bool ImageProcessor::findBall(int& imageX, int& imageY) {
  imageX = imageY = 0;
  unsigned char* segImg = getSegImg();
  float ct = 0;
  for (int x=0; x<iparams_.height; x++)
  {
    for (int y=0; y<iparams_.width; y++)
    {
      if (segImg[iparams_.width * y + x] == c_ORANGE) {
        imageX += x;
        imageY += y;
        ct += 1;
      }
    }
  }
  if (ct == 0)
    return false;
  imageX /= ct;
  imageY /= ct;
//  std::cout << "Pct: " << ct / (iparams_.width * iparams_.height) << std::endl;
  if (ct < 0.0003 * iparams_.width * iparams_.height)
    return false;
  return true;
}

bool ImageProcessor::findGoal(int& imageX, int& imageY) {
  imageX = imageY = 0;
  unsigned char* segImg = getSegImg();
  int ct = 0;
  for (int x=0; x<iparams_.height; x++)
  {
    for (int y=0; y<iparams_.width; y++)
    {
      if (segImg[iparams_.width * y + x] == c_BLUE) {
        imageX += x;
        imageY += y;
        ct += 1;
      }
    }
  }
  if (ct == 0)
    return false;
  imageX /= ct;
  imageY /= ct;
  return true;

}

int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
