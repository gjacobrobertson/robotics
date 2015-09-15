#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <vision/RegionDetector.h>
#include "structures/Blob.h"
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
  region_detector_ = new RegionDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  region_detector_->init(tl);
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
  vector<vector<Run*>> regions = region_detector_->findRegions(getSegImg());
  auto blobs = extractBlobs(regions);
  detectBall(blobs);
  detectGoal(blobs);
  for (auto it=blobs.begin();it!=blobs.end();it++)
  {
    for (auto blob=it->second.begin();blob!=it->second.end();blob++)
    {
      delete *blob;
    }
  }
  for (auto row=regions.begin();row!=regions.end();row++)
  {
    for (auto run=row->begin();run!=row->end();run++)
    {
      delete *run;
    }
  }
  //beacon_detector_->findBeacons(regions);
}

void ImageProcessor::detectBall(map<char, vector<Blob*>> &blob_map) {
  int imageX=0, imageY=0;
  float aspectRatio;
  bool aspectRatioMatch, pixelRatioMatch;
  int minSize = 70;
  float targetPixelRatio = 3.14 / 4.0;
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  ball->seen = false;
  vector<Blob*> blobs = blob_map[c_ORANGE];
  for (auto it=blobs.begin(); it!=blobs.end(); it++) {
    Blob *blob = *it;
    if (blob->dx * blob->dy < minSize) return;
    aspectRatio = blob->dx / (1.0 * blob->dy);
    aspectRatioMatch = aspectRatio >= 0.8 && aspectRatio <= 1.25;
    pixelRatioMatch = blob->correctPixelRatio >= targetPixelRatio * 0.8 && blob->correctPixelRatio <= targetPixelRatio * 1.25;
    if (aspectRatioMatch && pixelRatioMatch)
    {
      imageX = blob->xi + (blob->dx / 2);
      imageY = blob->yi + (blob->dy / 2);
      ball->seen = true;
      ball->frameLastSeen = vblocks_.frame_info->frame_id;
      ball->imageCenterX = imageX;
      ball->imageCenterY = imageY;
      ball->radius = (blob->dx + blob->dy) / 4;
      ball->fromTopCamera = camera_ == Camera::TOP;
      Position p = cmatrix_.getWorldPosition(imageX, imageY, blob->dy);
      ball->visionBearing = cmatrix_.bearing(p);
      ball->visionElevation = cmatrix_.elevation(p);
      ball->visionDistance = cmatrix_.groundDistance(p);
      return;
    }  
  }
}

void ImageProcessor::detectGoal(map<char, vector<Blob*>> &blob_map) {
  int imageX=0, imageY=0;
  int minSize = 1500;
  vector<Blob*> blobs = blob_map[c_BLUE];
  WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
  goal->seen = false;
  if (blobs.empty()) return;
  Blob *blob = blobs[0];
  if (blob->dx * blob->dy > minSize)
  {
    imageX = blob->xi + (blob->dx / 2);
    imageY = blob->yi + (blob->dy / 2);
    goal->seen = true;
    goal->imageCenterX = imageX;
    goal->imageCenterY = imageY;
    goal->fromTopCamera = camera_ == Camera::TOP;
    Position p = cmatrix_.getWorldPosition(imageX, imageY);
    goal->visionBearing = cmatrix_.bearing(p);
    goal->visionElevation = cmatrix_.elevation(p); 
    goal->visionDistance = cmatrix_.groundDistance(p);
    return;
  }
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

map<char, vector<Blob*>> ImageProcessor::extractBlobs(vector<vector<Run*>> &regions)
{
  map<char, vector<Blob*>> blobs;
  for (int i=0; i<regions.size(); i++){
    for (int j=0; j<regions[i].size(); j++) {
      Run* run = regions[i][j];
      if (run-> parent == run){
        Blob *blob = new Blob();
        blob->xi = run->xi;
        blob->xf = run->xf;
        blob->yi = run->yi;
        blob->yf = run->yf;
        blob->dx = (blob->xf - blob->xi) + 1;
        blob->dy = (blob->yf - blob->yi) + 1;
        blob->correctPixelRatio = 2.0 * run->color_ct / (blob->dx * blob->dy);
        
        if (blobs.count(run->color) == 0)
        {
          vector<Blob*> list;
          blobs[run->color] = list;
        }
        blobs[run->color].push_back(blob); 
      }
    }
  }
  for (auto it=blobs.begin(); it!=blobs.end(); it++)
  {
    sort(it->second.begin(), it->second.end(), sortBlobAreaPredicate);
  }
  return blobs;
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
