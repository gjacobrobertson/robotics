#include "OpticalFlowEstimator.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

OpticalFlowEstimator::OpticalFlowEstimator(const VisionBlocks& vblocks, const ImageParams& iparams, const Camera::Type& camera):
  vblocks_(vblocks),
  iparams_(iparams),
  camera_(camera)
{
  prevCorners_.reserve(FEATURE_MAX_CORNERS);
  nextCorners_.reserve(FEATURE_MAX_CORNERS);
  status_.reserve(FEATURE_MAX_CORNERS);
  error_.reserve(FEATURE_MAX_CORNERS);
}

OpticalFlowEstimator::~OpticalFlowEstimator() {
}

void OpticalFlowEstimator::detectCorners() {
  cv::goodFeaturesToTrack(prevImg_, prevCorners_, FEATURE_PARAMS);
  for (auto& corner : prevCorners_){
    visionLog(30, "Detected Corner %2.f, %2.f", corner.x, corner.y);
  }
}

void OpticalFlowEstimator::estimateFlow() {
  prevImg_ = nextImg_.clone();
  prevCorners_ = nextCorners_;
  if(prevCorners_.size() < 5) {
    detectCorners();
  }
  nextImg_ = getImg();
  
  if (prevImg_.empty()) return;
  if (prevCorners_.size() == 0) return;

  cv::calcOpticalFlowPyrLK(prevImg_, nextImg_, prevCorners_, nextCorners_, status_, error_);
  filterCorners(prevCorners_);
  filterCorners(nextCorners_);
}

cv::Mat OpticalFlowEstimator::getImg() {
  unsigned char* img_;
  if(camera_ == Camera::TOP)
  {
    img_ = vblocks_.image->getImgTop();
  }
  else {
    img_ = vblocks_.image->getImgBottom();
  }
  cv::Mat colorImage = color::rawToMat(img_, iparams_);
  cv::Mat grayImage;
  cvtColor(colorImage, grayImage, CV_BGR2GRAY);
  return grayImage;
}

void OpticalFlowEstimator::filterCorners(vector<cv::Point2f>& corners) {
  vector<cv::Point2f> filtered;
  for (vector<cv::Point2f>::size_type i = 0; i<corners.size();i++)
  {
    if(status_[i]) {
      filtered.push_back(corners[i]);
    }
  }
  corners = filtered;
}
