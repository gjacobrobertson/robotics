#pragma once
#include <vector>

#include <memory/TextLogger.h>
#include <opencv2/core/core.hpp>
#include <vision/VisionBlocks.h>

#define FEATURE_MAX_CORNERS 100
#define FEATURE_QUALITY_LEVEL 0.3
#define FEATURE_MIN_DISTANCE 3
#define FEATURE_PARAMS FEATURE_MAX_CORNERS, FEATURE_QUALITY_LEVEL, FEATURE_MIN_DISTANCE

class OpticalFlowEstimator {
  public:
    OpticalFlowEstimator(const VisionBlocks& vblocks, const ImageParams& iparams, const Camera::Type& camera);
    ~OpticalFlowEstimator();
    void init(TextLogger* tl){textlogger = tl;};

    vector<cv::Point2f> getPrevCorners() { return prevCorners_; }
    vector<cv::Point2f> getNextCorners() { return nextCorners_; }
    void detectCorners();
    void estimateFlow();

  private:
    const VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    const Camera::Type& camera_;
    TextLogger* textlogger;
    
    cv::Mat prevImg_;
    cv::Mat nextImg_;
    vector<cv::Point2f> prevCorners_;
    vector<cv::Point2f> nextCorners_;
    vector<uchar> status_;
    vector<float> error_;

    cv::Mat getImg();
    void filterCorners(vector<cv::Point2f>&);
};
