#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/Classifier.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include "structures/Blob.h"

class BeaconDetector;
class RegionDetector;
struct Run;

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    Classifier* classifier_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(RobotCalibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    void detectBall(map<char, vector<Blob*>> &blobs);
    void detectGoal(map<char, vector<Blob*>> &blobs);
    bool findBall(int& imageX, int& imageY);
    bool findGoal(int& imageX, int& imageY);
  private:
    int getTeamColor();
    double getCurrentTime();
    map<char, vector<Blob*>> extractBlobs(vector<vector<Run*>> &regions);

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    RobotCalibration* calibration_;
    bool enableCalibration_;
    BeaconDetector* beacon_detector_;
    RegionDetector* region_detector_;

    int hstep, vstep;
};

#endif
