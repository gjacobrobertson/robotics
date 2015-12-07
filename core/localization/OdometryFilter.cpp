#include <localization/OdometryFilter.h>
#include <memory/RobotVisionBlock.h>
#include <memory/OdometryBlock.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

OdometryFilter::OdometryFilter(MemoryCache& cache, TextLogger*& tl)
  : cache_(cache), textlogger(tl) {
}

void OdometryFilter::processFrame() {
  const auto& disp = cache_.odometry->displacement;
  locLog(41, "Odometry: %f, %f @ %f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  //cv::Mat E, R, t, mask;
  //double focal = 1.0;
  //cv::Point2d pp(0.0, 0.0);

  //std::vector<cv::Point2f> corners = cache_.robot_vision->corners;
  //std::vector<cv::Point2f> prevCorners = cache_.robot_vision->prevCorners;
  //if(corners.size() < 5) return;

  //E = cv::findEssentialMat(prevCorners, corners, focal, pp, RANSAC, 0.999, 1.0, mask);
  //cv::recoverPose(E, prevCorners, corners, R, t, focal, pp, mask);
  //locLog(41, "Rotation: %s", R);
}
