#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include "structures/Blob.h"

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(vector<vector<Run*>> &regions) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 }
  };
  static map<WorldObjectType,vector<int>> beacons = {
    { WO_BEACON_YELLOW_BLUE, { 24, 15, 74, 83} },
    { WO_BEACON_YELLOW_PINK, { 104, 41, 138, 96 } },
    { WO_BEACON_PINK_YELLOW, { 187, 38, 212, 90 } },
    { WO_BEACON_BLUE_PINK, { 246, 36, 268, 86 } }
  };
}
