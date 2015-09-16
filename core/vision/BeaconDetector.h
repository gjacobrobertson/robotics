#pragma once

#include <vision/ObjectDetector.h>
#include <vision/RegionDetector.h>

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacons(vector<vector<Run*>> &regions);
 private:
  TextLogger* textlogger;
  void checkBeacon(Run* run, vector<vector<Run*>> &regions);
  void updateWorldObject(Run*, Run*); 
  bool checkRatio(Run*); 
  bool checkColor(Run*);
  Run* findRegion(vector<vector<Run*>> &regions, int x, int y);
  Run* findRegionAbove(vector<vector<Run*>> &regions, Run *run);
};
