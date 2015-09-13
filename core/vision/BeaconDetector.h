#pragma once

#include <vision/ObjectDetector.h>
#include "structures/Blob.h"

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacons(vector<Blob*> &blobs);
 private:
  TextLogger* textlogger;
};
