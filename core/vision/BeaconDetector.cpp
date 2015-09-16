#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/RegionDetector.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(vector<vector<Run*>> &regions) {
  if(camera_ == Camera::BOTTOM) return;
  for (auto row: regions)
  {
    for (auto run: row)
    {
      if (run->color == c_WHITE && run->parent == run)
      {
        checkBeacon(run, regions);
      }
    }
  }
}
void BeaconDetector::checkBeacon(Run *run, vector<vector<Run*>> &regions)
{
  if (!checkRatio(run)) return;
  Run* botRing = findRegionAbove(regions, run);
  if (botRing == NULL || !checkRatio(botRing) || !checkColor(botRing)) return;
  Run* topRing = findRegionAbove(regions, botRing);
  if (topRing == NULL || !checkRatio(topRing) || !checkColor(topRing)) return;
  Run* aboveBeacon = findRegionAbove(regions, topRing);
  if (aboveBeacon != NULL && checkColor(aboveBeacon)) return;
  updateWorldObject(botRing, topRing); 
}

void BeaconDetector::updateWorldObject(Run* botRing, Run* topRing)
{
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300},
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }
  };
  WorldObjectType beaconType;
  if (topRing->color == c_YELLOW && botRing->color == c_PINK)
    beaconType = WO_BEACON_YELLOW_PINK;
  else if (topRing->color == c_YELLOW && botRing->color == c_BLUE)
    beaconType = WO_BEACON_YELLOW_BLUE;
  else if (topRing->color == c_PINK && botRing->color == c_YELLOW)
    beaconType = WO_BEACON_PINK_YELLOW;
  else if (topRing->color == c_PINK && botRing->color == c_BLUE)
    beaconType = WO_BEACON_PINK_BLUE;
  else if (topRing->color == c_BLUE && botRing->color == c_YELLOW)
    beaconType = WO_BEACON_BLUE_YELLOW;
  else if (topRing->color == c_BLUE && botRing->color == c_PINK)
    beaconType = WO_BEACON_BLUE_PINK;

  WorldObject* beacon = &vblocks_.world_object->objects_[beaconType];
  int beaconLeft = min(botRing->xi, topRing->xi);
  int beaconRight = max(botRing->xf, topRing->xf);
  beacon->seen = true;
  beacon->frameLastSeen = vblocks_.frame_info->frame_id;
  beacon->imageCenterX = (beaconLeft + beaconRight) / 2;
  beacon->imageCenterY =  (topRing->yi + botRing->yf) / 2;
  beacon->fromTopCamera = true;
  Position p = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, heights[beaconType]);
  beacon->visionBearing = cmatrix_.bearing(p);
  beacon->visionElevation = cmatrix_.elevation(p);
  beacon->visionDistance = cmatrix_.groundDistance(p);
}

bool BeaconDetector::checkRatio(Run* run)
{
  int dx = run->xf - run->xi + 1;
  int dy = run->yf - run->yi + 1;
  float aspectRatio = 1.0 * dx / dy;
  return (aspectRatio >= 0.25 && aspectRatio <= 1.5);
}

bool BeaconDetector::checkColor(Run* run)
{
  return run->color == c_YELLOW || run->color == c_PINK || run->color == c_BLUE;
}

Run* BeaconDetector::findRegion(vector<vector<Run*>> &regions, int x, int y)
{
  if (y < 0 || y/2 >= regions.size()) return NULL;
  auto row = regions[y/2];
  for (auto run: row)
  {
    if (x >= run->start and x <= run->end){
      return run->find();
    }
  }
  return NULL;
}

Run* BeaconDetector::findRegionAbove(vector<vector<Run*>> &regions, Run* run)
{
  int dx = run->xf - run->xi + 1;
  int dy = run->yf - run->yi + 1;
  int xc = run->xi + (dx/2);
  int yc = run->yi + (dy/2); 
  return findRegion(regions, xc, yc - dy);
}
