#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include "structures/Blob.h"

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(vector<Blob*> &blobs) {
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

/*  for(auto beacon : beacons) {
    auto& object = vblocks_.world_object->objects_[beacon.first];
    auto box = beacon.second;
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
*/

// Get candidate beacon parts
// This should not be too strict
  vector<Blob*> p_candidates;
  vector<Blob*> y_candidates;
  vector<Blob*> b_candidates;
  for (int i=0; i< blobs.size(); i++) {

    if (blobs[i]->dx < blobs[i]->dy * 3 && blobs[i]->dy < blobs[i]->dx * 3){
      if (blobs[i]->color == c_PINK)
        p_candidates.push_back(blobs[i]);
      if (blobs[i]->color == c_YELLOW)
        y_candidates.push_back(blobs[i]);
      if (blobs[i]->color == c_BLUE)
        b_candidates.push_back(blobs[i]);
    }
  }

  // Yellow and blue
  for (int i=0; i<y_candidates.size(); i++){
    for (int j=0; j<b_candidates.size(); j++) {
      // Do they overlap?
      if (y_candidates[i]->xf > b_candidates[j]->xi && y_candidates[i]->xi < b_candidates[j]->xf){
        // Ok, are they close vertically?
        if (abs(y_candidates[i]->yi - b_candidates[j]->yf) < 10 || abs(y_candidates[i]->yf - b_candidates[j]->yi) < 10) {
          if (y_candidates[i]->yi + y_candidates[i]->dy / 2 > b_candidates[j]->yi + b_candidates[j]->dy/2) { // Blue over yellow
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW];
            beacon->imageCenterX = y_candidates[i]->xi + y_candidates[i]->dy / 2;
            beacon->imageCenterY = (b_candidates[j]->yi + y_candidates[i]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 300);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
        //    b_candidates[j]->print();
      //      y_candidates[i]->print();
    //        cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << y_candidates[i]->yf - b_candidates[j]->yi << endl;
          }
          else { // Yellow over blue
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE];
            beacon->imageCenterX = y_candidates[i]->xi + y_candidates[i]->dy / 2;
            beacon->imageCenterY = (y_candidates[i]->yi + b_candidates[j]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 300);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
  //          y_candidates[i]->print();
//            b_candidates[j]->print();
            //cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << b_candidates[j]->yf - y_candidates[i]->yi << endl;
          }          
        }
      }
    }
  }

  // pink and blue
  for (int i=0; i<p_candidates.size(); i++){
    for (int j=0; j<b_candidates.size(); j++) {
      // Do they overlap?
      if (p_candidates[i]->xf > b_candidates[j]->xi && p_candidates[i]->xi < b_candidates[j]->xf){
        // Ok, are they close vertically?
        if (abs(p_candidates[i]->yi - b_candidates[j]->yf) < 10 || abs(p_candidates[i]->yf - b_candidates[j]->yi) < 10) {
          if (p_candidates[i]->yi + p_candidates[i]->dy / 2 > b_candidates[j]->yi + b_candidates[j]->dy/2) { // Blue over pinkg
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK];
            beacon->imageCenterX = p_candidates[i]->xi + p_candidates[i]->dy / 2;
            beacon->imageCenterY = (b_candidates[j]->yi + p_candidates[i]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 200);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
          //  b_candidates[j]->print();
        //    p_candidates[i]->print();
      //      cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << p_candidates[i]->yf - b_candidates[j]->yi << endl;
          }
          else { // pink over blue
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE];
            beacon->imageCenterX = p_candidates[i]->xi + p_candidates[i]->dy / 2;
            beacon->imageCenterY = (p_candidates[i]->yi + b_candidates[j]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 200);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
    //        p_candidates[i]->print();
  //          b_candidates[j]->print();
//            cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << b_candidates[j]->yf - p_candidates[i]->yi << endl;
          }
        }
      }
    }
  }

  // yellow and pink
  for (int i=0; i<y_candidates.size(); i++){
    for (int j=0; j<p_candidates.size(); j++) {
      // Do they overlap?
      if (y_candidates[i]->xf > p_candidates[j]->xi && y_candidates[i]->xi < p_candidates[j]->xf){
        // Ok, are they close vertically?
        if (abs(y_candidates[i]->yi - p_candidates[j]->yf) < 10 || abs(y_candidates[i]->yf - p_candidates[j]->yi) < 10) {
          if (y_candidates[i]->yi + y_candidates[i]->dy / 2 > p_candidates[j]->yi + p_candidates[j]->dy/2) { // pink over yellow
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW];
            beacon->imageCenterX = y_candidates[i]->xi + y_candidates[i]->dy / 2;
            beacon->imageCenterY = (p_candidates[j]->yi + y_candidates[i]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 200);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
        //    p_candidates[j]->print();
      //      y_candidates[i]->print();
      //      cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << y_candidates[i]->yf - p_candidates[j]->yi << endl;
          }
          else { // Yellow over pink
            WorldObject* beacon = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK];
            beacon->imageCenterX = y_candidates[i]->xi + y_candidates[i]->dy / 2;
            beacon->imageCenterY = (y_candidates[i]->yi + p_candidates[j]->yf) / 2;
            beacon->seen = true;
            beacon->fromTopCamera = camera_ == Camera::TOP;
            auto position = cmatrix_.getWorldPosition(beacon->imageCenterX, beacon->imageCenterY, 200);
            beacon->visionDistance = cmatrix_.groundDistance(position);
            beacon->visionBearing = cmatrix_.bearing(position);
    //        y_candidates[i]->print();
  //          p_candidates[j]->print();
//            cout << "Beacon at: " << beacon->imageCenterX << ", " << beacon->imageCenterY << " " << p_candidates[j]->yf - y_candidates[i]->yi << endl;
          }
        }
      }
    }
  }

}
