#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

map<Color, list<BlobDetector::Blob>> BlobDetector::findBlobs() {
  map<Color, list<BlobDetector::Blob>> blobs;
  if(camera_ == Camera::BOTTOM) return blobs;
  blobs[c_ORANGE] = findBlobs(c_ORANGE);
  return blobs;
}

list<BlobDetector::Blob> BlobDetector::findBlobs(Color c) {
  list<Blob> blobs;
  for (int x=0; x < iparams_.height; x++)
  {
    for (int y=0; y < iparams_.width; y++)
    {
    }
  }
  return blobs;
}
