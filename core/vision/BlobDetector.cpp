#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BlobDetector::findBlobs() {
  if(camera_ == Camera::BOTTOM) return;
  auto rows = findRuns();
}

vector<vector<DisjointSet::Node<BlobDetector::Run>>> BlobDetector::findRuns() {
  vector<vector<DisjointSet::Node<BlobDetector::Run>>> rows;
  unsigned char* segImg = vblocks_.image->getImgTop();
  for (int y=0; y < iparams_.height; y++) {
    rows.push_back(findRunsInRow(segImg, y));
  }
  return rows;
}

vector<DisjointSet::Node<BlobDetector::Run>> BlobDetector::findRunsInRow(unsigned char *segImg, int y) {
  vector<DisjointSet::Node<BlobDetector::Run>> runs;
  int start = 0;
  char runColor = segImg[iparams_.width * y];
  for (int x=0; x <= iparams_.width; x++) { 
    char color = segImg[iparams_.width * y + x];
    if (runColor != color or x == iparams_.width) {
      int end = x - 1;
      BlobDetector::Run run(start, end, runColor);
      DisjointSet::Node<BlobDetector::Run> node(run);
      runs.push_back(node);
      start = x;
      runColor = color;
    }
  }
  return runs;
}
