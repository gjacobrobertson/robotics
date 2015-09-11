#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BlobDetector::findBlobs() {
  if(camera_ == Camera::BOTTOM) return;
  vector<vector<DisjointSet::Node<BlobDetector::Run>>> rows;
  findRuns(rows);
}

void BlobDetector::findRuns(vector<vector<DisjointSet::Node<BlobDetector::Run>>>& rows) {
  unsigned char* segImg = vblocks_.image->getImgTop();
  for (int y=0; y < iparams_.width; y++) {
    rows.push_back(findRunsInRow(segImg, y));
  }
}

vector<DisjointSet::Node<BlobDetector::Run>> BlobDetector::findRunsInRow(unsigned char *segImg, int y) {
  vector<DisjointSet::Node<BlobDetector::Run>> runs;
  Run run = new Run(0, 0);
  bool firstRun = true;
  for (int x=0; x < iparams_.width; x++) { 
    char color = segImg[iparams_.width * y + x];
    if (firstRun) {
      run = new Run(x, color);
      firstRun = false;
    } else if (run->color != color) {
      run->end = x - 1;
      runs.push_back(new DisjointSet::Node<BlobDetector::Run>(run));
      run = new Run(x, color);
    }
  }
  runs.push_back(new DisjointSet::Node<BlobDetector::Run>(run));
}
