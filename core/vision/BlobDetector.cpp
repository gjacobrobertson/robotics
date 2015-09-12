#include "BlobDetector.h"
#include "Node.h"

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BlobDetector::findBlobs() {
  if(camera_ == Camera::BOTTOM) return;
  auto rows = findRuns();
  mergeRuns(rows);
}

vector<vector<Node>> BlobDetector::findRuns() {
  vector<vector<Node>> rows;
  unsigned char* segImg = vblocks_.image->getImgTop();
  for (int y=0; y < iparams_.height; y++) {
    rows.push_back(findRunsInRow(segImg, y));
  }
  return rows;
}

vector<Node> BlobDetector::findRunsInRow(unsigned char *segImg, int y) {
  vector<Node> runs;
  int start = 0;
  char runColor = segImg[iparams_.width * y];
  for (int x=0; x <= iparams_.width; x++) { 
    char color = segImg[iparams_.width * y + x];
    if (runColor != color or x == iparams_.width) {
      int end = x - 1;
      Run *run = new Run(start, end, runColor);
      Node node(run);
      runs.push_back(node);
      start = x;
      runColor = color;
    }
  }
  return runs;
}

void BlobDetector::mergeRuns(vector<vector<Node>> rows) {
  for (int y=1; y<iparams_.height;y++) {
    vector<Node> firstRow = rows[y-1];
    vector<Node> secondRow = rows[y];
    int i=0;
    int j=0;
    while (i < firstRow.size() and j < secondRow.size()) {
      Node *a = &firstRow[i];
      Node *b = &secondRow[j];
      if (a->data->color == b->data->color) {
        a->merge(b);
      }
      if (a->data->end < b->data->end) {
        i++;
      } else {
        j++;
      }
    } 
  } 
}
