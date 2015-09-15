#include "RegionDetector.h"
#include "structures/Blob.h"

RegionDetector::RegionDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
  dx = 4;
  dy = 2;
  width = iparams_.width;
  height = iparams_.height;
}

vector<vector<Run*>> RegionDetector::findRegions(unsigned char* segImg) {
  vector<vector<Run*>> rows;
  for (int y=0; y < height; y+=dy) {
    rows.push_back(findRuns(segImg, y));
  }
  mergingPass(rows);
  return rows;
}

vector<Run*> RegionDetector::findRuns(unsigned char *segImg, int y) {
  vector<Run*> runs;
  int start = 0;
  char runColor = segImg[width * y];
  char color;
  for (int x=0; x <= width; x+=dx) { 
    if (x < width) 
      color = segImg[width * y + x];
    else
      color = c_UNDEFINED;
    if (runColor != color){
      int end = x - 1;
      if (runColor != c_UNDEFINED){
        Run* run = new Run(start, end, y, runColor);
        runs.push_back(run);
      }
      start = x;
      runColor = color;
    }
  }
  return runs;
}

void RegionDetector::mergingPass(vector<vector<Run*>> &rows) {
  for (int y=dy; y<height;y+=dy) {
    vector<Run*> firstRow = rows[y/dy - 1];
    vector<Run*> secondRow = rows[y/dy];
    int i=0;
    int j=0;
    while (i < firstRow.size() and j < secondRow.size()) {
      Run* a = firstRow[i];
      Run* b = secondRow[j];
      if (a->color == b->color &&
          a->start <= b->end &&
          a->end >= b->start)
      {
        merge(a, b);
      }
      if (a->end < b->end) {
        i++;
      } else {
        j++;
      }
    } 
  } 
}
Run* RegionDetector::find(Run* run) {
  if (run != run->parent) {
    run->parent = find(run->parent);
  }
  return run->parent;
}

void RegionDetector::merge(Run* a, Run* b) {
  a = find(a);
  b = find(b);
  if (a == b) { // Both already in same blob
    return;
  }
  if (a->rank < b->rank) {
    a->parent = b;
    mergeStats(b, a);
  } else if (a->rank > b->rank) {
    b->parent = a;
    mergeStats(a, b); 
  } else {
    b->parent = a;
    a->rank++;
    mergeStats(a, b);
  }
}

void RegionDetector::mergeStats(Run *root, Run *child)
{
  root->xi = min(root->xi, child->xi);
  root->xf = max(root->xf, child->xf);
  root->yi = min(root->yi, child->yi);
  root->yf = max(root->yf, child->yf);
  root->color_ct += child->color_ct;
}
