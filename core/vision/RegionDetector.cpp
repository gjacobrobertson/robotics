#include "RegionDetector.h"
#include "structures/Blob.h"

Run* Run::find()
{
  if (parent != this) {
    parent = parent->find();
  }
  return parent;
}

void Run::print()
{
  cout << "Run(" << COLOR_NAME(color) << ")";
  cout << " Bounded by (" << xi << "," << yi << "," << xf << "," << yf << ")"; 
  cout << endl;
}
RegionDetector::RegionDetector(DETECTOR_DECLARE_ARGS, int hstep, int vstep) : DETECTOR_INITIALIZE {
  dx = hstep;
  dy = vstep;
  width = iparams_.width;
  height = iparams_.height;
  bottom_multiplier = 1;
}

vector<vector<Run*>> RegionDetector::findRegions(unsigned char* segImg) {

  if (camera_ == Camera::BOTTOM)
    bottom_multiplier = 1;
  else
    bottom_multiplier = 1;

  vector<vector<Run*>> rows;
  for (int y=0; y < height; y+=dy*bottom_multiplier) {
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
  for (int x=0; x <= width; x+=(dx*bottom_multiplier)) { 
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
  for (int y=(dy*bottom_multiplier); y<height;y+=(dy*bottom_multiplier)) {
    vector<Run*> firstRow = rows[y/(dy*bottom_multiplier) - 1];
    vector<Run*> secondRow = rows[y/(dy*bottom_multiplier)];
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
  return run->find();
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
