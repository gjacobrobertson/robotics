#pragma once

#include "ObjectDetector.h"
#include "structures/Blob.h"

class TextLogger;

struct Run{
  int start, end;
  char color;
  int xi,xf,yi,yf;
  int color_ct;
  int rank;
  Run *parent;
  
  Run(int s, int e, int y, char c):
    start(s),
    end(e),
    color(c),
    color_ct(e - s + 1),
    xi(s),
    xf(e),
    yi(y),
    yf(y),
    rank(0),
    parent(this) {}
};

class RegionDetector : public ObjectDetector {
  public:
    RegionDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    std::vector<std::vector<Run*>> findRegions(unsigned char* segImg);
  protected:
    std::vector<Run*> findRuns(unsigned char *, int);
    void mergingPass(std::vector<std::vector<Run*>> &rows);
    Run *find(Run*);
    void merge(Run* a, Run* b);
    void mergeStats(Run* a, Run* b);
  private:
    TextLogger* textlogger;
    int dx,dy,width,height;
};
