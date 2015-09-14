#pragma once

#include "ObjectDetector.h"
#include "Node.h"
#include "structures/Blob.h"

class TextLogger;
//namespace DisjointSet {
class Node;
//}

struct Run{
  int start, end;
  char color;
  int xi,xf,yi,yf;
  int color_ct;
  Run(int s, int e, int y, char c):
    start(s),
    end(e),
    color(c),
    color_ct(e - s + 1),
    xi(s),
    xf(e),
    yi(i),
    yf(i) {}
};

class BlobDetector : public ObjectDetector {
  public:
//    class Run {
//      public:
//        int start, end;
//        char color;
//        Run(int start, int end, char color): start(start), end(end), color(color) { }
//    };
/*    class Blob {
      public:
        int left, top, right, bottom;
    };*/
    BlobDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    vector<Blob*> findBlobs(unsigned char* segImg);
  protected:
    std::vector<std::vector<Node*>> findRuns(unsigned char* segImg);
    std::vector<Node*> findRunsInRow(unsigned char *, int);
    void mergeRuns(std::vector<std::vector<Node*>> &rows);
  private:
    TextLogger* textlogger;
    int dx,dy,width,height;
};
