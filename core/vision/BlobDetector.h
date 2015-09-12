#pragma once

#include "ObjectDetector.h"
#include "Node.h"

class TextLogger;
//namespace DisjointSet {
class Node;
//}

struct Run{
  int start, end;
  char color;
  Run(int s, int e, char c): start(s), end(e), color(c) {}
};

class BlobDetector : public ObjectDetector {
  public:
//    class Run {
//      public:
//        int start, end;
//        char color;
//        Run(int start, int end, char color): start(start), end(end), color(color) { }
//    };
    class Blob {
      public:
        int left, top, right, bottom;
    };
    BlobDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    void findBlobs();
  protected:
    std::vector<std::vector<Node>> findRuns();
    std::vector<Node> findRunsInRow(unsigned char *, int);
    void mergeRuns(std::vector<std::vector<Node>> rows);
  private:
    TextLogger* textlogger;
};
