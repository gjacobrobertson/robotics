#pragma once

#include <vision/ObjectDetector.h>
#include <common/DisjointSet.h>

class TextLogger;

class BlobDetector : public ObjectDetector {
  public:
    class Run {
      public:
        int start, end;
        char color;
        Run(int start, int end, char color): start(start), end(end), color(color) { }
    };
    class Blob {
      public:
        int left, top, right, bottom;
    };
    BlobDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    void findBlobs();
  protected:
    vector<vector<DisjointSet::Node<Run>>> findRuns();
    vector<DisjointSet::Node<Run>> findRunsInRow(unsigned char *, int);
    void mergeRuns(vector<vector<DisjointSet::Node<Run>>>);
  private:
    TextLogger* textlogger;
};
