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
        Run(int start, char color);
    };
    class Blob {
      public:
        int left, top, right, bottom;
    };
    BlobDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    void findBlobs();
  protected:
    void findRuns(vector<vector<DisjointSet::Node<Run>>>& rows);
    vector<DisjointSet::Node<Run>> findRunsInRow(unsigned char *, int);
  private:
    TextLogger* textlogger;
};
