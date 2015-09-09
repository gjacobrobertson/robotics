#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

class BlobDetector : public ObjectDetector {
  public:
    class Blob {
      public:
        int left, top, right, bottom;
    };
    BlobDetector(DETECTOR_DECLARE_ARGS);
    void init(TextLogger* tl) { textlogger = tl; }
    map<Color, list<Blob>> findBlobs();
  protected:
    list<Blob> findBlobs(Color c);
  private:
    TextLogger* textlogger;
};
