#pragma once

#include <memory/MemoryCache.h>
#include <memory/TextLogger.h>

class OdometryFilter {
  public:
    OdometryFilter(MemoryCache& cache, TextLogger*& tlogger);
    void processFrame();

  private:
    MemoryCache& cache_;
    TextLogger*& textlogger;
};
