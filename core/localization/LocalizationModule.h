#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <localization/KalmanFilter.h>

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void initBallFilter();
    void processFrame();

    void loadParams(LocalizationParams params);
  protected:
    typedef KalmanFilter<4, 4> BallFilter;
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    BallFilter ball_filter_;
    bool seen_last_frame_;
    Point2D last_seen_ball;
    int frames_since_last_seen_;
};
