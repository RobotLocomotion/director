#ifndef __ddFPSCounter_h
#define __ddFPSCounter_h

#include <QTime>
#include "ddAppConfigure.h"


// A class for keeping a exponential moving average of frames per second.

class DD_APP_EXPORT ddFPSCounter
{
public:


  ddFPSCounter()
  {
    mAlpha = 0.9;
    mTimeWindow = 1.0;
    mAverageFPS = 0.0;
    mFramesThisWindow = 0;
    mTime.restart();
  }

  ~ddFPSCounter()
  {
  }

  double alpha() const
  {
    return mAlpha;
  }

  void setAlpha(double alpha)
  {
    mAlpha = alpha;
  }

  double timeWindow() const
  {
    return mTimeWindow;
  }

  void setTimeWindow(double seconds)
  {
    mTimeWindow = seconds;
  }

  void update()
  {
    ++mFramesThisWindow;
    updateAverage();
  }

  double averageFPS()
  {
    updateAverage();
    return mAverageFPS;
  }

private:

  void updateAverage()
  {
    // check if a time window has elapsed
    double elapsedTime = mTime.elapsed() / 1000.0;

    if (elapsedTime > mTimeWindow)
    {
      // compute FPS for this time window
      double averageFPSThisWindow = mFramesThisWindow / elapsedTime;

      // update moving average
      mAverageFPS = mAlpha * averageFPSThisWindow + (1.0 - mAlpha) * mAverageFPS;

      // reset counters
      mTime.restart();
      mFramesThisWindow = 0;
    }
  }

  ddFPSCounter(const ddFPSCounter&); // Not implemented
  void operator=(const ddFPSCounter&); // Not implemented

  double mAlpha;
  double mAverageFPS;
  double mTimeWindow;

  size_t mFramesThisWindow;

  QTime mTime;
};

#endif
