#ifndef __ddLCMThread_h
#define __ddLCMThread_h

#include <QThread>
#include <QMutex>
#include "ddAppConfigure.h"


class ddLCMSubscriber;

namespace lcm
{
  class LCM;
}

class DD_APP_EXPORT ddLCMThread : public QThread
 {
  Q_OBJECT

public:

  ddLCMThread(QObject* parent=NULL);
  virtual ~ddLCMThread();

  void stop();

  void addSubscriber(ddLCMSubscriber* subscriber);
  void removeSubscriber(ddLCMSubscriber* subscriber);

  lcm::LCM* lcmHandle()
  {
    this->initLCM();
    return mLCM;
  }

 protected:

  void run();
  void threadLoopWithSelect();
  bool waitForLCM(double timeout);
  void initLCM();

  bool mShouldStop;
  double mSelectTimeout;
  QList<ddLCMSubscriber*> mSubscribers;
  lcm::LCM* mLCM;

  QMutex mMutex;
};

#endif
