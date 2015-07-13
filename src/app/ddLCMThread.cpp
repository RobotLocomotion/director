#include "ddLCMThread.h"

#include "ddLCMSubscriber.h"

#include <lcm/lcm-cpp.hpp>
#include <iostream>

#ifdef _WIN32
  #include <Windows.h>
#endif

//-----------------------------------------------------------------------------
ddLCMThread::ddLCMThread(QObject* parent) : QThread(parent)
{
  mSelectTimeout = 0.3;
  mShouldStop = false;
  mLCM = 0;
}

//-----------------------------------------------------------------------------
ddLCMThread::~ddLCMThread()
{
  this->stop();
}

//-----------------------------------------------------------------------------
void ddLCMThread::initLCM()
{
  QMutexLocker locker(&mMutex);

  if (mLCM)
  {
    return;
  }

  mLCM = new lcm::LCM();
  if (!mLCM->good())
  {
    printf("initLCM() failed.\n");
    return;
  }
}

//-----------------------------------------------------------------------------
void ddLCMThread::addSubscriber(ddLCMSubscriber* subscriber)
{
  this->initLCM();
  mSubscribers.append(subscriber);
  subscriber->subscribe(mLCM);
}

//-----------------------------------------------------------------------------
void ddLCMThread::removeSubscriber(ddLCMSubscriber* subscriber)
{
  subscriber->unsubscribe(mLCM);
  mSubscribers.removeAll(subscriber);
}

//-----------------------------------------------------------------------------
bool ddLCMThread::waitForLCM(double timeout)
{
  int lcmFd = this->mLCM->getFileno();

  timeval tv;
  tv.tv_sec = (int) timeout;
  tv.tv_usec = (int) ((timeout - tv.tv_sec) * 1.0e6);

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcmFd, &fds);

  int status = select(lcmFd + 1, &fds, 0, 0, &tv);
  if (status == -1 && errno != EINTR)
  {
    printf("select() returned error: %d\n", errno);
  }
  else if (status == -1 && errno == EINTR)
  {
    printf("select() interrupted\n");
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}

//-----------------------------------------------------------------------------
void ddLCMThread::threadLoopWithSelect()
{
  while (!this->mShouldStop)
  {
    bool lcmReady = this->waitForLCM(this->mSelectTimeout);

    if (this->mShouldStop)
    {
      break;
    }

    if (lcmReady)
    {
      if (this->mLCM->handle() != 0)
      {
        std::cout << "Error: lcm->handle() returned non-zero.  Stopping thread." << std::endl;
        break;
      }
    }
  }
}

//-----------------------------------------------------------------------------
void ddLCMThread::run()
{
  this->mShouldStop = false;
  this->initLCM();
  this->threadLoopWithSelect();
}

//-----------------------------------------------------------------------------
void ddLCMThread::stop()
{
  mShouldStop = true;
  this->wait();
}
