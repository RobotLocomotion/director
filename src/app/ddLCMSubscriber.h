#ifndef __ddLCMSubscriber_h
#define __ddLCMSubscriber_h

#include <QObject>
#include <ddMacros.h>

#include <string>

#include <PythonQt.h>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <QTime>

#include <lcm/lcm-cpp.hpp>

#include "ddFPSCounter.h"
#include "ddAppConfigure.h"


namespace lcm
{
  class LCM;
}

class DD_APP_EXPORT ddLCMSubscriber : public QObject
 {
  Q_OBJECT

public:

  ddLCMSubscriber(const QString& channel, QObject* parent=NULL) : QObject(parent)
  {
    mChannel = channel;
    this->mEmitMessages = true;
    this->mNotifyAllMessages = false;
    this->mRequiredElapsedMilliseconds = 0;
    this->connect(this, SIGNAL(messageReceivedInQueue(const QString&)), SLOT(onMessageInQueue(const QString&)));
  }

  virtual ~ddLCMSubscriber()
  {
  }

  virtual void subscribe(lcm::LCM* lcmHandle)
  {
    mSubscription = lcmHandle->subscribe(mChannel.toAscii().data(), &ddLCMSubscriber::messageHandler, this);
  }

  virtual void unsubscribe(lcm::LCM* lcmHandle)
  {
    lcmHandle->unsubscribe(mSubscription);
    mSubscription = 0;
  }

  const QString& channel() const
  {
    return mChannel;
  }

  void setCallbackEnabled(bool enabled)
  {
    this->mEmitMessages = enabled;
  }

  bool callbackIsEnabled() const
  {
    return this->mEmitMessages;
  }

  // See notifyAllMessagesIsEnabled()
  void setNotifyAllMessagesEnabled(bool enabled)
  {
    this->mNotifyAllMessages = enabled;
  }

  // If the main thread is busy while several LCM messages are received by this
  // subscriber on the LCM thread, then this flag determines which messages the
  // main thread will see when it becomes ready to process messages.  If this
  // flag is true, then the main thread will be notified, via the messageReceived()
  // signal, for each message.  If this flag is false, then the main thread will
  // be notified only once with the most recently received message.  Set this
  // flag to true if it is important to never miss a message.  The default is
  // false, meaning that messages will be dropped if the main thread is not
  // available to process them before a new message is received.
  bool notifyAllMessagesIsEnabled() const
  {
    return this->mNotifyAllMessages;
  }

  void setSpeedLimit(double hertz)
  {
    if (hertz <= 0.0)
    {
      this->mRequiredElapsedMilliseconds = 0;
    }
    else
    {
      this->mRequiredElapsedMilliseconds = static_cast<int>(1000.0 / hertz);
    }
  }

  double getMessageRate()
  {
    return this->mFPSCounter.averageFPS();
  }

  QByteArray getNextMessage(int timeout)
  {

    QMutexLocker locker(&this->mMutex);

    QByteArray msg = this->mLastMessage;
    this->mLastMessage.clear();


    if (msg.size())
    {
      return msg;
    }

    bool haveNewMessage = this->mWaitCondition.wait(&this->mMutex, timeout);

    if (!haveNewMessage)
    {
      return QByteArray();
    }

    msg = this->mLastMessage;
    this->mLastMessage.clear();

    return msg;
  }

signals:

  void messageReceived(const QByteArray& messageData, const QString& channel);
  void messageReceivedInQueue(const QString& channel);

protected slots:

  void onMessageInQueue(const QString& channel)
  {
    QByteArray msg = this->getNextMessage(0);
    emit this->messageReceived(msg, channel);
  }


protected:


  void messageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
  {
    ddNotUsed(channel);

    QByteArray messageBytes = QByteArray((char*)rbuf->data, rbuf->data_size);

    mFPSCounter.update();

    if (this->mEmitMessages)
    {
      if (this->mRequiredElapsedMilliseconds == 0 || mTimer.elapsed() > this->mRequiredElapsedMilliseconds)
      {
        this->mTimer.restart();

        if (this->mNotifyAllMessages)
        {
          emit this->messageReceived(messageBytes, QString(channel.c_str()));
        }
        else
        {
          this->mMutex.lock();
          bool doEmit = !this->mLastMessage.size();
          this->mLastMessage = messageBytes;
          this->mMutex.unlock();

          if (doEmit)
          {
            emit this->messageReceivedInQueue(QString(channel.c_str()));
          }
        }

      }
    }
    else
    {
      this->mMutex.lock();
      this->mLastMessage = messageBytes;
      this->mMutex.unlock();
      this->mWaitCondition.wakeAll();
    }

  }

  bool mEmitMessages;
  bool mNotifyAllMessages;
  int mRequiredElapsedMilliseconds;
  mutable QMutex mMutex;
  QWaitCondition mWaitCondition;
  QByteArray mLastMessage;
  ddFPSCounter mFPSCounter;
  QTime mTimer;
  QString mChannel;
  lcm::Subscription* mSubscription;

};

#endif
