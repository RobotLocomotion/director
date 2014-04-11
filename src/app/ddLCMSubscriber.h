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

namespace lcm
{
  class LCM;
}

class ddLCMSubscriber : public QObject
 {
  Q_OBJECT

public:

  ddLCMSubscriber(const QString& channel, QObject* parent=NULL) : QObject(parent)
  {
    mChannel = channel;
    this->mEmitMessages = true;
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

    if (this->mEmitMessages)
    {
      if (this->mRequiredElapsedMilliseconds == 0 || mTimer.elapsed() > this->mRequiredElapsedMilliseconds)
        {
        this->mTimer.restart();

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
    else
    {
      this->mMutex.lock();
      this->mLastMessage = messageBytes;
      this->mMutex.unlock();
      this->mWaitCondition.wakeAll();
    }

  }

  bool mEmitMessages;
  int mRequiredElapsedMilliseconds;
  mutable QMutex mMutex;
  QWaitCondition mWaitCondition;
  QByteArray mLastMessage;
  QTime mTimer;
  QString mChannel;
  lcm::Subscription* mSubscription;

};

#endif
