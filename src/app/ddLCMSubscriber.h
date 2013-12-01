#ifndef __ddLCMSubscriber_h
#define __ddLCMSubscriber_h

#include <QObject>
#include <ddMacros.h>

#include <string>

#include <PythonQt.h>

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
    mCallback = NULL;
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

  void setCallback(QVariant callback)
  {
    mCallback = callback.value<PythonQtObjectPtr>();
  }

  const QString& channel() const
  {
    return mChannel;
  }

signals:

  void messageReceived(const QByteArray& messageData, const QString& mChannel);

protected:


  void messageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
  {
    ddNotUsed(channel);
    emit this->messageReceived(QByteArray((char*)rbuf->data, rbuf->data_size), mChannel);

    /*
    if (mCallback)
    {
      QVariantList args;
      args << QByteArray((char*)rbuf->data, rbuf->data_size);
      QVariant result = PythonQt::self()->call(mCallback, args);
      ddNotUsed(result);
    }
    */
  }

  PythonQtObjectPtr mCallback;

  QString mChannel;
  lcm::Subscription* mSubscription;

};

#endif
