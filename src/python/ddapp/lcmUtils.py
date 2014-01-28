import lcm
import PythonQt
from PythonQt import QtCore
import pickle

class GlobalLCM(object):

  _handle = None
  _lcmThread = None

  @classmethod
  def get(cls):
      if cls._handle == None:
          cls._handle = lcm.LCM()
      return cls._handle

  @classmethod
  def getThread(cls):
      if cls._lcmThread == None:
          cls._lcmThread = PythonQt.dd.ddLCMThread(PythonQt.QtCore.QCoreApplication.instance())
          cls._lcmThread.start()
      return cls._lcmThread


def getGlobalLCM():
    return GlobalLCM.get()


def getGlobalLCMThread():
    return GlobalLCM.getThread()


def captureMessage(channel, messageClass, lcmHandle=None):

    lcmHandle = lcmHandle or getGlobalLCM()

    messages = []
    def handleMessage(channel, messageData):
        messages.append(messageClass.decode(messageData))

    subscription = lcmHandle.subscribe(channel, handleMessage)

    while not messages:
        lcmHandle.handle()

    lcmHandle.unsubscribe(subscription)
    return messages[0]


def captureMessageAsync(channel, messageClass):

    lcmThread = getGlobalLCMThread()
    sub = PythonQt.dd.ddLCMSubscriber(channel)

    messages = []
    def handleMessage(messageData):
        messages.append(messageClass.decode(messageData.data()))
        lcmThread.removeSubscriber(sub)

    sub.connect('messageReceived(const QByteArray&, const QString&)', handleMessage)
    lcmThread.addSubscriber(sub)

    while not messages:
        yield None

    yield messages[0]


def captureMessageCallback(channel, messageClass, callback):

    subscriber = None
    messages = []
    def handleMessage(message):
        getGlobalLCMThread().removeSubscriber(subscriber)
        if not messages:
            messages.append(True)
            callback(message)

    subscriber = addSubscriber(channel, messageClass, handleMessage)
    return subscriber


def addSubscriber(channel, messageClass, callback):

    lcmThread = getGlobalLCMThread()
    subscriber = PythonQt.dd.ddLCMSubscriber(channel, lcmThread)

    def handleMessage(messageData):
        callback(messageClass.decode(messageData.data()))

    subscriber.connect('messageReceived(const QByteArray&, const QString&)', handleMessage)
    lcmThread.addSubscriber(subscriber)
    return subscriber


def publish(channel, message):
    getGlobalLCM().publish(channel, message.encode())


def dumpMessage(msg, filename):
    pickle.dump((type(msg), msg.encode()), open(filename, 'w'))


def loadMessage(filename):
    cls, bytes = pickle.load(open(filename))
    return cls.decode(bytes)
