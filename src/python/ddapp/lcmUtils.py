import lcm
import PythonQt
import pickle
import atexit
import time

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
          cls._lcmThread = PythonQt.dd.ddLCMThread()
          atexit.register(cls.finalize)
          cls._lcmThread.start()
      return cls._lcmThread

  @classmethod
  def finalize(cls):
      if cls._lcmThread:
          cls._lcmThread.delete()
          cls._lcmThread = None

# Produces utime equivalent to libbot          
def timestamp_now ():
    return int (time.time () * 1000000)          
          
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


def addSubscriber(channel, messageClass=None, callback=None):

    lcmThread = getGlobalLCMThread()
    subscriber = PythonQt.dd.ddLCMSubscriber(channel, lcmThread)

    def handleMessage(messageData):
        try:
            msg = messageClass.decode(messageData.data())
        except ValueError:
            print 'error decoding message on channel:', channel
        else:
            callback(msg)

    if callback is not None:
        if messageClass is not None:
            subscriber.connect('messageReceived(const QByteArray&, const QString&)', handleMessage)
        else:
            subscriber.connect('messageReceived(const QByteArray&, const QString&)', callback)
    else:
        subscriber.setCallbackEnabled(False)

    lcmThread.addSubscriber(subscriber)
    return subscriber


def removeSubscriber(subscriber):
    lcmThread = getGlobalLCMThread()
    lcmThread.removeSubscriber(subscriber)
    if subscriber.parent() == lcmThread:
        subscriber.setParent(None)


def getNextMessage(subscriber, messageClass=None, timeout=0):

    messageData = subscriber.getNextMessage(timeout).data()

    if not messageData:
        return None

    if messageClass is not None:
        try:
            msg = messageClass.decode(messageData)
        except ValueError:
            print 'error decoding message on channel:', subscriber.channel()
            return None

        return msg
    else:
        return messageData


class MessageResponseHelper(object):

    def __init__(self, responseChannel, responseMessageClass=None):
        self.subscriber = addSubscriber(responseChannel)
        self.messageClass = responseMessageClass

    def waitForResponse(self, timeout=5000, keepAlive=True):
        response = getNextMessage(self.subscriber, self.messageClass, timeout)
        if not keepAlive:
            self.finish()
        return response

    def finish(self):
        removeSubscriber(self.subscriber)

    @staticmethod
    def publishAndWait(channel, message, responseChannel, responseMessageClass=None, timeout=5000):
        helper = MessageResponseHelper(responseChannel, responseMessageClass)
        publish(channel, message)
        return helper.waitForResponse(timeout, keepAlive=False)


def publish(channel, message):
    getGlobalLCM().publish(channel, message.encode())


def dumpMessage(msg, filename):
    pickle.dump((type(msg), msg.encode()), open(filename, 'w'))


def loadMessage(filename):
    cls, bytes = pickle.load(open(filename))
    return cls.decode(bytes)
