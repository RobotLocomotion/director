import lcm

class GlobalLCM(object):

  _handle = None

  @classmethod
  def get(cls):
      if cls._handle == None:
          cls._handle = lcm.LCM()
      return cls._handle


def getGlobalLCM():
    return GlobalLCM.get()


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


def publish(channel, message):
    getGlobalLCM().publish(channel, message.encode())
