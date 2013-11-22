from ddapp import lcmUtils
from ddapp import transformUtils
import PythonQt
import bot_core as lcmbotcore

from ddapp.simpletimer import SimpleTimer

class AffordanceUpdater(object):

    def __init__(self):
        self.subscriber = None
        self.callbacks = []
        self.currentTransform = None
        self.timer = SimpleTimer()
        self.resetTime = self.timer.now()
        self.paused = True


    def addCallback(self, func):
        self.callbacks.append(func)


    def off(self):
        self.paused = True
        self._reset()


    def on(self):
        self.paused = False
        self._reset()


    def _reset(self):
        self.resetTime = self.timer.now()


    def _onFootPose(self, messageData):

        messageClass = lcmbotcore.pose_t
        m = messageClass.decode(messageData.data())
        self.currentTransform = transformUtils.transformFromPose(m.pos, m.orientation)

        if not self.paused and self.timer.elapsed() > 1.0:
            self.timer.reset()
            for func in self.callbacks:
                func(self.currentTransform, self.resetTime)


    def _init(self):

        lcmThread = lcmUtils.getGlobalLCMThread()
        self.subscriber = PythonQt.dd.ddLCMSubscriber('POSE_RIGHT_FOOT', lcmThread)
        self.subscriber.connect('messageReceived(const QByteArray&)', self._onFootPose)
        lcmThread.addSubscriber(self.subscriber)


updater = AffordanceUpdater()
updater._init()
