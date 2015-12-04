from director.timercallback import TimerCallback
import numpy as np
import time

class AnimatePropertyValue(object):
    '''
    This class is used to ramp a scalar or vector property from its current value to a
    target value using linear inteprolation.  For example:

    obj = getSomeObject()

    # fade the Alpha property to 0.0
    AnimatePropertyValue(obj, 'Alpha', 0.0).start()

    '''

    def __init__(self, obj, propertyName, targetValue, animationTime=1.0):

        self.obj = obj
        self.propertyName = propertyName
        self.animationTime = animationTime
        self.targetValue = targetValue
        self.timer = TimerCallback()

    def start(self):
        self.startTime = time.time()
        self.startValue = np.array(self.obj.getProperty(self.propertyName))
        self.targetValue = np.array(self.targetValue)
        self.timer.callback = self.tick
        self.timer.start()

    def tick(self):
        elapsed = time.time() - self.startTime
        p = elapsed/self.animationTime
        if p > 1.0:
            p = 1.0

        newValue = self.startValue + (self.targetValue - self.startValue)*p
        self.obj.setProperty(self.propertyName, newValue)

        if p == 1.0:
            self.timer.callback = None
            return False
