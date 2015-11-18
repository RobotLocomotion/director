import time


class SimpleTimer(object):

    def __init__(self):
        self.reset()

    def now(self):
        return time.time()

    def elapsed(self):
        return self.now() - self.t0

    def reset(self):
        self.t0 = self.now()


class FPSCounter(object):

    def __init__(self):
        self.averageComputer = MovingAverageComputer()
        self.printToConsole = False

    def tick(self):
        newAverage =  self.averageComputer.timer.elapsed() > self.averageComputer.timeWindow
        self.averageComputer.update(1)
        if newAverage and self.printToConsole:
            print 'fps:', self.getAverageFPS()

    def getAverageFPS(self):
        return self.averageComputer.getAverage()


class AverageComputer(object):

    def __init__(self):
        self.timer = SimpleTimer()
        self.quantity = 0.0

    def update(self, quantitySinceLastUpdate):
        self.quantity += quantitySinceLastUpdate

    def getAverage(self):
        return self.quantity / self.timer.elapsed()

    def reset(self):
        self.quantity = 0.0
        self.timer.reset()


class MovingAverageComputer(object):

    def __init__(self):
        self.timer = SimpleTimer()
        self.alpha = 0.9
        self.timeWindow = 1.0
        self.average = 0.0
        self.quantityThisWindow = 0.0

    def update(self, quantitySinceLastUpdate):
        self.quantityThisWindow += quantitySinceLastUpdate
        self._updateAverage()

    def getAverage(self):
        self._updateAverage()
        return self.average

    def _updateAverage(self):

        elapsedTime = self.timer.elapsed()

        if elapsedTime > self.timeWindow:

            # compute FPS for this time window
            averageThisWindow = self.quantityThisWindow / elapsedTime

            # update moving average
            self.average = self.alpha * averageThisWindow + (1.0 - self.alpha) * self.average

            # reset
            self.timer.reset()
            self.quantityThisWindow = 0.0
