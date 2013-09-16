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
