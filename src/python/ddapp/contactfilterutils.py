__author__ = 'manuelli'
import numpy as np
import collections


def getForceDirectionInWorld(q, robotStateModel, linkName, forceLocation, forceDirection):
    forceDirection = forceDirection/np.linalg.norm(forceDirection)
    linkToWorld = robotStateModel.getLinkFrame(linkName)
    forceLocationInWorld = np.array(linkToWorld.TransformPoint(forceLocation))
    forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(forceDirection))

    return forceDirectionInWorld, forceLocationInWorld


def removeElementsFromList(list, toRemove):
    for x in toRemove:
        if x in list:
            list.remove(x)

class DequePeak(collections.deque):

    def __init__(self):
        collections.deque.__init__(self)

    def peakRight(self):
        peakVal = None
        try:
            peakVal = self.pop()
            self.append(peakVal)
        except IndexError:
            peakVal = None

        return peakVal

    def peakLeft(self):
        peakVal = None
        try:
            peakVal = self.popleft()
            self.appendleft(peakVal)
        except IndexError:
            peakVal = None

        return peakVal
