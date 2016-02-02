__author__ = 'manuelli'
import numpy as np

def getForceDirectionInWorld(q, robotStateModel, linkName, forceLocation, forceDirection):
    forceDirection = forceDirection/np.linalg.norm(forceDirection)
    linkToWorld = robotStateModel.getLinkFrame(linkName)
    forceLocationInWorld = np.array(linkToWorld.TransformPoint(forceLocation))
    forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(forceDirection))

    return forceDirectionInWorld, forceLocationInWorld