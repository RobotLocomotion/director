__author__ = 'manuelli'
import ddapp
from ddapp import roboturdf
import numpy as np
import vtkAll as vtk
import PythonQt
import matplotlib.pyplot as plt
import Queue
import collections
from contactfilterutils import DequePeak

import os
import os.path
import csv
import copy
import time
import itertools
import scipy.stats
import contactfilterutils as cfUtils


from PythonQt import QtCore, QtGui
from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import contactfiltergurobi
from ddapp.debugVis import DebugData
from ddapp import visualization as vis
from ddapp import gurobiutils as grbUtils
from ddapp.timercallback import TimerCallback
from ddapp import objectmodel as om
from ddapp import filterUtils
from ddapp import ioUtils

try:
    from vtk.util import numpy_support
except ImportError:
    from paraview import numpy_support


import drc as lcmdrc
import drake as lcmdrake


class ContactPointLocator(object):

    def __init__(self, robotStateModel):
        self.robotStateModel = robotStateModel
        self.loadCellsFromFile()


    def loadCellsFromFile(self, filename=None):
        if filename is None:
            filename = "test"

        drcBase = os.getenv('DRC_BASE')
        fullFilename = drcBase + "/software/ddapp/src/python/ddapp/data/" + filename + ".out"

        dataDict = ioUtils.readDataFromFile(fullFilename)
        self.createCellLocators(dataDict=dataDict)

    def createCellLocators(self, dataDict=None):

        self.locatorData = {}

        for linkName, cellCodeArray in dataDict.iteritems():
            polyData = vtk.vtkPolyData()
            self.robotStateModel.model.getLinkModelMesh(linkName, polyData)
            cellCodeArrayVTK = numpy_support.numpy_to_vtk(cellCodeArray)
            arrayName = "cellCodeArray"
            cellCodeArrayVTK.SetName(arrayName)
            polyData.GetCellData().AddArray(cellCodeArrayVTK)
            thresholdRange = [1,1]
            polyData = filterUtils.thresholdCells(polyData, arrayName, thresholdRange)
            cellData = polyData.GetCellData()
            normals = cellData.GetNormals()

            if polyData.GetNumberOfCells() == 0:
                continue

            locator = vtk.vtkCellLocator()
            locator.SetDataSet(polyData)
            locator.BuildLocator()

            meshToWorld = transformUtils.copyFrame(self.robotStateModel.getLinkFrame(linkName))
            worldToMesh = meshToWorld.GetLinearInverse()
            self.locatorData[linkName] = {'locator':locator, 'polyData': polyData, 'meshToWorld': meshToWorld,
                                          'worldToMesh': worldToMesh, 'cellData': cellData, 'normals': normals}


     # should return a dict with linkName, contactLocation, contactNormal, etc.
    def findClosestPoint(self, pointInWorldFrame):
        closestPointData = None

        for linkName in self.locatorData:
            worldToLink = self.robotStateModel.getLinkFrame(linkName).GetLinearInverse()
            pointInLinkFrame = worldToLink.TransformPoint(pointInWorldFrame)
            tempClosestPointData = self.findClosestPointSingleLink(linkName, pointInLinkFrame)

            if closestPointData is None or (tempClosestPointData['dist2'] < closestPointData['dist2']):
                closestPointData = tempClosestPointData

        return closestPointData

    # point should be in world frame
    def findClosestPointSingleLink(self, linkName, pointInLinkFrame):

        data = self.locatorData[linkName]

        pointInWorldFrame = data['meshToWorld'].TransformPoint(pointInLinkFrame)

        cellId = vtk.mutable(0)
        subId = vtk.mutable(0)
        dist2 = vtk.mutable(0)
        closestPoint = [0.0, 0.0, 0.0]
        data['locator'].FindClosestPoint(pointInWorldFrame, closestPoint, cellId, subId, dist2)


        # this is a hack to get the normals to turn out correctly
        normal = -np.array(data['normals'].GetTuple(cellId))


        # want to convert all quantities back to linkFrame so that they are portable across different robot poses
        worldToMesh = data['worldToMesh']
        closestPointLinkFrame = worldToMesh.TransformPoint(closestPoint)
        normalLinkFrame = worldToMesh.TransformVector(normal)
        closestPointData = {'linkName': linkName, 'closestPoint': closestPointLinkFrame, 'cellId': cellId,
                            'normal': normalLinkFrame, 'dist2': dist2}

        return closestPointData


    def showPolyData(self):
        for linkName, data in self.locatorData.iteritems():
            vis.showPolyData(data['polyData'], linkName)


    def removePolyData(self):
        for linkName in self.locatorData:
            om.removeFromObjectModel(om.findObjectByName(linkName))

