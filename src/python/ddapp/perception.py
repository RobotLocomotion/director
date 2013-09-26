import os
import sys
import vtk
import PythonQt
from PythonQt import QtCore, QtGui

from ddapp.timercallback import TimerCallback

import vtkDRCFiltersPython as drc


class MultiSenseSource(TimerCallback):

    def __init__(self, view, models):
        TimerCallback.__init__(self)
        self.view = view
        self.models = models
        self.reader = None
        self.lastScanLine = -1
        self.numberOfActors = 300
        self.nextActorId = 0
        self._createActors()

        self.revPolyData, self.revMapper, self.revActor = self._newActor()

        self.targetFps = 60


    def _newActor(self):
        polyData = vtk.vtkPolyData()
        mapper = vtk.vtkPolyDataMapper()
        actor = vtk.vtkActor()
        actor.GetProperty().SetOpacity(0.5)

        mapper.SetInputData(polyData)
        actor.SetMapper(mapper)
        self.view.renderer().AddActor(actor)
        return polyData, mapper, actor


    def _createActors(self):

        self.polyData = []
        self.mappers = []
        self.actors = []

        for i in xrange(self.numberOfActors):
            polyData, mapper, actor = self._newActor()
            self.polyData.append(polyData)
            self.mappers.append(mapper)
            self.actors.append(actor)

    def showRevolution(self, revId):
        self.reader.GetDataForRevolution(revId, self.revPolyData)
        self.revActor.SetVisibility(True)
        self.view.render()

    def hideRevolution(self):
        self.revActor.SetVisibility(False)

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMultisenseSource()
            self.reader.Start()

        TimerCallback.start(self)


    def updateRobotState(self):

        robotState = vtk.vtkDoubleArray()
        self.reader.GetCurrentRobotState(robotState)
        #print 'robot position:', robotState.GetValue(0), robotState.GetValue(1), robotState.GetValue(2)

        robotState = [robotState.GetValue(i) for i in xrange(robotState.GetNumberOfTuples())]
        if len(robotState) <= 7:
            return

        for model in self.models:
            model.setEstRobotState(robotState)


    def updateScanLines(self):

        currentScanLine = self.reader.GetCurrentScanLine() - 1
        scanLinesToUpdate = currentScanLine - self.lastScanLine

        if not scanLinesToUpdate:
            return


        #print 'current scanline:', currentScanLine
        #print 'scan lines to update:', scanLinesToUpdate
        #print 'updating actors:', self.nextActorId, (self.nextActorId + (scanLinesToUpdate-1)) % self.numberOfActors
        #print 'updating scan lines:', self.lastScanLine + 1, self.lastScanLine + 1 + (scanLinesToUpdate-1)

        for i in xrange(scanLinesToUpdate):
            polyData = self.polyData[(self.nextActorId + i) % self.numberOfActors]
            self.reader.GetDataForScanLine(self.lastScanLine + i + 1, polyData)

        self.lastScanLine = currentScanLine
        self.nextActorId = (self.nextActorId + scanLinesToUpdate) % self.numberOfActors

        self.view.render()


    def tick(self):

        self.updateRobotState()
        self.updateScanLines()






def init(view, models):
  global m
  m = MultiSenseSource(view, models)
  m.start()

