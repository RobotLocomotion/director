import os
import sys
import vtk
import PythonQt
from PythonQt import QtCore, QtGui

from ddapp.timercallback import TimerCallback

import vtkDRCFiltersPython as drc


class MultiSenseSource(TimerCallback):

    def __init__(self, view):
        TimerCallback.__init__(self)
        self.view = view
        self.reader = None
        self.lastScanLine = -1
        self.numberOfActors = 0
        self.nextActorId = 0
        self._createActors()

    def _createActors(self):

        self.polyData = []
        self.mappers = []
        self.actors = []

        for i in xrange(self.numberOfActors):
            polyData = vtk.vtkPolyData()
            mapper = vtk.vtkPolyDataMapper()
            actor = vtk.vtkActor()

            mapper.SetInputData(polyData)
            actor.SetMapper(mapper)
            self.view.renderer().AddActor(actor)

            self.polyData.append(polyData)
            self.mappers.append(mapper)
            self.actors.append(actor)


    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMultisenseSource()
            self.reader.Start()

        TimerCallback.start(self)


    def tick(self):

        robotState = vtk.vtkDoubleArray()
        self.reader.GetCurrentRobotState(robotState)
        #print 'robot position:', robotState.GetValue(0), robotState.GetValue(1), robotState.GetValue(2)

        robotState = [robotState.GetValue(i) for i in xrange(robotState.GetNumberOfTuples())]
        if len(robotState) == 7:
            return

        models[0].setEstRobotState(robotState)

        return

        currentScanLine = self.reader.GetCurrentScanLine()
        scanLinesToUpdate = currentScanLine - self.lastScanLine

        #print 'current scanline:', currentScanLine
        #print 'scan lines to update:', scanLinesToUpdate

        for i in xrange(scanLinesToUpdate):
            polyData = self.polyData[(self.nextActorId + i) % self.numberOfActors]
            self.reader.GetDataForScanLine(self.lastScanLine + i + 1, polyData)

        self.lastScanLine = currentScanLine
        self.nextActorId = (self.nextActorId + scanLinesToUpdate) % self.numberOfActors

        if scanLinesToUpdate:
            view.render()


def init(view):
  global m
  m = MultiSenseSource(view)
  m.start()

