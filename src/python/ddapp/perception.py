import os
import sys
import vtk
import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.objectmodel as om

from ddapp.timercallback import TimerCallback

import vtkDRCFiltersPython as drc


'''

>>> 
>>> perception.m.reader.GetTransform('SCAN', 'local', perception.m.lastRobotStateUtime, trans)
>>> print trans.GetMatrix()
vtkMatrix4x4 (0xec978b0)
  Debug: Off
  Modified Time: 4619724
  Reference Count: 2
  Registered Events: (none)
  Elements:
    0.835177 -0.192944 0.515025 0.263848 
    0.0033693 0.93822 0.346022 -0.0175402 
    -0.54997 -0.287255 0.78423 1.78908 
    0 0 0 1 


>>> p = [0,0,0]
>>> trans.TransformPoint(p)
(0.26384816893784385, -0.01754017254221784, 1.7890823239833162)
>>> import vtkPointCloudUtils
>>> from vtkPointCloudUtils import debugVis
Traceback (most recent call last):
  File "<console>", line 1, in <module>
  File "/home/drc/source/drc/paraview/vtkPointCloudUtils/debugVis.py", line 70
    self.addPolyData(cone.GetOutput(), color)
   ^
IndentationError: unexpected indent
>>> 
>>> from vtkPointCloudUtils import debugVis
>>> d = debugVis.DebugData()
>>> d.addSphere(p, radius=0.05, color=[0,1,0])
>>> mapper = vtk.vtkMapper()
Traceback (most recent call last):
  File "<console>", line 1, in <module>
TypeError: this is an abstract class and cannot be instantiated
>>> mapper = vtk.vtkPolyDataMapper()
>>> mapper.SetInputData(d.getPolyData())
>>> actor = vtk.vtkActor()
>>> view.renderer().AddActor(actor)
>>> d
<vtkPointCloudUtils.debugVis.DebugData object at 0xd580290>
>>> d.getPolyData().GetNumberOfPoints()
530L
>>> actor.SetMapper(mapper)
>>> p
[0, 0, 0]
>>> scanPos = perception.m.reader.GetTransform('SCAN', 'local', perception.m.lastRobotStateUtime, trans)
>>> scanPos
>>> scanPos = trans.TransformPoint(p)
>>> scanPos
(0.26384816893784385, -0.01754017254221784, 1.7890823239833162)
>>> d = debugVis.DebugData()
>>> d.addSphere(scanPos, radius=0.05, color=[0,1,0])
>>> mapper.SetInputData(d.getPolyData())

'''

_view = None
def getRenderView():
    return _view


_debugItem = None

def updateDebugItem(polyData):
    global _debugItem
    if not _debugItem:
        debugItem = PolyDataItem('', polyData)
        

class MultisenseItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'Multisense', om.Icons.Laser)

        self.model = model
        self.addProperty('Enabled', True)
        self.addProperty('Visible', model.visible)
        self.addProperty('Point Size', model.pointSize)
        self.addProperty('Alpha', model.alpha)

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        #self.addProperty('Scanline Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertyName)

        if propertyName == 'Enabled':
            if self.getProperty('Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Point Size':
            self.model.setPointSize(self.getProperty(propertyName))

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Point Size':
            return om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False)
        else:
            return om.ObjectModelItem.getPropertyAttributes(self, propertyName)



class PolyDataItem(om.ObjectModelItem):

    def __init__(self, name, polyData):

        om.ObjectModelItem.__init__(self, name, om.Icons.Robot)

        self.polyData = polyData
        self.mapper = vtk.vtkPolyDataMapper()
        self.actor = vtk.vtkActor()
        getRenderView().renderer().AddActor(actor)
        self.addProperty('Visible', True)
        self.addProperty('Alpha', 1.0)
        self.addProperty('Color', QtGui.QColor(255,255,255))


    def setPolyData(self, polyData):
        self.polyData = polyData
        self.mapper.SetInputData(polyData)

    def _onPropertyChanged(self, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertyName)

        if propertyName == 'Alpha':
            self.actor.GetProperty().SetOpacity(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.actor.SetVisibility(self.getProperty(propertyName))

        elif propertyName == 'Color':
            color = self.getProperty(propertyName)
            color = [color.r()/255.0, color.g()/255.0, color.b()/255.0]
            self.actor.GetProperty().SetColor(color)


    def getPropertyAttributes(self, propertyName):
        pass
        #if propertyName == '...':
        #    return om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False)
        #else:
        #    return om.ObjectModelItem.getPropertyAttributes(self, propertyName)


class MultiSenseSource(TimerCallback):

    def __init__(self, view, models):
        TimerCallback.__init__(self)
        self.view = view
        self.models = models
        self.reader = None
        self.displayedRevolution = -1
        self.lastScanLine = -1
        self.numberOfActors = 1
        self.nextActorId = 0
        self.lastRobotStateUtime = 0
        self.pointSize = 1
        self.alpha = 0.5
        self.visible = True
        self._createActors()

        self.revPolyData, self.revMapper, self.revActor = self._newActor()

        self.setPointSize(self.pointSize)
        self.setAlpha(self.alpha)
        self.targetFps = 60


    def _newActor(self):
        polyData = vtk.vtkPolyData()
        mapper = vtk.vtkPolyDataMapper()
        actor = vtk.vtkActor()

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
            actor.GetProperty().SetColor(1,0,0)

    def getScanToLocal(self):
        return None

    def showRevolution(self, revId):
        self.reader.GetDataForRevolution(revId, self.revPolyData)
        self.revActor.SetVisibility(True)
        self.view.render()
        self.displayedRevolution = revId

    def hideRevolution(self):
        self.revActor.SetVisibility(False)

    def setPointSize(self, pointSize):
        for actor in self.actors:
            actor.GetProperty().SetPointSize(pointSize+2)
        self.revActor.GetProperty().SetPointSize(pointSize)

    def setAlpha(self, alpha):
        self.alpha = alpha
        for actor in self.actors:
            actor.GetProperty().SetOpacity(alpha)
        self.revActor.GetProperty().SetOpacity(alpha)

    def setVisible(self, visible):
        self.visible = visible
        for actor in self.actors:
            actor.SetVisibility(visible)
        self.revActor.SetVisibility(visible)

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMultisenseSource()
            self.reader.Start()

        TimerCallback.start(self)

    def updateRobotState(self):

        robotState = vtk.vtkDoubleArray()
        timestamp = self.reader.GetCurrentRobotState(robotState)
        if timestamp == self.lastRobotStateUtime:
            return

        self.lastRobotStateUtime = timestamp
        robotState = [robotState.GetValue(i) for i in xrange(robotState.GetNumberOfTuples())]
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


    def updateRevolution(self):

        currentRevolution = self.reader.GetCurrentRevolution() - 1

        if currentRevolution == self.displayedRevolution:
            return

        self.showRevolution(currentRevolution)
        self.view.render()


    def updateDebugItems(self):
        pass

    def tick(self):

        self.updateRobotState()
        self.updateRevolution()
        self.updateScanLines()
        self.updateDebugItems()



def init(view, models):
    global _multisenseItem
    global _view

    _view = view

    m = MultiSenseSource(view, models)
    m.start()

    sensorsFolder = om.addContainer('sensors')

    _multisenseItem = MultisenseItem(m)
    om.addToObjectModel(_multisenseItem, sensorsFolder)
    return _multisenseItem


