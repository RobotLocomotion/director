import PythonQt
from PythonQt import QtCore, QtGui
import re
import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp import cameracontrol
from ddapp import splinewidget
from ddapp import transformUtils
from ddapp import teleoppanel
from ddapp import applogic as app
from ddapp import vtkAll as vtk
from ddapp.shallowCopy import shallowCopy
from ddapp import segmentationpanel
from ddapp import segmentation
import numpy as np



def resetCameraToRobot():
    t = robotModel.getLinkFrame('utorso')
    focalPoint = [0.0, 0.0, 0.0]
    position = [-4.0, -2.0, 2.0]
    t.TransformPoint(focalPoint, focalPoint)
    t.TransformPoint(position, position)
    flyer.zoomTo(focalPoint, position)


def resetCameraToHeadView():

    head = robotModel.getLinkFrame('head')
    utorso = robotModel.getLinkFrame('utorso')

    viewDirection = np.array([1.0, 0.0, 0.0])
    utorso.TransformVector(viewDirection, viewDirection)

    cameraPosition = np.array(head.GetPosition()) + 0.10 * viewDirection

    camera = view.camera()

    focalOffset = np.array(camera.GetFocalPoint()) - np.array(camera.GetPosition())
    focalOffset /= np.linalg.norm(focalOffset)

    camera.SetPosition(cameraPosition)
    camera.SetFocalPoint(cameraPosition + focalOffset*0.03)
    camera.SetViewUp([0, 0, 1])
    camera.SetViewAngle(90)
    view.render()


def zoomToPick(displayPoint, view):
    pickedPoint, prop, _ = vis.pickProp(displayPoint, view)
    if prop:
        flyer.zoomTo(pickedPoint)


def getChildFrame(obj):
    if hasattr(obj, 'getChildFrame'):
        return obj.getChildFrame()


def placeHandModel(displayPoint, view):

    side = 'left'

    obj, _ = vis.findPickedObject(displayPoint, view)
    if isinstance(obj, vis.FrameItem):
        _, handFrame = handFactory.placeHandModelWithTransform(obj.transform, view, side=side, parent=obj.parent())
        handFrame.frameSync = vis.FrameSync()
        handFrame.frameSync.addFrame(obj)
        handFrame.frameSync.addFrame(handFrame, ignoreIncoming=True)
        return

    pickedPoint, prop, _, normal = vis.pickPoint(displayPoint, view, pickType='cells', tolerance=0.0, returnNormal=True)

    obj = vis.getObjectByProp(prop)
    if not obj:
        return

    yaxis = -normal
    zaxis = [0,0,1]
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)
    zaxis = np.cross(xaxis, yaxis)
    zaxis /= np.linalg.norm(zaxis)

    t = transformUtils.getTransformFromAxes(-zaxis, yaxis, xaxis)
    t.PostMultiply()
    t.Translate(pickedPoint)
    _, handFrame = handFactory.placeHandModelWithTransform(t, view, side=side, parent=obj)

    syncFrame = getChildFrame(obj)
    if syncFrame:
        handFrame.frameSync = vis.FrameSync()
        handFrame.frameSync.addFrame(syncFrame)
        handFrame.frameSync.addFrame(handFrame, ignoreIncoming=True)


def highlightSelectedLink(displayPoint, view):

    model = robotModel.model
    pickedPoint, _, polyData = vis.pickProp(displayPoint, view)

    linkName = model.getLinkNameForMesh(polyData)
    if not linkName:
        return False

    colorNoHighlight = QtGui.QColor(190, 190, 190)
    colorHighlight = QtCore.Qt.red

    linkNames = [linkName]
    model.setColor(colorNoHighlight)

    for name in linkNames:
        model.setLinkColor(name, colorHighlight)

    return True


def toggleFrameWidget(displayPoint, view):

    obj, _ = vis.findPickedObject(displayPoint, view)

    if not isinstance(obj, vis.FrameItem):
        obj = getChildFrame(obj)

    if not obj:
        return False

    edit = not obj.getProperty('Edit')
    obj.setProperty('Edit', edit)

    parent = obj.parent()
    if getChildFrame(parent) == obj:
        parent.setProperty('Alpha', 0.5 if edit else 1.0)

    return True


def toggleFootstepWidget(displayPoint, view):

    obj, _ = vis.findPickedObject(displayPoint, view)

    if not obj:
        return False

    name = obj.getProperty('Name')

    if name in ('footstep widget', 'footstep widget frame'):
        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        return True

    match = re.match('^step (\d+)$', name)
    if not match:
        return False

    stepIndex = int(match.group(1))

    existingWidget = om.findObjectByName('footstep widget')
    if existingWidget:
        previousStep = existingWidget.stepIndex
        print 'have existing widget for step:', stepIndex

        om.removeFromObjectModel(existingWidget)
        if previousStep == stepIndex:
            print 'returning because widget was for selected step'
            return True


    footMesh = shallowCopy(obj.polyData)
    footFrame = transformUtils.copyFrame(obj.getChildFrame().transform)
    footFrame = transformUtils.frameFromPositionAndRPY(footFrame.GetPosition(), [0.0, 0.0, footFrame.GetOrientation()[2]])

    footObj = vis.showPolyData(footMesh, 'footstep widget', parent='planning', alpha=0.2)
    footObj.stepIndex = stepIndex
    frameObj = vis.showFrame(footFrame, 'footstep widget frame', parent=footObj, scale=0.2)
    footObj.actor.SetUserTransform(frameObj.transform)
    footObj.setProperty('Color', obj.getProperty('Color'))
    frameObj.setProperty('Edit', True)

    def onFootWidgetChanged(frame):
        footstepsDriver.onStepModified(stepIndex - 1, frame)

    frameObj.connectFrameModified(onFootWidgetChanged)
    return True


def reachToHand(pickedObj):
    side = 'left'
    frame = pickedObj.getChildFrame()
    assert frame

    goalFrame = teleoppanel.panel.endEffectorTeleop.newReachTeleop(frame.transform, side)
    goalFrame.frameSync = vis.FrameSync()
    goalFrame.frameSync.addFrame(goalFrame, ignoreIncoming=True)
    goalFrame.frameSync.addFrame(frame)


def showRightClickMenu(displayPoint, view):

    pickedObj, pickedPoint = vis.findPickedObject(displayPoint, view)
    if not pickedObj:
        return

    objectName = pickedObj.getProperty('Name')
    if objectName == 'grid':
        return

    displayPoint = displayPoint[0], view.height - displayPoint[1]

    globalPos = view.mapToGlobal(QtCore.QPoint(*displayPoint))

    menu = QtGui.QMenu(view)

    widgetAction = QtGui.QWidgetAction(menu)
    label = QtGui.QLabel('  ' + objectName + '  ')
    widgetAction.setDefaultWidget(label)
    menu.addAction(widgetAction)
    menu.addSeparator()

    def onDelete():
        om.removeFromObjectModel(pickedObj)

    def onHide():
        pickedObj.setProperty('Visible', False)

    def onSelect():
        om.setActiveObject(pickedObj)

    def onTouch():
        reachToHand(pickedObj)

    def onSpline():
        splinewidget.newSpline(pickedObj, view)


    def getPointCloud(obj):
        try:
            obj = obj.model.polyDataObj
        except AttributeError:
            pass
        try:
            obj.polyData
        except AttributeError:
            return None
        if obj and obj.polyData.GetNumberOfPoints() and (obj.polyData.GetNumberOfCells() == obj.polyData.GetNumberOfVerts()):
            return obj

    pointCloudObj = getPointCloud(pickedObj)


    def onSegmentGround():
        groundPoints, scenePoints =  segmentation.removeGround(pointCloudObj.polyData)
        vis.showPolyData(groundPoints, 'ground points', color=[0,1,0], parent='segmentation')
        vis.showPolyData(scenePoints, 'scene points', color=[1,0,1], parent='segmentation')
        pickedObj.setProperty('Visible', False)


    def onCopyPointCloud():
        polyData = vtk.vtkPolyData()
        polyData.DeepCopy(pointCloudObj.polyData)
        obj = vis.showPolyData(polyData, pointCloudObj.getProperty('Name') + ' copy', color=[0,1,0], parent='segmentation')
        om.setActiveObject(obj)
        pickedObj.setProperty('Visible', False)

    def onSegmentTableScene():
        data = segmentation.segmentTableScene(pointCloudObj.polyData, pickedPoint)
        tableObj = vis.showPolyData(data.table.mesh, 'table', color=[0,1,0], parent='segmentation')
        vis.showPolyData(data.table.box, 'table box', color=[0,1,0], parent=tableObj, alpha=0.2)
        vis.showPolyData(data.table.points, 'table points', color=[0,1,0], parent=tableObj, visible=False)


        colors =  [ QtCore.Qt.green,
                    QtCore.Qt.red,
                    QtCore.Qt.blue,
                    QtCore.Qt.yellow,
                    QtCore.Qt.magenta,
                    QtCore.Qt.cyan,
                    QtCore.Qt.darkCyan,
                    QtCore.Qt.darkGreen,
                    QtCore.Qt.darkMagenta ]

        colors = [QtGui.QColor(c) for c in colors]
        colors = [(c.red()/255.0, c.green()/255.0, c.blue()/255.0) for c in colors]

        for i, cluster in enumerate(data.clusters):
            name = 'object %d' % i
            color= colors[i+1] #segmentation.getRandomColor()
            clusterObj = vis.showPolyData(cluster.mesh, name, color=color, parent='segmentation', alpha=0.4)
            vis.showPolyData(cluster.box, name + ' box', color=color, parent=clusterObj, alpha=1.0)
            pts = vis.showPolyData(cluster.points, name + ' points', color=color, parent=clusterObj, visible=True, alpha=1.0)
            pts.setProperty('Point Size', 8)

    actions = [
      (None, None),
      ('Hide', onHide),
      ('Delete', onDelete),
      ('Select', onSelect)
      ]

    if 'robotiq' in objectName.lower():
        actions.extend([
            (None, None),
            ('Touch', onTouch),
            ('Spline', onSpline),
            ])

    if pointCloudObj:
        actions.extend([
            (None, None),
            ('Copy Pointcloud', onCopyPointCloud),
            ('Segment Ground', onSegmentGround),
            ('Segment Table', onSegmentTableScene)
            ])

    for actionName, func in actions:
        if not actionName:
            menu.addSeparator()
        else:
            action = menu.addAction(actionName)
            action.connect('triggered()', func)

    selectedAction = menu.popup(globalPos)


class ViewEventFilter(object):

    def __init__(self, view):
        self.view = view
        self.mouseStart = None
        self.initEventFilter()

    def filterEvent(self, obj, event):

        if event.type() == QtCore.QEvent.MouseButtonDblClick and event.button() == QtCore.Qt.LeftButton:
            self.onLeftDoubleClick(event)

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.RightButton:
            self.mouseStart = QtCore.QPoint(event.pos())

        elif event.type() == QtCore.QEvent.MouseMove and self.mouseStart is not None:
            delta = QtCore.QPoint(event.pos()) - self.mouseStart
            if delta.manhattanLength() > 3:
                self.mouseStart = None

        elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.RightButton and self.mouseStart is not None:
            self.mouseStart = None
            self.onRightClick(event)

    def consumeEvent(self):
        self.eventFilter.setEventHandlerResult(True)

    def onLeftDoubleClick(self, event):

        displayPoint = vis.mapMousePosition(self.view, event)

        if toggleFootstepWidget(displayPoint, self.view):
            return

        if toggleFrameWidget(displayPoint, self.view):
            return

        #if highlightSelectedLink(displayPoint, self.view):
        #    return

        if segmentationpanel.activateSegmentationMode():
            return

    def onRightClick(self, event):
        displayPoint = vis.mapMousePosition(self.view, event)
        showRightClickMenu(displayPoint, self.view)

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonRelease)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


class KeyEventFilter(object):

    def __init__(self, view):
        self.view = view
        self.initEventFilter()

    def filterEvent(self, obj, event):

        if event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'f':
                self.eventFilter.setEventHandlerResult(True)
                zoomToPick(self.getCursorDisplayPosition(), self.view)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                resetCameraToRobot()
            elif str(event.text()).lower() == 's':
                self.eventFilter.setEventHandlerResult(True)
                placeHandModel(self.getCursorDisplayPosition(), self.view)

    def getCursorDisplayPosition(self):
        cursorPos = self.view.mapFromGlobal(QtGui.QCursor.pos())
        return cursorPos.x(), self.view.height - cursorPos.y()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


def setupGlobals(handFactory_, robotModel_, footstepsDriver_, flyer_):

    global handFactory, robotModel, footstepsDriver, flyer

    handFactory = handFactory_
    robotModel = robotModel_
    footstepsDriver = footstepsDriver_
    flyer = flyer_


class ViewBehaviors(object):

    def __init__(self, view, handFactory, robotModel, footstepsDriver):
        self.view = view
        self.handFactory = handFactory
        self.robotModel = robotModel

        self.keyEventFilter = KeyEventFilter(view)
        self.mouseEventFilter = ViewEventFilter(view)
        self.flyer = cameracontrol.Flyer(view)

        setupGlobals(handFactory, robotModel, footstepsDriver, self.flyer)


