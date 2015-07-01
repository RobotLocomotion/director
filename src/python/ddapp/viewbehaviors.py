import PythonQt
from PythonQt import QtCore, QtGui
import re
import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp.timercallback import TimerCallback
from ddapp import affordanceitems
#from ddapp import lcmUtils
from ddapp import callbacks
from ddapp import cameracontrol
#from ddapp import midi
from ddapp import propertyset
from ddapp import splinewidget
from ddapp import transformUtils
#from ddapp import teleoppanel
#from ddapp import footstepsdriverpanel
from ddapp import applogic as app
from ddapp import vtkAll as vtk
from ddapp import filterUtils
from ddapp.shallowCopy import shallowCopy
#from ddapp import segmentationpanel
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import frameupdater
import numpy as np
import ioUtils
import os
import random
import colorsys

# todo: refactor these global variables
# several functions in this module depend on these global variables
# which are set by calling ViewBehaviors.addRobotBehaviors().
# These could be refactored to be members of a new behaviors class.
robotModel = None
handFactory = None
neckDriver = None
footstepsDriver = None
robotLinkSelector = None
lastRandomColor = 0.0

class MidiBehaviorControl(object):

    def __init__(self):
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.tick
        self.stop = self.timer.stop
        self.reader = None
        self.initReader()

        self.inputs = {
            'slider' : [0, 8, True],
            'dial' : [16, 8, True],
            'r_button' : [64, 8, False],
            'm_button' : [48, 8, False],
            's_button' : [32, 8, False],
            'track_left' : [58, 1, False],
            'track_right' : [59, 1, False],
            'cycle' : [46, 1, False],
            'marker_set' : [60, 1, False],
            'marker_left' : [61, 1, False],
            'marker_right' : [62, 1, False],
            'rewind' : [43, 1, False],
            'fastforward' : [44, 1, False],
            'stop' : [42, 1, False],
            'play' : [41, 1, False],
            'record' : [45, 1, False],
        }

        signalNames = []

        for inputName, inputDescription in self.inputs.iteritems():
            channelStart, numChannels, isContinuous = inputDescription

            for i in xrange(numChannels):

                channelId = '' if numChannels == 1 else '_%d' % i
                if isContinuous:
                    signalNames.append('%s%s_value_changed' % (inputName, channelId))
                else:
                    signalNames.append('%s%s_pressed' % (inputName, channelId))
                    signalNames.append('%s%s_released' % (inputName, channelId))

        self.callbacks = callbacks.CallbackRegistry(signalNames)


    def start(self):
        self.initReader()
        if self.reader is not None:
            self.timer.start()

    def initReader(self):

        if self.reader:
            return
        try:
            self.reader = midi.MidiReader(midi.findKorgNanoKontrol2())
        except:
            print 'midi controller not found.'
            self.reader = None


    def onMidiCommand(self, channel, value):

        #print channel, '%.2f' % value

        inputs = self.inputs

        for inputName, inputDescription in inputs.iteritems():
            channelStart, numChannels, isContinuous = inputDescription

            if channelStart <= channel < (channelStart + numChannels):

                if numChannels > 1:
                    inputName = '%s_%d' % (inputName, channel - channelStart)

                if isContinuous:
                    self.onContinuousInput(inputName, value)
                elif value == 1:
                    self.onButtonDown(inputName)
                elif value == 0:
                    self.onButtonUp(inputName)


    def onContinuousInput(self, name, value):
        #print name, '%.2f' % value
        self.callbacks.process(name + '_value_changed', value)

    def onButtonDown(self, name):
        #print name, 'down'
        self.callbacks.process(name + '_pressed')

    def onButtonUp(self, name):
        #print name, 'up'
        self.callbacks.process(name + '_released')

    def tick(self):
        try:
            messages = self.reader.getMessages()
        except:
            messages = []

        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        for channel, value in targets.iteritems():
            position = value/127.0
            self.onMidiCommand(channel, position)



def resetCameraToRobot(view):
    t = robotModel.getLinkFrame('utorso')
    focalPoint = [0.0, 0.0, 0.0]
    position = [-4.0, -2.0, 2.0]
    t.TransformPoint(focalPoint, focalPoint)
    t.TransformPoint(position, position)
    flyer = cameracontrol.Flyer(view)
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
    if not prop:
        return
    flyer = cameracontrol.Flyer(view)
    flyer.zoomTo(pickedPoint)


def getChildFrame(obj):
    if hasattr(obj, 'getChildFrame'):
        return obj.getChildFrame()


def placeHandModel(displayPoint, view, side='left'):

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
    handObj, handFrame = handFactory.placeHandModelWithTransform(t, view, side=side, parent=obj)

    syncFrame = getChildFrame(obj)
    if syncFrame:
        handFrame.frameSync = vis.FrameSync()
        handFrame.frameSync.addFrame(handFrame, ignoreIncoming=True)
        handFrame.frameSync.addFrame(syncFrame)


class RobotLinkSelector(object):

    def __init__(self):
        self.selectedLink = None
        self.setupMenuAction()

    def setupMenuAction(self):
        self.action = app.addMenuAction('Tools', 'Robot Link Selector')
        self.action.setCheckable(True)
        self.action.checked = False

    def enabled(self):
        return self.action.checked == True

    def selectLink(self, displayPoint, view):

        if not self.enabled():
            return False

        robotModel, _ = vis.findPickedObject(displayPoint, view)

        try:
            robotModel.model.getLinkNameForMesh
        except AttributeError:
            return False

        model = robotModel.model

        pickedPoint, _, polyData = vis.pickProp(displayPoint, view)

        linkName = model.getLinkNameForMesh(polyData)
        if not linkName:
            return False

        fadeValue = 1.0 if linkName == self.selectedLink else 0.05

        for name in model.getLinkNames():
            linkColor = model.getLinkColor(name)
            linkColor.setAlphaF(fadeValue)
            model.setLinkColor(name, linkColor)

        if linkName == self.selectedLink:
            self.selectedLink = None
            vis.hideCaptionWidget()
            om.removeFromObjectModel(om.findObjectByName('selected link frame'))

        else:
            self.selectedLink = linkName
            linkColor = model.getLinkColor(self.selectedLink)
            linkColor.setAlphaF(1.0)
            model.setLinkColor(self.selectedLink, linkColor)
            vis.showCaptionWidget(robotModel.getLinkFrame(self.selectedLink).GetPosition(), self.selectedLink, view=view)
            vis.updateFrame(robotModel.getLinkFrame(self.selectedLink), 'selected link frame', scale=0.2, parent=robotModel)

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


def newWalkingGoal(displayPoint, view):

    footFrame = footstepsDriver.getFeetMidPoint(robotModel)

    worldPt1, worldPt2 = vis.getRayFromDisplayPoint(view, displayPoint)
    groundOrigin = footFrame.GetPosition()
    groundNormal = [0.0, 0.0, 1.0]
    selectedGroundPoint = [0.0, 0.0, 0.0]

    t = vtk.mutable(0.0)
    vtk.vtkPlane.IntersectWithLine(worldPt1, worldPt2, groundNormal, groundOrigin, t, selectedGroundPoint)

    footFrame.Translate(np.array(selectedGroundPoint) - np.array(footFrame.GetPosition()))

    footstepsdriverpanel.panel.onNewWalkingGoal(footFrame)


def toggleFootstepWidget(displayPoint, view, useHorizontalWidget=False):

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

    if useHorizontalWidget:
        rpy = [0.0, 0.0, transformUtils.rollPitchYawFromTransform(footFrame)[2]]
        footFrame = transformUtils.frameFromPositionAndRPY(footFrame.GetPosition(), np.degrees(rpy))

    footObj = vis.showPolyData(footMesh, 'footstep widget', parent='planning', alpha=0.2)
    footObj.stepIndex = stepIndex
    frameObj = vis.showFrame(footFrame, 'footstep widget frame', parent=footObj, scale=0.2)
    footObj.actor.SetUserTransform(frameObj.transform)
    footObj.setProperty('Color', obj.getProperty('Color'))
    frameObj.setProperty('Edit', True)

    rep = frameObj.widget.GetRepresentation()
    rep.SetTranslateAxisEnabled(2, False)
    rep.SetRotateAxisEnabled(0, False)
    rep.SetRotateAxisEnabled(1, False)
    frameObj.widget.HandleRotationEnabledOff()

    walkGoal = om.findObjectByName('walking goal')
    if walkGoal:
        walkGoal.setProperty('Edit', False)


    def onFootWidgetChanged(frame):
        footstepsDriver.onStepModified(stepIndex - 1, frame)

    frameObj.connectFrameModified(onFootWidgetChanged)
    return True


def reachToFrame(frameObj, side, collisionObj):
    goalFrame = teleoppanel.panel.endEffectorTeleop.newReachTeleop(frameObj.transform, side, collisionObj)
    goalFrame.frameSync = vis.FrameSync()
    goalFrame.frameSync.addFrame(goalFrame, ignoreIncoming=True)
    goalFrame.frameSync.addFrame(frameObj)


def getAsFrame(obj):
    if isinstance(obj, vis.FrameItem):
        return obj
    elif hasattr(obj, 'getChildFrame'):
        return obj.getChildFrame()


def isGraspSeed(obj):
    return hasattr(obj, 'side')


def getCollisionParent(obj):
    '''
    If obj is an affordance, return obj
    If obj is a frame or a grasp seed, return first parent.
    '''
    if isinstance(obj, vis.FrameItem):
        return obj.parent()
    if isGraspSeed(obj):
        return obj.parent()
    else:
        return obj


# The most recently cached PickedPoint - available as input to any other algorithm
lastCachedPickedPoint = np.array([0,0,0])

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
    label = QtGui.QLabel('<b>%s</b>' % objectName)
    label.setContentsMargins(9,9,6,6)
    widgetAction.setDefaultWidget(label)
    menu.addAction(widgetAction)
    menu.addSeparator()


    propertiesPanel = PythonQt.dd.ddPropertiesPanel()
    propertiesPanel.setBrowserModeToWidget()
    propertyset.PropertyPanelHelper.addPropertiesToPanel(pickedObj.properties, propertiesPanel)

    def onPropertyChanged(prop):
        om.PropertyPanelHelper.setPropertyFromPanel(prop, propertiesPanel, pickedObj.properties)
    propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)

    propertiesMenu = menu.addMenu('Properties')
    propertiesWidgetAction = QtGui.QWidgetAction(propertiesMenu)
    propertiesWidgetAction.setDefaultWidget(propertiesPanel)
    propertiesMenu.addAction(propertiesWidgetAction)


    def onDelete():
        om.removeFromObjectModel(pickedObj)

    def onHide():
        pickedObj.setProperty('Visible', False)

    def onSelect():
        om.setActiveObject(pickedObj)

    reachFrame = getAsFrame(pickedObj)
    collisionParent = getCollisionParent(pickedObj)
    def onReachLeft():
        reachToFrame(reachFrame, 'left', collisionParent)
    def onReachRight():
        reachToFrame(reachFrame, 'right', collisionParent)

    def flipHandSide():
        for obj in [pickedObj] + pickedObj.children():
            if not isGraspSeed(obj):
                continue
            side = 'right' if obj.side == 'left' else 'left'
            obj.side = side
            color = [1.0, 1.0, 0.0]
            if side == 'right':
                color = [0.33, 1.0, 0.0]
            obj.setProperty('Color', color)

    def flipHandThumb():
        handFrame = pickedObj.children()[0]
        t = transformUtils.copyFrame(handFrame.transform)
        t.PreMultiply()
        t.RotateY(180)
        handFrame.copyFrame(t)
        pickedObj._renderAllViews()

    def onSplineLeft():
        splinewidget.planner.newSpline(pickedObj, 'left')
    def onSplineRight():
        splinewidget.planner.newSpline(pickedObj, 'right')


    def getPointCloud(obj):
        try:
            obj = obj.model.polyDataObj
        except AttributeError:
            pass
        try:
            obj.polyData
        except AttributeError:
            return None
        if obj and obj.polyData.GetNumberOfPoints():# and (obj.polyData.GetNumberOfCells() == obj.polyData.GetNumberOfVerts()):
            return obj


    pointCloudObj = getPointCloud(pickedObj)
    affordanceObj = pickedObj if isinstance(pickedObj, affordanceitems.AffordanceItem) else None

    def onSegmentGround():
        groundPoints, scenePoints =  segmentation.removeGround(pointCloudObj.polyData)
        vis.showPolyData(groundPoints, 'ground points', color=[0,1,0], parent='segmentation')
        vis.showPolyData(scenePoints, 'scene points', color=[1,0,1], parent='segmentation')
        pickedObj.setProperty('Visible', False)


    def onCopyPointCloud():
        global lastRandomColor
        polyData = vtk.vtkPolyData()
        polyData.DeepCopy(pointCloudObj.polyData)
        
        if pointCloudObj.getChildFrame():
            polyData = segmentation.transformPolyData(polyData, pointCloudObj.getChildFrame().transform)
        polyData = segmentation.addCoordArraysToPolyData(polyData)

        # generate random color, and average with a common color to make them generally similar
        lastRandomColor = lastRandomColor + 0.1 + 0.1*random.random()
        rgb = colorsys.hls_to_rgb(lastRandomColor, 0.7, 1.0)
        obj = vis.showPolyData(polyData, pointCloudObj.getProperty('Name') + ' copy', color=rgb, parent='point clouds')

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(filterUtils.computeCentroid(polyData))
        segmentation.makeMovable(obj, t)
        om.setActiveObject(obj)
        pickedObj.setProperty('Visible', False)

    def onMergeIntoPointCloud():
        allPointClouds = om.findObjectByName('point clouds')
        if allPointClouds:
            allPointClouds = [i.getProperty('Name') for i in allPointClouds.children()]
        sel =  QtGui.QInputDialog.getItem(None, "Point Cloud Merging", "Pick point cloud to merge into:", allPointClouds, current=0, editable=False)
        sel = om.findObjectByName(sel)

        # Make a copy of each in same frame
        polyDataInto = vtk.vtkPolyData()
        polyDataInto.ShallowCopy(sel.polyData)
        if sel.getChildFrame():
            polyDataInto = segmentation.transformPolyData(polyDataInto, sel.getChildFrame().transform)

        polyDataFrom = vtk.vtkPolyData()
        polyDataFrom.DeepCopy(pointCloudObj.polyData)
        if pointCloudObj.getChildFrame():
            polyDataFrom = segmentation.transformPolyData(polyDataFrom, pointCloudObj.getChildFrame().transform)

        # Actual merge
        append = filterUtils.appendPolyData([polyDataFrom, polyDataInto])
        if sel.getChildFrame():
            polyDataInto = segmentation.transformPolyData(polyDataInto, sel.getChildFrame().transform.GetInverse())

        # resample
        append = segmentationroutines.applyVoxelGrid(append, 0.01)
        append = segmentation.addCoordArraysToPolyData(append)

        # Recenter the frame
        sel.setPolyData(append)
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(filterUtils.computeCentroid(append))
        segmentation.makeMovable(sel, t)

        # Hide the old one
        if pointCloudObj.getProperty('Name') in allPointClouds:
            pointCloudObj.setProperty('Visible', False)


    def onSegmentTableScene():
        data = segmentation.segmentTableScene(pointCloudObj.polyData, pickedPoint)
        vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')

    def onSegmentDrillAlignedWithTable():
        segmentation.segmentDrillAlignedWithTable(pickedPoint, pointCloudObj.polyData)

    def onCachePickedPoint():
        ''' Cache the Picked Point for general purpose use'''
        global lastCachedPickedPoint
        lastCachedPickedPoint = pickedPoint
        #data = segmentation.segmentTableScene(pointCloudObj.polyData, pickedPoint)
        #vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')


    def onLocalPlaneFit():
        planePoints, normal = segmentation.applyLocalPlaneFit(pointCloudObj.polyData, pickedPoint, searchRadius=0.1, searchRadiusEnd=0.2)
        obj = vis.showPolyData(planePoints, 'local plane fit', color=[0,1,0])
        obj.setProperty('Point Size', 7)

        fields = segmentation.makePolyDataFields(obj.polyData)

        pose = transformUtils.poseFromTransform(fields.frame)
        desc = dict(classname='BoxAffordanceItem', Name='local plane', Dimensions=list(fields.dims), pose=pose)
        box = segmentation.affordanceManager.newAffordanceFromDescription(desc)

    def onOrientToMajorPlane():
        polyData, planeFrame = segmentation.orientToMajorPlane(pointCloudObj.polyData, pickedPoint=pickedPoint)
        pointCloudObj.setPolyData(polyData)


    def onDiskGlyph():
        result = segmentation.applyDiskGlyphs(pointCloudObj.polyData)
        obj = vis.showPolyData(result, 'disks', color=[0.8,0.8,0.8])
        om.setActiveObject(obj)
        pickedObj.setProperty('Visible', False)

    def onArrowGlyph():
        result = segmentation.applyArrowGlyphs(pointCloudObj.polyData)
        obj = vis.showPolyData(result, 'disks')

    def onSegmentationEditor():
        segmentationpanel.activateSegmentationMode(pointCloudObj.polyData)

    def addNewFrame():
        t = transformUtils.copyFrame(affordanceObj.getChildFrame().transform)
        t.PostMultiply()
        t.Translate(np.array(pickedPoint) - np.array(t.GetPosition()))
        newFrame = vis.showFrame(t, '%s frame %d' % (affordanceObj.getProperty('Name'), len(affordanceObj.children())), scale=0.2, parent=affordanceObj)
        affordanceObj.getChildFrame().getFrameSync().addFrame(newFrame, ignoreIncoming=True)

    def copyAffordance():
        desc = dict(affordanceObj.getDescription())
        del desc['uuid']
        desc['Name'] = desc['Name'] + ' copy'
        aff = robotSystem.affordanceManager.newAffordanceFromDescription(desc)
        aff.getChildFrame().setProperty('Edit', True)

    def onPromoteToAffordance():
        affObj = affordanceitems.MeshAffordanceItem.promotePolyDataItem(pickedObj)
        robotSystem.affordanceManager.registerAffordance(affObj)

    actions = [
      (None, None),
      ('Hide', onHide),
      ('Delete', onDelete),
      ('Select', onSelect)
      ]


    if affordanceObj:
        actions.extend([
            ('Copy affordance', copyAffordance),
            ('Add new frame', addNewFrame),
        ])

    elif type(pickedObj) == vis.PolyDataItem:
        actions.extend([
            ('Promote to Affordance', onPromoteToAffordance),
        ])

    if isGraspSeed(pickedObj):
        actions.extend([
            (None, None),
            ('Flip Side', flipHandSide),
            ('Flip Thumb', flipHandThumb),
        ])

    if reachFrame is not None:
        actions.extend([
            (None, None),
            ('Reach Left', onReachLeft),
            ('Reach Right', onReachRight),
            #('Spline Left', onSplineLeft),
            #('Spline Right', onSplineRight),
            ])

    if pointCloudObj:
        actions.extend([
            (None, None),
            ('Copy Pointcloud', onCopyPointCloud),
            ('Merge Pointcloud Into', onMergeIntoPointCloud),
            ('Segment Ground', onSegmentGround),
            ('Segment Table', onSegmentTableScene),
            ('Segment Drill Aligned', onSegmentDrillAlignedWithTable),
            ('Local Plane Fit', onLocalPlaneFit),
            ('Orient with Horizontal', onOrientToMajorPlane),
            ('Arrow Glyph', onArrowGlyph),
            ('Disk Glyph', onDiskGlyph),
            ('Cache Pick Point', onCachePickedPoint),
            (None, None),
            ('Open Segmentation Editor', onSegmentationEditor)
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

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self.onLeftMousePress(event)

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.RightButton:
            self.mouseStart = QtCore.QPoint(event.pos())

        elif event.type() == QtCore.QEvent.MouseMove:

            if self.mouseStart is not None:
                delta = QtCore.QPoint(event.pos()) - self.mouseStart
                if delta.manhattanLength() > 3:
                    self.mouseStart = None
            else:
                self.onMouseMove(event)

        elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.RightButton and self.mouseStart is not None:
            self.mouseStart = None
            self.onRightClick(event)

        elif event.type() == QtCore.QEvent.Wheel:
            self.onWheelEvent(event)

    def consumeEvent(self):
        self.eventFilter.setEventHandlerResult(True)

    def onWheelEvent(self, event):
        if neckDriver:
            neckDriver.onWheelDelta(event.delta())

    def onMouseMove(self, event):

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMouseMove(vis.mapMousePosition(self.view, event), event.modifiers())
            self.consumeEvent()


    def onLeftMousePress(self, event):
        if event.modifiers() == QtCore.Qt.ControlModifier:
            displayPoint = vis.mapMousePosition(self.view, event)
            if footstepsDriver:
                newWalkingGoal(displayPoint, self.view)
                self.consumeEvent()

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMousePress(vis.mapMousePosition(self.view, event), event.modifiers())
            self.consumeEvent()

    def onLeftDoubleClick(self, event):

        displayPoint = vis.mapMousePosition(self.view, event)

        useHorizontalWidget =  (event.modifiers() == QtCore.Qt.ShiftModifier)
        if toggleFootstepWidget(displayPoint, self.view, useHorizontalWidget):
            return

        if toggleFrameWidget(displayPoint, self.view):
            return

        if robotLinkSelector and robotLinkSelector.selectLink(displayPoint, self.view):
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
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Wheel)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


class KeyEventFilter(object):

    def __init__(self, view):
        self.view = view
        self.initEventFilter()

    def filterEvent(self, obj, event):

        consumed = False
        if event.type() == QtCore.QEvent.KeyPress and not event.isAutoRepeat():
            key = str(event.text()).lower()
            if key == 'f':
                zoomToPick(self.getCursorDisplayPosition(), self.view)
                consumed = True
            elif key == 'r':
                consumed = True
                if robotModel is not None:
                    resetCameraToRobot(self.view)
                else:
                    self.view.resetCamera()
                    self.view.render()
            elif key == 's':
                consumed = True
                if handFactory is not None:
                    side = 'left' if event.modifiers() != QtCore.Qt.ShiftModifier else 'right'
                    placeHandModel(self.getCursorDisplayPosition(), self.view, side)
            elif key == 'n':
                if neckDriver:
                    neckDriver.activateNeckControl()

            elif key in ['0', '1', '2', '3']:
                if neckDriver:
                    consumed = neckDriver.applyNeckPitchPreset(int(key))

            if key == '3':
                # block vtk keypress handler 3d mode
                consumed = True

        elif event.type() == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
            if str(event.text()).lower() == 'n':
                if neckDriver:
                    neckDriver.deactivateNeckControl()
        
        if event.type() == QtCore.QEvent.KeyPress and not consumed:
            consumed = frameupdater.handleKey(event)


        self.eventFilter.setEventHandlerResult(consumed)


    def getCursorDisplayPosition(self):
        cursorPos = self.view.mapFromGlobal(QtGui.QCursor.pos())
        return cursorPos.x(), self.view.height - cursorPos.y()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyRelease)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


class KeyPressLogCommander(object):

    def __init__(self, widget):
        self.widget = widget
        self.initEventFilter()
        self.commander = lcmUtils.LogPlayerCommander()

    def filterEvent(self, obj, event):

        if event.type() == QtCore.QEvent.KeyPress:
            key = str(event.text()).lower()
            consumed = True

            if key == 'p':
                self.commander.togglePlay()
            elif key == 'n':
                self.commander.step()
            elif key in ('+', '='):
                self.commander.faster()
            elif key in ('-', '_'):
                self.commander.slower()
            elif key == '[':
                self.commander.back()
            elif key == ']':
                self.commander.forward()
            else:
                consumed = False

            self.eventFilter.setEventHandlerResult(consumed)

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.widget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)



class ViewBehaviors(object):

    def __init__(self, view):
        self.view = view
        self.mouseEventFilter = ViewEventFilter(view)
        #self.logCommander = KeyPressLogCommander(view.vtkWidget())
        self.keyEventFilter = KeyEventFilter(view)

    @staticmethod
    def addRobotBehaviors(_robotSystem):
        global robotSystem, robotModel, handFactory, footstepsDriver, neckDriver, robotLinkSelector
        robotSystem = _robotSystem
        robotModel = robotSystem.robotStateModel
        handFactory = robotSystem.handFactory
        footstepsDriver = robotSystem.footstepsDriver
        neckDriver = robotSystem.neckDriver
        if app.getMainWindow() is not None:
            robotLinkSelector = RobotLinkSelector()
