import PythonQt
from PythonQt import QtCore, QtGui
import director.objectmodel as om
import director.visualization as vis
from director import affordanceitems
from director import lcmUtils
from director import callbacks
from director import cameracontrol
from director import splinewidget
from director import transformUtils
from director import teleoppanel
from director import footstepsdriverpanel
from director import applogic as app
from director import vtkAll as vtk
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import segmentationpanel
from director import segmentation
from director import segmentationroutines
from director.robotlinkselector import RobotLinkSelector
from director.vieweventfilter import ViewEventFilter
from director import viewbehaviors
import numpy as np
from . import ioUtils
import os
import re
import random
import colorsys

# todo: refactor these global variables
# several functions in this module depend on these global variables
# which are set by calling ViewBehaviors.addRobotBehaviors().
# These could be refactored to be members of a new behaviors class.
robotSystem = None
robotModel = None
handFactory = None
neckDriver = None
footstepsDriver = None
robotLinkSelector = None
lastRandomColor = 0.0


def resetCameraToRobot(view):
    t = robotModel.getLinkFrame('pelvis')
    if t is None:
        t = vtk.vtkTransform()

    focalPoint = [0.0, 0.0, 0.25]
    position = [-4.0, -2.0, 2.25]
    t.TransformPoint(focalPoint, focalPoint)
    t.TransformPoint(position, position)
    flyer = cameracontrol.Flyer(view)
    flyer.zoomTo(focalPoint, position)


def resetCameraToHeadView(view):

    head = robotModel.getLinkFrame('head')
    pelvis = robotModel.getLinkFrame('pelvis')

    viewDirection = np.array([1.0, 0.0, 0.0])
    pelvis.TransformVector(viewDirection, viewDirection)

    cameraPosition = np.array(head.GetPosition()) + 0.10 * viewDirection

    camera = view.camera()

    focalOffset = np.array(camera.GetFocalPoint()) - np.array(camera.GetPosition())
    focalOffset /= np.linalg.norm(focalOffset)

    camera.SetPosition(cameraPosition)
    camera.SetFocalPoint(cameraPosition + focalOffset*0.03)
    camera.SetViewUp([0, 0, 1])
    camera.SetViewAngle(90)
    view.render()


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

    pickedPointFields = vis.pickPoint(displayPoint, view, pickType='cells', tolerance=0.0)
    pickedPoint = pickedPointFields.pickedPoint
    prop = pickedPointFields.pickedProp

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

    if side == 'right':
        t.PreMultiply()
        t.RotateY(180)

    handObj, handFrame = handFactory.placeHandModelWithTransform(t, view, side=side, parent=obj)

    syncFrame = getChildFrame(obj)
    if syncFrame:
        handFrame.frameSync = vis.FrameSync()
        handFrame.frameSync.addFrame(handFrame, ignoreIncoming=True)
        handFrame.frameSync.addFrame(syncFrame)


def newWalkingGoal(displayPoint, view):

    footFrame = footstepsDriver.getFeetMidPoint(robotModel)

    worldPt1, worldPt2 = vis.getRayFromDisplayPoint(view, displayPoint)
    groundOrigin = footFrame.GetPosition()
    groundNormal = [0.0, 0.0, 1.0]
    selectedGroundPoint = [0.0, 0.0, 0.0]

    t = vtk.mutable(0.0)
    vtk.vtkPlane.IntersectWithLine(worldPt1, worldPt2, groundNormal, groundOrigin, t, selectedGroundPoint)

    walkingTarget = transformUtils.frameFromPositionAndRPY(selectedGroundPoint, np.array(footFrame.GetOrientation()))

    footstepsdriverpanel.panel.onNewWalkingGoal(walkingTarget)


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
        om.removeFromObjectModel(existingWidget)
        if previousStep == stepIndex:
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


def getObjectAsPointCloud(obj):
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


def getRobotActions(view, pickedObj, pickedPoint):

    reachFrame = getAsFrame(pickedObj)
    collisionParent = getCollisionParent(pickedObj)
    pointCloudObj = getObjectAsPointCloud(pickedObj)
    affordanceObj = pickedObj if isinstance(pickedObj, affordanceitems.AffordanceItem) else None

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

            polyData = handFactory.getNewHandPolyData(side)
            obj.setPolyData(polyData)

            handFrame = obj.children()[0]
            t = transformUtils.copyFrame(handFrame.transform)
            t.PreMultiply()
            t.RotateY(180)
            handFrame.copyFrame(t)

            objName = obj.getProperty('Name')
            frameName = handFrame.getProperty('Name')
            if side == 'left':
                obj.setProperty('Name', objName.replace("right", "left"))
                handFrame.setProperty('Name', frameName.replace("right", "left"))
            else:
                obj.setProperty('Name', objName.replace("left", "right"))
                handFrame.setProperty('Name', frameName.replace("left", "right"))
            obj._renderAllViews()

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

        #t = vtk.vtkTransform()
        #t.PostMultiply()
        #t.Translate(filterUtils.computeCentroid(polyData))
        #segmentation.makeMovable(obj, t)
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
        vis.showClusterObjects(data.clusters, parent='segmentation')
        segmentation.showTable(data.table, parent='segmentation')


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
        obj = vis.showPolyData(result, 'arrows')

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


    actions = []


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

    return actions


viewbehaviors.registerContextMenuActions(getRobotActions)


class RobotViewEventFilter(ViewEventFilter):

    def onMouseWheel(self, event):
        if neckDriver:
            neckDriver.onWheelDelta(event.delta())

    def onMouseMove(self, event):

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMouseMove(self.getMousePositionInView(event), event.modifiers())
            self.consumeEvent()

    def onLeftMousePress(self, event):
        if event.modifiers() == QtCore.Qt.ControlModifier:
            displayPoint = self.getMousePositionInView(event)
            if footstepsDriver:
                newWalkingGoal(displayPoint, self.view)
                self.consumeEvent()

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMousePress(self.getMousePositionInView(event), event.modifiers())
            self.consumeEvent()

    def onLeftDoubleClick(self, event):

        displayPoint = self.getMousePositionInView(event)

        useHorizontalWidget =  (event.modifiers() == QtCore.Qt.ShiftModifier)
        if toggleFootstepWidget(displayPoint, self.view, useHorizontalWidget):
            self.consumeEvent()
            return

        if robotLinkSelector and robotLinkSelector.selectLink(displayPoint, self.view):
            self.consumeEvent()
            return

    def onKeyPress(self, event):

        consumed = False

        key = str(event.text()).lower()

        if key == 'r':
            consumed = True
            if robotModel is not None:
                resetCameraToRobot(self.view)

        elif key == 's':
            if handFactory is not None:
                side = 'left' if event.modifiers() != QtCore.Qt.ShiftModifier else 'right'
                placeHandModel(self.getCursorDisplayPosition(), self.view, side)
        elif key == 'n':
            if neckDriver:
                neckDriver.activateNeckControl()

        elif key in ['0', '1', '2', '3']:
            if neckDriver:
                neckDriver.applyNeckPitchPreset(int(key))

        if consumed:
            self.consumeEvent()

    def onKeyRelease(self, event):
        if str(event.text()).lower() == 'n':
            if neckDriver:
                neckDriver.deactivateNeckControl()


class KeyPressLogCommander(ViewEventFilter):

    def __init__(self, view):
        ViewEventFilter.__init__(self, view)
        self.commander = lcmUtils.LogPlayerCommander()

    def onKeyPressRepeat(self, event):

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

        if consumed:
            self.consumeEvent()


class RobotViewBehaviors(object):

    def __init__(self, view, _robotSystem):
        self.view = view
        self.viewBehaviors = viewbehaviors.ViewBehaviors(view)
        self.logCommander = KeyPressLogCommander(view)
        self.robotViewBehaviors = RobotViewEventFilter(view)

        global robotSystem, robotModel, handFactory, footstepsDriver, neckDriver, robotLinkSelector

        robotSystem = _robotSystem
        robotModel = robotSystem.robotStateModel
        handFactory = robotSystem.handFactory
        footstepsDriver = robotSystem.footstepsDriver
        neckDriver = robotSystem.neckDriver
        if app.getMainWindow() is not None:
            robotLinkSelector = RobotLinkSelector()
