from ddapp import asynctaskqueue as atq
from ddapp import segmentation
from ddapp import visualization as vis
import ddapp.objectmodel as om
from ddapp import propertyset
from ddapp import pointpicker
from ddapp import planplayback
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp import ikplanner
from ddapp import callbacks
from ddapp import robotsystem
from ddapp import transformUtils
from ddapp import vtkNumpy as vnp
from ddapp.debugVis import DebugData
from ddapp.robotplanlistener import ManipulationPlanItem
from ddapp.footstepsdriver import FootstepPlanItem
import numpy as np
import copy
import pickle
import PythonQt
from PythonQt import QtCore, QtGui

import re
import inspect


robotSystem = None


def _splitCamelCase(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1 \2', name)


class AsyncTask(object):
    '''
    AsyncTask documentation.
    '''

    def __init__(self, **kwargs):
        self.statusMessage = ''
        self.properties = propertyset.PropertySet()
        self.properties.addProperty('Name', _splitCamelCase(self.__class__.__name__).lower())

        for cls in reversed(inspect.getmro(self.__class__)):
            if hasattr(cls, 'getDefaultProperties'):
                cls.getDefaultProperties(self.properties)

        for name, value in kwargs.iteritems():
            self.properties.setProperty(_splitCamelCase(name).capitalize(), value)


    def __call__(self):
        return self.run()

    def stop(self):
        pass

    def run(self):
        pass

    @staticmethod
    def fail(reason):
        print 'task failed:', reason
        raise atq.AsyncTaskQueue.PauseException()

    def copy(self):
        return copy.deepcopy(self)


class PrintTask(AsyncTask):
    '''
    Name: Print Task
    Short Description: prints a string
    Description:

    This task prints a message string.
    '''

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', '<empty message>')

    def run(self):
      print self.properties.message


class UserPromptTask(AsyncTask):

    promptsEnabled = True


    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', 'continue?')
        properties.addProperty('Always', False)

    def showDialog(self):

        self.d = QtGui.QDialog()

        buttons = QtGui.QDialogButtonBox()
        buttons.addButton('Yes', QtGui.QDialogButtonBox.AcceptRole)
        buttons.addButton('No', QtGui.QDialogButtonBox.RejectRole)
        buttons.connect('accepted()', self.d.accept)
        buttons.connect('rejected()', self.d.reject)

        l = QtGui.QVBoxLayout(self.d)
        l.addWidget(QtGui.QLabel(self.properties.message))
        l.addWidget(buttons)

        self.d.setAttribute(QtCore.Qt.WA_QuitOnClose, False)
        self.d.show()
        self.d.raise_()
        self.d.connect('accepted()', self.onYes)
        self.d.connect('rejected()', self.onNo)

    def onYes(self):
        self.result = True

    def onNo(self):
        self.result = False

    def run(self):

        if not self.promptsEnabled and not self.properties.getProperty('Always'):
            return

        self.showDialog()

        self.result = None
        while self.result is None:
            yield

        if not self.result:
            raise atq.AsyncTaskQueue.PauseException()


class DelayTask(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Delay time', 1.0, attributes=propertyset.PropertyAttributes(minimum=0.0, maximum=1e4, singleStep=0.1))

    def run(self):

        delayTime = self.properties.getProperty('Delay time')
        t = SimpleTimer()

        while True:

            elapsed = t.elapsed()
            if elapsed >= delayTime:
                break

            self.statusMessage = 'Waiting %.1f seconds' % (delayTime - elapsed)
            yield


class PauseTask(AsyncTask):

    def run(self):
        raise atq.AsyncTaskQueue.PauseException()


class QuitTask(AsyncTask):

    def run(self):
        QtCore.QCoreApplication.instance().quit()


class WaitForMultisenseLidar(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Number of sweeps', 1, attributes=propertyset.PropertyAttributes(minimum=0, maximum=100))

    def run(self):

        self.multisenseDriver = robotSystem.multisenseDriver
        currentRevolution = self.multisenseDriver.displayedRevolution
        desiredRevolution = currentRevolution + self.properties.getProperty('Number of sweeps')

        while self.multisenseDriver.displayedRevolution < desiredRevolution:
            self.statusMessage = 'Waiting for multisense sweep'
            yield


class SnapshotPointcloud(AsyncTask):

    def run(self):
        polyData = self.getPointCloud()
        om.removeFromObjectModel(om.findObjectByName('pointcloud snapshot'))
        vis.showPolyData(polyData, 'pointcloud snapshot', parent='segmentation', visible=False)


class SnapshotMultisensePointcloud(SnapshotPointcloud):

    def getPointCloud(self):
        return segmentation.getCurrentRevolutionData()

class SnapshotSelectedPointcloud(SnapshotPointcloud):

    def getPointCloud(self):
        obj = om.getActiveObject()
        if obj and obj.getProperty('Name') == 'Multisense':
            return SnapshotMultisensePointcloud().getPointCloud()
        elif obj and obj.getProperty('Name') == 'stereo point cloud':
            return SnapshotStereoPointcloud().getPointCloud()
        elif obj and hasattr(obj, 'polyData'):
            return obj.polyData
        else:
            self.fail('no pointcloud is selected')


class SnapshotStereoPointcloud(SnapshotPointcloud):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Remove Outliers', False)

    def getPointCloud(self):
        return segmentation.getDisparityPointCloud(decimation=1, removeOutliers=self.getProperty('Remove Outliers'))


class PointCloudAlgorithmBase(AsyncTask):


    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Use pointcloud snapshot', True)

    def getPointCloud(self):
        if self.properties.getProperty('Use pointcloud snapshot'):
            obj = om.findObjectByName('pointcloud snapshot')
            if obj is None:
                self.fail('pointcloud snapshot not found')
            if not obj.polyData.GetNumberOfPoints():
                self.fail('input pointcloud is empty')
            return obj.polyData
        else:
            return SnapshotSelectedPointcloud().getPointCloud()


class FitDrill(PointCloudAlgorithmBase):

    def run(self):
        polyData = self.getPointCloud()
        segmentation.findAndFitDrillBarrel(polyData)


class FindDrillOnTableByAnnotation(PointCloudAlgorithmBase):

    def getAnnotationInputPoint(self):
        obj = om.findObjectByName('drill on table annotation')
        if obj is None:
            self.fail('user annotation not found')
        return obj.annotationPoints[0]

    def run(self):
        point = self.getAnnotationInputPoint()
        polyData = self.getPointCloud()
        segmentation.segmentTableThenFindDrills(polyData, point)


class FindRotaryDrillByAnnotation(PointCloudAlgorithmBase):

    def getAnnotationInputPoint(self):
        obj = om.findObjectByName('rotary drill annotation')
        if obj is None:
            self.fail('user annotation not found')
        return obj.annotationPoints[0]

    def run(self):
        point = self.getAnnotationInputPoint()
        polyData = self.getPointCloud()
        #segmentation.segmentDrillAuto(point, polyData)
        om.removeFromObjectModel(om.findObjectByName('drill'))
        segmentation.segmentDrillAlignedWithTable(point, polyData)


class WaitForAtlasBehavior(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Behavior name', '')

    def run(self):
        behaviorName = self.properties.getProperty('Behavior name')
        assert behaviorName in robotSystem.atlasDriver.getBehaviorMap().values()
        while robotSystem.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


class WaitForWalkExecution(AsyncTask):

    def run(self):
        # wait for controller status WALK
        while robotSystem.atlasDriver.getControllerStatus() != 2:
            yield

        # wait for controller status STAND
        while robotSystem.atlasDriver.getControllerStatus() != 1:
            yield

class WaitForWalkExecutionBDI(AsyncTask):

    def run(self):
        while robotSystem.atlasDriver.getCurrentBehaviorName() != 'step':
            yield

        while robotSystem.atlasDriver.getCurrentBehaviorName() != 'stand':
            yield

class WaitForManipulationPlanExecution(AsyncTask):

    def run(self):
        plan = robotSystem.manipPlanner.committedPlans[-1]
        delayTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)

        t = SimpleTimer()

        while True:

            elapsed = t.elapsed()
            if elapsed >= delayTime:
                break

            self.statusMessage = 'Waiting for execution: %.1f seconds' % (delayTime - elapsed)
            yield


class UserSelectAffordanceCandidate(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Candidate name prefix', '')
        properties.addProperty('New name', '')
        properties.addProperty('Delete candidates', True)

    def getCandidates(self):
        namePrefix = self.properties.getProperty('Candidate name prefix')
        matchStr = '^%s [0-9]+$' % namePrefix
        return [obj for obj in om.getObjects() if re.match(matchStr, obj.getProperty('Name'))]

    def selectCandidate(self, selectedObj, candidates):

        if self.properties.getProperty('Delete candidates'):
            for obj in candidates:
                if obj != selectedObj:
                    om.removeFromObjectModel(obj)

        newName = self.properties.getProperty('New name')
        if newName:
            oldName = selectedObj.getProperty('Name')
            selectedObj.setProperty('Name', newName)
            for child in selectedObj.children():
                child.setProperty('Name', child.getProperty('Name').replace(oldName, newName))

    def run(self):

        candidates = self.getCandidates()
        if not candidates:
            self.fail('no affordance candidates found')

        om.clearSelection()

        self.statusMessage = 'Please select affordance candidate: %s' % self.properties.getProperty('Candidate name prefix')

        while True:
            obj = om.getActiveObject()
            if obj and obj in candidates:
                break
            else:
                yield

        self.selectCandidate(obj, candidates)


class TransformFrame(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Frame input name', '')
        properties.addProperty('Frame output name', '')

        properties.addProperty('Translation', [0.0, 0.0, 0.0], attributes=propertyset.PropertyAttributes(decimals=3, minimum=-1e5, maximum=1e5))
        properties.addProperty('Rotation', [0.0, 0.0, 0.0], attributes=propertyset.PropertyAttributes(decimals=3, minimum=-360, maximum=360))


    def getInputFrame(self):
        name = self.properties.getProperty('Frame input name')
        frame = om.findObjectByName(name)
        if not isinstance(frame, vis.FrameItem):
            fail('frame not found: %s' % name)
        return frame

    def run(self):
        inputFrame = self.getInputFrame()

        translation = self.properties.getProperty('Translation')
        rpy = self.properties.getProperty('Rotation')

        offset = transformUtils.frameFromPositionAndRPY(translation, rpy)
        offset.PostMultiply()
        offset.Concatenate(transformUtils.copyFrame(inputFrame.transform))

        outputFrame = vis.updateFrame(offset, self.properties.getProperty('Frame output name'), scale=0.2, parent=inputFrame.parent())

        if not hasattr(inputFrame, 'frameSync'):
            inputFrame.frameSync = vis.FrameSync()
            inputFrame.frameSync.addFrame(inputFrame)
        inputFrame.frameSync.addFrame(outputFrame, ignoreIncoming=True)


class ComputeRobotFootFrame(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Pose name', 'EST_ROBOT_STATE')
        properties.addProperty('Frame output name', 'robot foot frame')

    def run(self):
        poseName = self.properties.getProperty('Pose name')
        if poseName == 'EST_ROBOT_STATE':
            pose = robotSystem.robotStateJointController.q.copy()
        else:
            pose = robotSystem.ikPlanner.jointController.getPose(poseName)

        robotModel = robotSystem.ikPlanner.getRobotModelAtPose(pose)
        footFrame = robotSystem.footstepsDriver.getFeetMidPoint(robotModel)
        vis.updateFrame(footFrame, self.properties.getProperty('Frame output name'), scale=0.2)


class ProjectAffordanceToGround(PointCloudAlgorithmBase):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Affordance name', '')
        properties.addProperty('Ground frame name', '')
        properties.addProperty('Frame output name', '')

    def getSelectedAffordance(self):

        affordanceName = self.properties.getProperty('Affordance name')
        if affordanceName:
            obj = om.findObjectByName(affordanceName)
            if not obj:
                self.fail('could not find affordance: %s' % affordanceName)

        else:
            obj = om.getActiveObject()
            if obj is None:
                self.fail('no affordance is selected')

        try:
            frame = obj.getChildFrame()
        except AttributeError:
            frame = None

        if frame is None:
            self.fail('affordance does not have a frame')
        return obj

    def getGroundFrame(self):

        frame = om.findObjectByName(self.properties.getProperty('Ground frame name'))
        if not frame:
            self.fail('could not find ground frame')
        return frame


    def run(self):
        aff = self.getSelectedAffordance()
        affFrame = aff.getChildFrame().transform

        groundFrame = self.getGroundFrame().transform

        projectedXYZ = np.hstack([affFrame.GetPosition()[0:2], groundFrame.GetPosition()[2]])

        result = transformUtils.copyFrame(affFrame)
        result.Translate(projectedXYZ - np.array(result.GetPosition()))

        outputName = self.properties.getProperty('Frame output name')
        outputName = outputName or '%s ground frame' % aff.getProperty('Name')

        vis.updateFrame(result, outputName, scale=0.2)


class UserAnnotatePointCloud(PointCloudAlgorithmBase):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Annotation name', 'user annotation')
        properties.addProperty('Number of points', 2)

    def clearPicker(self):
        self.picker.stop()
        self.picker.clear()
        self.picker = None

    def onAnnotationAborted(self):
        om.removeFromObjectModel(self.picker.annotationObj)
        self.aborted = True
        self.clearPicker()

    def onAnnotationComplete(self, *pts):
        self.picker.annotationObj.annotationPoints = pts
        self.clearPicker()

    def startAnnotationPicker(self):

        view = robotSystem.view
        polyData = self.getPointCloud()
        self.picker = pointpicker.PointPicker(view, numberOfPoints=self.properties.getProperty('Number of points'), drawLines=True, callback=self.onAnnotationComplete, abortCallback=self.onAnnotationAborted)
        self.picker.annotationName = self.properties.getProperty('Annotation name')
        self.picker.annotationFolder = 'annotations'
        self.aborted = False
        self.picker.start()


    def run(self):
        self.startAnnotationPicker()
        self.statusMessage = 'Annotate point cloud (shift+click) to select points'
        while self.picker is not None:
            yield

        if self.aborted:
            self.fail('user abort')


class FindHorizontalSurfaces(PointCloudAlgorithmBase):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Normal estimation search radius', 0.03, attributes=propertyset.PropertyAttributes(decimals=3, minimum=0.0, maximum=100))
        properties.addProperty('Cluster tolerance', 0.02, attributes=propertyset.PropertyAttributes(decimals=3, minimum=0.0, maximum=10))
        properties.addProperty('Distance to plane threshold', 0.01, attributes=propertyset.PropertyAttributes(decimals=4, minimum=0.0, maximum=1))
        properties.addProperty('Normals dot up range', [0.9, 1.0], attributes=propertyset.PropertyAttributes(decimals=2, minimum=0.0, maximum=1))


    def run(self):
        polyData = self.getPointCloud()
        segmentation.findHorizontalSurfaces(polyData,
          removeGroundFirst=True,
          showClusters=True,
          normalEstimationSearchRadius=self.properties.getProperty('Normal estimation search radius'),
          clusterTolerance=self.properties.getProperty('Cluster tolerance'),
          distanceToPlaneThreshold=self.properties.getProperty('Distance to plane threshold'),
          normalsDotUpRange=self.properties.getProperty('Normals dot up range')
          )



class PublishAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Affordance name', 'affordance')

    def getAffordance(self):
        obj = om.findObjectByName(self.properties.getProperty('Affordance name'))
        assert obj and hasattr(obj, 'publish')
        return obj

    def run(self):
        aff = self.getAffordance()
        aff.publish()



class CloseHand(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Side', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return robotSystem.lHandDriver if side == 'left' else robotSystem.rHandDriver

    def run(self):
        side = self.properties.getPropertyEnumValue('Side').lower()
        self.getHandDriver(side).sendClose()


class OpenHand(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Side', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return robotSystem.lHandDriver if side == 'left' else robotSystem.rHandDriver

    def run(self):
        side = self.properties.getPropertyEnumValue('Side').lower()
        self.getHandDriver(side).sendOpen()


class CommitFootstepPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Plan name', '')

    def run(self):

        planName = self.properties.getProperty('Plan name')
        plan = om.findObjectByName(planName)
        if not isinstance(plan, FootstepPlanItem):
            fail('could not find footstep plan')
        plan = plan.plan

        robotSystem.footstepsDriver.commitFootstepPlan(plan)


class CommitManipulationPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Plan name', '')

    def run(self):

        planName = self.properties.getProperty('Plan name')
        plan = om.findObjectByName(planName)
        if not isinstance(plan, ManipulationPlanItem):
            fail('could not find manipulation plan')
        plan = plan.plan

        robotSystem.manipPlanner.commitManipPlan(plan)


class RequestWalkingPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Start pose name', 'EST_ROBOT_STATE')
        properties.addProperty('Footstep plan name', '')


    def run(self):
        poseName = self.properties.getProperty('Start pose name')
        if poseName == 'EST_ROBOT_STATE':
            pose = robotSystem.robotStateJointController.q.copy()
        else:
            pose = robotSystem.ikPlanner.jointController.getPose(poseName)

        planName = self.properties.getProperty('Footstep plan name')
        plan = om.findObjectByName(planName)
        if not isinstance(plan, FootstepPlanItem):
            self.fail('could not find footstep plan: %s' % planName)
        plan = plan.plan

        robotSystem.footstepsDriver.sendWalkingPlanRequest(plan, pose, waitForResponse=True)


def _addPlanItem(plan, name, itemClass):
        item = itemClass(name)
        item.plan = plan
        om.removeFromObjectModel(om.findObjectByName(name))
        om.addToObjectModel(item, om.getOrCreateContainer('segmentation'))
        return item


class RequestFootstepPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Stance frame name', 'stance frame')
        properties.addProperty('Start pose name', 'EST_ROBOT_STATE')

    def run(self):
        poseName = self.properties.getProperty('Start pose name')
        if poseName == 'EST_ROBOT_STATE':
            pose = robotSystem.robotStateJointController.q.copy()
        else:
            pose = robotSystem.ikPlanner.jointController.getPose(poseName)

        goalFrame = om.findObjectByName(self.properties.getProperty('Stance frame name')).transform

        request = robotSystem.footstepsDriver.constructFootstepPlanRequest(pose, goalFrame)
        footstepPlan = robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)

        _addPlanItem(footstepPlan, self.properties.getProperty('Stance frame name') + ' footstep plan', FootstepPlanItem)


class PlanPostureGoal(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Posture group', 'General')
        properties.addProperty('Posture name', 'arm up pregrasp')
        properties.addProperty('Side', 1, attributes=om.PropertyAttributes(enumNames=['Default', 'Left', 'Right']))


    def run(self):
        startPosture = robotSystem.robotStateJointController.q.copy()
        side = [None, 'left', 'right'][self.properties.getProperty('Side')]

        pose = robotSystem.ikPlanner.getMergedPostureFromDatabase(startPosture, self.properties.getProperty('Posture group'), self.properties.getProperty('Posture name'), side)
        plan = robotSystem.ikPlanner.computePostureGoal(startPosture, pose)

        _addPlanItem(plan, self.properties.getProperty('Posture name') + ' posture plan', ManipulationPlanItem)


class PlanStandPosture(AsyncTask):

    def run(self):
        startPosture = robotSystem.robotStateJointController.q.copy()
        plan = robotSystem.ikPlanner.computeStandPlan(startPosture)
        _addPlanItem(plan, 'stand pose plan', ManipulationPlanItem)


class PlanNominalPosture(AsyncTask):

    def run(self):
        startPosture = robotSystem.robotStateJointController.q.copy()
        plan = robotSystem.ikPlanner.computeNominalPlan(startPosture)
        _addPlanItem(plan, 'nominal pose plan', ManipulationPlanItem)


class PlanReachToFrame(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Frame input name', '')
        properties.addProperty('Side', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))


    def getInputFrame(self):
        name = self.properties.getProperty('Frame input name')
        frame = om.findObjectByName(name)
        if not isinstance(frame, vis.FrameItem):
            fail('frame not found: %s' % name)
        return frame


    def run(self):

        side = self.properties.getPropertyEnumValue('Side').lower()

        startPose = robotSystem.robotStateJointController.q.copy()

        targetFrame = self.getInputFrame()

        constraintSet = robotSystem.ikPlanner.planEndEffectorGoal(startPose, side, targetFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        _addPlanItem(plan, '%s reach plan' % targetFrame.getProperty('Name'), ManipulationPlanItem)



class FitWallFrameFromAnnotation(PointCloudAlgorithmBase):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Annotation input name', '')


    def getAnnotationInput(self):
        obj = om.findObjectByName(self.properties.getProperty('Annotation input name'))
        if obj is None:
            self.fail('user annotation not found')
        return obj


    def run(self):

        polyData = self.getPointCloud()
        annotation = self.getAnnotationInput()
        annotationPoint = annotation.annotationPoints[0]
        planePoints, normal = segmentation.applyLocalPlaneFit(polyData, annotationPoint, searchRadius=0.1, searchRadiusEnd=0.2)
        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()

        if np.dot(normal, viewDirection) < 0:
            normal = -normal

        xaxis = normal
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)
        yaxis /= np.linalg.norm(yaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(annotationPoint)

        polyData = annotation.polyData
        polyData = segmentation.transformPolyData(polyData, t.GetLinearInverse())

        annotation.setProperty('Visible', False)
        om.removeFromObjectModel(om.findObjectByName('wall'))
        obj = vis.showPolyData(polyData, 'wall')
        obj.actor.SetUserTransform(t)
        vis.showFrame(t, 'wall frame', scale=0.2, parent=obj)


class SpawnValveAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):

        properties.addProperty('Radius', 0.195, attributes=om.PropertyAttributes(decimals=4, minimum=0, maximum=10))
        properties.addProperty('Position', [0.7, -0.22, 1.21], attributes=om.PropertyAttributes(decimals=3, minimum=-1e4, maximum=1e4))
        properties.addProperty('Rotation',  [180, -90, 16], attributes=om.PropertyAttributes(decimals=2, minimum=-360, maximum=360))

    def getGroundFrame(self):
        robotModel = robotSystem.robotStateModel
        return robotSystem.footstepsDriver.getFeetMidPoint(robotModel)

    def computeValveFrame(self):
        position = self.properties.getProperty('Position')
        rpy = self.properties.getProperty('Rotation')
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.getGroundFrame())
        return t

    def run(self):

        radius = self.properties.getProperty('Radius')
        zwidth = 0.03

        valveFrame = self.computeValveFrame()

        folder = om.getOrCreateContainer('affordances')
        z = DebugData()
        z.addLine ( np.array([0, 0, -0.0254]) , np.array([0, 0, 0.0254]), radius=radius)
        valveMesh = z.getPolyData()

        om.removeFromObjectModel(om.findObjectByName('valve'))
        valveAffordance = vis.showPolyData(valveMesh, 'valve', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder, alpha=0.3)
        valveAffordance.actor.SetUserTransform(valveFrame)
        valveFrame = vis.showFrame(valveFrame, 'valve frame', parent=self.valveAffordance, visible=False, scale=0.2)

        params = dict(radius=radius, length=zwidth, xwidth=radius, ywidth=radius, zwidth=zwidth,
                      otdf_type='steering_cyl', friendly_name='valve')
        self.valveAffordance.setAffordanceParams(params)
        self.valveAffordance.updateParamsFromActorTransform()


class PlanGazeTrajectory(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Target frame name', '')
        properties.addProperty('Annotation input name', '')
        properties.addProperty('Side', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))

        properties.addProperty('Cone threshold degrees', 5.0, attributes=om.PropertyAttributes(decimals=1, minimum=0, maximum=360))
        properties.addProperty('Palm offset', 0.0, attributes=om.PropertyAttributes(decimals=3, minimum=-1e4, maximum=1e4))


    def getAnnotationInputPoints(self):
        obj = om.findObjectByName(self.properties.getProperty('Annotation input name'))
        if obj is None:
            self.fail('user annotation not found')
        return obj.annotationPoints


    def appendPositionConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)

    def initGazeConstraintSet(self, goalFrame):

        # create constraint set

        startPose = robotSystem.robotStateJointController.q.copy()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=True, lockBack=False, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        # add gaze constraint
        self.graspToHandLinkFrame = self.ikPlanner.newPalmOffsetGraspToHandFrame(self.graspingHand, self.properties.getProperty('Palm offset'))
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame, coneThresholdDegrees=self.properties.getProperty('Cone threshold degrees'))
        self.constraintSet.constraints.insert(0, gazeConstraint)


    def getGazeTargetFrame(self):
        frame = om.findObjectByName(self.properties.getProperty('Target frame name'))
        if not frame:
            self.fail('could not find ground frame')
        return frame

    def run(self):

        self.ikPlanner = robotSystem.ikPlanner
        side = self.properties.getPropertyEnumValue('Side').lower()
        self.graspingHand = side

        targetPoints = self.getAnnotationInputPoints()
        gazeTargetFrame = self.getGazeTargetFrame()

        self.initGazeConstraintSet(gazeTargetFrame)

        numberOfSamples = len(targetPoints)

        for i in xrange(numberOfSamples):
            targetPos = targetPoints[i]
            targetFrame = transformUtils.copyFrame(gazeTargetFrame.transform)
            targetFrame.Translate(targetPos - np.array(targetFrame.GetPosition()))
            self.appendPositionConstraintForTargetFrame(targetFrame, i+1)

        gazeConstraint = self.constraintSet.constraints[0]
        assert isinstance(gazeConstraint, ikplanner.ik.WorldGazeDirConstraint)
        gazeConstraint.tspan = [1.0, numberOfSamples]


        plan = self.constraintSet.runIkTraj()

        _addPlanItem(plan, '%s gaze plan' % gazeTargetFrame.getProperty('Name'), ManipulationPlanItem)

