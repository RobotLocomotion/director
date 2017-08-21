from director import asynctaskqueue as atq
from director import segmentation
from director import visualization as vis
import director.objectmodel as om
from director import propertyset
from director import pointpicker
from director import planplayback
from director.timercallback import TimerCallback
from director.simpletimer import SimpleTimer
from director import ikplanner
from director import callbacks
from director import robotsystem
from director import transformUtils
from director import affordanceitems
from director import vtkNumpy as vnp
from director.debugVis import DebugData
from director import vtkAll as vtk
from director import lcmUtils
import numpy as np
import copy
import pickle
import PythonQt
from PythonQt import QtCore, QtGui

import re
import inspect

try:
    import drc as lcmdrc
    HAVE_DRC_MESSAGES = True
except ImportError:
    HAVE_DRC_MESSAGES = False


robotSystem = None


class ManipulationPlanItem(om.ObjectModelItem):
    pass


class FootstepPlanItem(om.ObjectModelItem):
    pass


class WalkingPlanItem(om.ObjectModelItem):
    pass


def _splitCamelCase(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1 \2', name)


class AsyncTask(object):
    '''
    AsyncTask documentation.
    '''

    def __init__(self, **kwargs):
        self.statusMessage = ''
        self.failReason = ''
        self.properties = propertyset.PropertySet()
        self.properties.addProperty('Name', _splitCamelCase(self.__class__.__name__).lower())

        for cls in reversed(inspect.getmro(self.__class__)):
            if hasattr(cls, 'getDefaultProperties'):
                cls.getDefaultProperties(self.properties)

        for name, value in kwargs.items():
            self.properties.setProperty(_splitCamelCase(name).capitalize(), value)


    def __call__(self):
        return self.run()

    def stop(self):
        pass

    def run(self):
        pass

    def fail(self, reason):
        self.failReason = reason
        raise atq.AsyncTaskQueue.FailException(reason)

    def copy(self):
        return copy.deepcopy(self)


class PrintTask(AsyncTask):
    '''
    Name: Print Task
    Short Description: prints a string
    Description:

    This task prints a message string.
    '''

    printFunction = None

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', '<empty message>')

    def run(self):
        if self.printFunction:
            self.printFunction(self.properties.message)
        else:
            print(self.properties.message)


class CallbackTask(AsyncTask):

    def __init__(self, callback=None, **kwargs):
        AsyncTask.__init__(self, **kwargs)
        self.callback = callback

    def run(self):
        if self.callback:
            yield self.callback()


class ExceptionTask(AsyncTask):


    def run(self):
        raise Exception('Task exception')


class UserPromptTask(AsyncTask):

    promptsEnabled = True
    promptFunction = None


    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', 'continue?')
        properties.addProperty('Always', False)

    def showUserPrompt(self):
        if self.promptFunction:
            self.promptFunction(self, self.properties.message)
        else:
            self.showDialog()

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
        self.d.connect('accepted()', self.accept)
        self.d.connect('rejected()', self.reject)

    def accept(self):
        self.result = True

    def reject(self):
        self.result = False

    def run(self):

        if not self.promptsEnabled and not self.properties.getProperty('Always'):
            return

        self.result = None

        self.showUserPrompt()

        while self.result is None:
            yield

        if not self.result:
            raise atq.AsyncTaskQueue.PauseException()

class CheckPlanInfo(UserPromptTask):

    @staticmethod
    def getDefaultProperties(properties):
        UserPromptTask.getDefaultProperties(properties)
        properties.setProperty('Message', 'Plan is invalid. Do you want to accept it anyway?')

    def run(self):
        if robotSystem.ikPlanner.lastManipPlan and max(robotSystem.ikPlanner.lastManipPlan.plan_info) <= 10 and min(robotSystem.ikPlanner.lastManipPlan.plan_info) >= 0:
            return
        else:
            return UserPromptTask.run(self)


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
        assert behaviorName in list(robotSystem.atlasDriver.getBehaviorMap().values())
        while robotSystem.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield

class WaitForWalkExecutionBDI(AsyncTask):

    def run(self):

        self.statusMessage = 'Waiting for BDI walking to begin...'
        while robotSystem.atlasDriver.getCurrentBehaviorName() != 'step':
            yield

        self.statusMessage = 'Waiting for BDI walk execution...'
        while robotSystem.atlasDriver.getCurrentBehaviorName() != 'stand':
            yield

class WaitForPlanExecution(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Timeout', 5.0, attributes=propertyset.PropertyAttributes(minimum=0.0, maximum=1e4, singleStep=0.1, decimals=2))

    def promptUserForPlanRecommit(self):
        prompt = UserPromptTask(message='Plan appears dropped. Recommit?')
        return prompt.run()

    def run(self):

        def getMsg():
            return robotSystem.atlasDriver.lastControllerStatusMessage

        def isExecuting():
            return getMsg().execution_status == lcmdrc.plan_status_t.EXECUTION_STATUS_EXECUTING

        # wait for first status message
        while not getMsg():
            yield

        if isExecuting():
            raise Exception('error, invoked during plan execution and cannot guarantee safety.')

        t = SimpleTimer()
        lastPlanStartTime = getMsg().last_plan_start_utime

        # wait for next plan to begin
        self.statusMessage = 'Waiting for %s to begin...' % self.getTypeLabel()
        while getMsg().last_plan_start_utime == lastPlanStartTime:

            if t.elapsed() > self.properties.getProperty('Timeout'):
                yield self.promptUserForPlanRecommit()
                t.reset()
                self.recommitPlan()
            else:
                yield

        # wait for execution
        self.statusMessage = 'Waiting for %s execution...' % self.getTypeLabel()
        while getMsg().execution_status == lcmdrc.plan_status_t.EXECUTION_STATUS_EXECUTING:
            if getMsg().plan_type != self.getType():
                raise Exception('error, unexpected execution plan type: %s' % getMsg().plan_type)
            yield

        self.statusMessage = 'Waiting for recent robot state...'
        while robotSystem.robotStateJointController.lastRobotStateMessage.utime < getMsg().last_plan_start_utime:
            yield


class WaitForManipulationPlanExecution(WaitForPlanExecution):

    def getType(self):
        return lcmdrc.plan_status_t.MANIPULATING

    def getTypeLabel(self):
        return 'manipulation'

    def recommitPlan(self):
        lastPlan = robotSystem.manipPlanner.committedPlans.pop()
        robotSystem.manipPlanner.commitManipPlan(lastPlan)

class WaitForWalkExecution(WaitForPlanExecution):

    def getType(self):
        return lcmdrc.plan_status_t.WALKING

    def getTypeLabel(self):
        return 'walking'

    def recommitPlan(self):
        lastPlan = robotSystem.footstepsDriver.committedPlans.pop()
        robotSystem.footstepsDriver.commitFootstepPlan(lastPlan)

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
            selectedObj.rename(newName)

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
            self.fail('frame not found: %s' % name)
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



class FindAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Affordance name', '')

    def run(self):
        affordanceName = self.properties.getProperty('Affordance name')
        obj = om.findObjectByName(affordanceName)
        if not obj:
            self.fail('could not find affordance: %s' % affordanceName)



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

        self.picker.pickType = 'points' if polyData.GetNumberOfCells() == polyData.GetNumberOfVerts() else 'render'

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
        properties.addProperty('Min cluster size', 150, attributes=propertyset.PropertyAttributes(minimum=3, maximum=1e6))
        properties.addProperty('Distance to plane threshold', 0.01, attributes=propertyset.PropertyAttributes(decimals=4, minimum=0.0, maximum=1))
        properties.addProperty('Normals dot up range', [0.9, 1.0], attributes=propertyset.PropertyAttributes(decimals=2, minimum=0.0, maximum=1))


    def run(self):
        polyData = self.getPointCloud()
        segmentation.findHorizontalSurfaces(polyData,
          removeGroundFirst=True,
          showClusters=True,
          normalEstimationSearchRadius=self.properties.getProperty('Normal estimation search radius'),
          clusterTolerance=self.properties.getProperty('Cluster tolerance'),
          minClusterSize=self.properties.getProperty('Min cluster size'),
          distanceToPlaneThreshold=self.properties.getProperty('Distance to plane threshold'),
          normalsDotUpRange=self.properties.getProperty('Normals dot up range')
          )


class SetNeckPitch(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Angle', 0, attributes=om.PropertyAttributes(minimum=-35, maximum=90))

    def run(self):
        robotSystem.neckDriver.setNeckPitch(self.properties.getProperty('Angle'))

class SetArmsPosition(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Posture group', 'General')
        properties.addProperty('Posture name', 'handsdown both')

    def run(self):
        startPosture = robotSystem.robotStateJointController.q.copy()
        side = None

        pose = robotSystem.ikPlanner.getMergedPostureFromDatabase(startPosture, self.properties.getProperty('Posture group'), self.properties.getProperty('Posture name'), side)
        plan = robotSystem.ikPlanner.computePostureGoal(startPosture, pose)

        #_addPlanItem(plan, self.properties.getProperty('Posture name') + ' posture plan', ManipulationPlanItem)

class CloseHand(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Side', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        properties.addProperty('Mode', 0, attributes=om.PropertyAttributes(enumNames=['Basic', 'Pinch']))
        properties.addProperty('Amount', 100, attributes=propertyset.PropertyAttributes(minimum=0, maximum=100))
        properties.addProperty('Check status', False)

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return robotSystem.lHandDriver if side == 'left' else robotSystem.rHandDriver

    def run(self):
        side = self.properties.getPropertyEnumValue('Side').lower()
        self.getHandDriver(side).sendCustom(self.properties.getProperty('Amount'), 100, 100, self.properties.getProperty('Mode'))

        if self.properties.getProperty('Check status'):
            WaitForGraspingState(actionName='Grasp').run()


class OpenHand(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Side', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        properties.addProperty('Mode', 0, attributes=om.PropertyAttributes(enumNames=['Basic', 'Pinch']))
        properties.addProperty('Amount', 100, attributes=propertyset.PropertyAttributes(minimum=0, maximum=100))
        properties.addProperty('Check status', False)

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return robotSystem.lHandDriver if side == 'left' else robotSystem.rHandDriver

    def run(self):
        side = self.properties.getPropertyEnumValue('Side').lower()
        self.getHandDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(100-self.properties.getProperty('Amount'), 100, 100, self.properties.getProperty('Mode'))

        if self.properties.getProperty('Check status'):
            WaitForGraspingState(actionName='Open').run()


class WaitForGraspingState(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Channel name', 'GRASPING_STATE')
        properties.addProperty('Action name', 0, attributes=om.PropertyAttributes(enumNames=['Open', 'Grasp']))
        # TODO: properties for timeout, responseMessageClass, expectedResponse

    def run(self):
        responseMessageClass = lcmdrc.boolean_t
        grasping_state = lcmUtils.MessageResponseHelper(self.properties.getProperty('Channel name'), responseMessageClass).waitForResponse(timeout=7000)

        if grasping_state is not None and self.properties.getPropertyEnumValue('Action name') == 'Open':
            if grasping_state.data == 0:
                # print "Hand opening successful"
                self.statusMessage = "Hand opening successful"
            else:
                self.fail("Could not open hand")
        elif grasping_state is not None and self.properties.getPropertyEnumValue('Action name') == 'Grasp':
            if grasping_state.data == 1:
                # print "Grasping successful"
                self.statusMessage = "Grasping successful"
            else:
                self.fail("No object in hand")
        else:
            self.fail("Grasping state timeout")


class CommitFootstepPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Plan name', '')

    def run(self):

        #planName = self.properties.getProperty('Plan name')
        #plan = om.findObjectByName(planName)
        #if not isinstance(plan, FootstepPlanItem):
        #    self.fail('could not find footstep plan')
        #plan = plan.plan
        plan = robotSystem.footstepsDriver.lastFootstepPlan

        robotSystem.footstepsDriver.commitFootstepPlan(plan)


class CommitManipulationPlan(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Plan name', '')

    def run(self):

        planName = self.properties.getProperty('Plan name')
        plan = om.findObjectByName(planName)
        if not isinstance(plan, ManipulationPlanItem):
            self.fail('could not find manipulation plan')
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
        assert plan is not None
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

        if not footstepPlan:
            self.fail('failed to get a footstep plan response')

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
            self.fail('frame not found: %s' % name)
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


class FitShelfItem(PointCloudAlgorithmBase):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Annotation input name', '')
        properties.addProperty('Cluster tolerance', 0.02, attributes=propertyset.PropertyAttributes(decimals=3, minimum=0.0, maximum=10))

    def getAnnotationInput(self):
        obj = om.findObjectByName(self.properties.getProperty('Annotation input name'))
        if obj is None:
            self.fail('user annotation not found')
        return obj


    def run(self):

        polyData = self.getPointCloud()
        annotation = self.getAnnotationInput()
        annotationPoint = annotation.annotationPoints[0]

        mesh = segmentation.fitShelfItem(polyData, annotationPoint, clusterTolerance=self.properties.getProperty('Cluster tolerance'))

        annotation.setProperty('Visible', False)
        om.removeFromObjectModel(om.findObjectByName('shelf item'))
        obj = vis.showPolyData(mesh, 'shelf item', color=[0,1,0])
        t = transformUtils.frameFromPositionAndRPY(segmentation.computeCentroid(mesh), [0,0,0])
        segmentation.makeMovable(obj, t)


class SpawnValveAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):

        properties.addProperty('Radius', 0.195, attributes=om.PropertyAttributes(decimals=4, minimum=0, maximum=10))
        properties.addProperty('Position', [0.7, -0.22, 1.21], attributes=om.PropertyAttributes(decimals=3, minimum=-1e4, maximum=1e4))
        properties.addProperty('Rotation',  [180, -90, 16], attributes=om.PropertyAttributes(decimals=2, minimum=-360, maximum=360))

    def getGroundFrame(self):
        return vtk.vtkTransform()

        robotModel = robotSystem.robotStateModel
        baseLinkFrame = robotModel.model.getLinkFrame(robotModel.model.getLinkNames()[0])
        #baseLinkFrame.PostMultiply()
        #baseLinkFrame.Translate(0,0,-baseLinkFrame.GetPosition()[2])
        return baseLinkFrame
        #return robotSystem.footstepsDriver.getFeetMidPoint(robotModel)

    def computeValveFrame(self):
        position = self.properties.getProperty('Position')
        rpy = self.properties.getProperty('Rotation')
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.getGroundFrame())
        return t

    def run(self):

        radius = self.properties.getProperty('Radius')
        thickness = 0.03

        folder = om.getOrCreateContainer('affordances')
        frame = self.computeValveFrame()
        d = DebugData()
        d.addLine(np.array([0, 0, -thickness/2.0]), np.array([0, 0, thickness/2.0]), radius=radius)
        mesh = d.getPolyData()
        params = dict(radius=radius, length=thickness, xwidth=radius, ywidth=radius, zwidth=thickness, otdf_type='steering_cyl', friendly_name='valve')

        affordance = vis.showPolyData(mesh, 'valve', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder, alpha=1.0)
        frame = vis.showFrame(frame, 'valve frame', parent=affordance, visible=False, scale=radius)
        affordance.actor.SetUserTransform(frame.transform)
        affordance.setAffordanceParams(params)
        affordance.updateParamsFromActorTransform()


class SpawnDrillBarrelAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):

        properties.addProperty('Position', [0.5, -0.22, 1.2], attributes=om.PropertyAttributes(decimals=3, minimum=-1e4, maximum=1e4))
        properties.addProperty('Rotation',  [0, 0, 0], attributes=om.PropertyAttributes(decimals=2, minimum=-360, maximum=360))

    def getGroundFrame(self):
        return vtk.vtkTransform()

        robotModel = robotSystem.robotStateModel
        baseLinkFrame = robotModel.model.getLinkFrame(robotModel.model.getLinkNames()[0])
        #baseLinkFrame.PostMultiply()
        #baseLinkFrame.Translate(0,0,-baseLinkFrame.GetPosition()[2])
        return baseLinkFrame
        #return robotSystem.footstepsDriver.getFeetMidPoint(robotModel)

    def computeAffordanceFrame(self):
        position = self.properties.getProperty('Position')
        rpy = self.properties.getProperty('Rotation')
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.getGroundFrame())
        return t

    def run(self):

        folder = om.getOrCreateContainer('affordances')

        frame = self.computeAffordanceFrame()
        mesh = segmentation.getDrillBarrelMesh()
        params = segmentation.getDrillAffordanceParams(np.array(frame.GetPosition()), [1,0,0], [0,1,0], [0,0,1], 'dewalt_barrel')

        affordance = vis.showPolyData(mesh, 'drill', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder)
        frame = vis.showFrame(frame, 'drill frame', parent=affordance, visible=False, scale=0.2)
        affordance.actor.SetUserTransform(frame.transform)
        affordance.setAffordanceParams(params)
        affordance.updateParamsFromActorTransform()


class SpawnDrillRotaryAffordance(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):

        properties.addProperty('Position', [0.5, -0.22, 1.2], attributes=om.PropertyAttributes(decimals=3, minimum=-1e4, maximum=1e4))
        properties.addProperty('Rotation',  [0, 0, 0], attributes=om.PropertyAttributes(decimals=2, minimum=-360, maximum=360))

    def getGroundFrame(self):
        return vtk.vtkTransform()

        robotModel = robotSystem.robotStateModel
        baseLinkFrame = robotModel.model.getLinkFrame(robotModel.model.getLinkNames()[0])
        #baseLinkFrame.PostMultiply()
        #baseLinkFrame.Translate(0,0,-baseLinkFrame.GetPosition()[2])
        return baseLinkFrame
        #return robotSystem.footstepsDriver.getFeetMidPoint(robotModel)

    def computeAffordanceFrame(self):
        position = self.properties.getProperty('Position')
        rpy = self.properties.getProperty('Rotation')
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.getGroundFrame())
        return t

    def run(self):

        folder = om.getOrCreateContainer('affordances')
        frame = self.computeAffordanceFrame()
        mesh = segmentation.getDrillMesh()
        params = segmentation.getDrillAffordanceParams(np.array(frame.GetPosition()), [1,0,0], [0,1,0], [0,0,1])

        affordance = vis.showPolyData(mesh, 'drill', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder)
        frame = vis.showFrame(frame, 'drill frame', parent=affordance, visible=False, scale=0.2)
        affordance.actor.SetUserTransform(frame.transform)
        affordance.setAffordanceParams(params)
        affordance.updateParamsFromActorTransform()


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

        for i in range(numberOfSamples):
            targetPos = targetPoints[i]
            targetFrame = transformUtils.copyFrame(gazeTargetFrame.transform)
            targetFrame.Translate(targetPos - np.array(targetFrame.GetPosition()))
            self.appendPositionConstraintForTargetFrame(targetFrame, i+1)

        gazeConstraint = self.constraintSet.constraints[0]
        assert isinstance(gazeConstraint, ikplanner.ikconstraints.WorldGazeDirConstraint)
        gazeConstraint.tspan = [1.0, numberOfSamples]


        plan = self.constraintSet.runIkTraj()

        _addPlanItem(plan, '%s gaze plan' % gazeTargetFrame.getProperty('Name'), ManipulationPlanItem)

