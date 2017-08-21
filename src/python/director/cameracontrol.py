import vtk
import time
import numpy as np
from director import transformUtils
from director.timercallback import TimerCallback
from director import propertyset
from collections import OrderedDict

class OrbitController(TimerCallback):

    def __init__(self, view):
        TimerCallback.__init__(self)
        self.view = view
        self.orbitTime = 20.0

    def tick(self):
        speed = 360.0 / self.orbitTime
        degrees = self.elapsed * speed
        self.view.camera().Azimuth(degrees)
        self.view.render()


class CameraInterpolator(object):

    def __init__(self, view):
        self.view = view
        self.reset()

    def getViewCameraCopy(self):
        camera = vtk.vtkCamera()
        camera.DeepCopy(self.view.camera())
        return camera

    def reset(self):
        self.interp = vtk.vtkCameraInterpolator()

    def addCameraAtTime(self, camera, t):
        self.interp.AddCamera(t, camera)

    def addViewCameraAtTime(self, t):
        self.addCameraAtTime(self.getViewCameraCopy(), t)

    def setViewCameraAtTime(self, t):
        self.interp.InterpolateCamera(t, self.view.camera())
        self.view.render()


class Flyer(TimerCallback):

    def __init__(self, view):
        TimerCallback.__init__(self)
        self.view = view
        self.flyTime = 0.5
        self.startTime = 0.0
        self.maintainViewDirection = False
        self.positionZoom = 0.7

    def getCameraCopy(self):
        camera = vtk.vtkCamera()
        camera.DeepCopy(self.view.camera())
        return camera

    def zoomTo(self, newFocalPoint, newPosition=None):

        self.interp = vtk.vtkCameraInterpolator()
        self.interp.AddCamera(0.0, self.getCameraCopy())

        c = self.getCameraCopy()
        newFocalPoint = np.array(newFocalPoint)
        oldFocalPoint = np.array(c.GetFocalPoint())
        oldViewUp = np.array(c.GetViewUp())
        oldPosition = np.array(c.GetPosition())

        if newPosition is None:
            if self.maintainViewDirection:
                newPosition = oldPosition + (newFocalPoint - oldFocalPoint)
            else:
                newPosition = oldPosition
            newPosition += self.positionZoom*(newFocalPoint - newPosition)
            #newPosition = newFocalPoint - self.positionZoom*(newFocalPoint - newPosition)

        c.SetFocalPoint(newFocalPoint)
        c.SetPosition(newPosition)
        c.SetViewUp(oldViewUp)

        self.interp.AddCamera(1.0, c)
        self.startTime = time.time()
        self.start()


    def tick(self):

        elapsed = time.time() - self.startTime
        t = (elapsed / float(self.flyTime)) if self.flyTime > 0 else 1.0

        self.interp.InterpolateCamera(t, self.view.camera())
        self.view.render()

        if t >= 1.0:
            return False


class CameraTracker(object):

    def __init__(self, view, targetFrame):
        self.view = view
        self.targetFrame = targetFrame
        self.camera = view.camera()
        self.actions = []
        self.properties = propertyset.PropertySet()
        self.properties.connectPropertyChanged(self.onPropertyChanged)
        self.setup()

    def getTargetPose(self):
        return transformUtils.poseFromTransform(self.targetFrame.transform)

    def getTargetQuaternion(self):
        return self.getTargetPose()[1]

    def getTargetPosition(self):
        return np.array(self.targetFrame.transform.GetPosition())

    def getCameraTransform(self):
        c = self.camera
        return transformUtils.getLookAtTransform(c.GetFocalPoint(), c.GetPosition(), c.GetViewUp())

    def getCameraToTargetTransform(self, targetFrame):

        targetToWorld = transformUtils.copyFrame(targetFrame)
        cameraToWorld = self.getCameraTransform()

        cameraToTarget = transformUtils.concatenateTransforms([cameraToWorld, targetToWorld.GetLinearInverse()])
        focalDistance = np.linalg.norm(np.array(self.camera.GetFocalPoint()) - np.array(self.camera.GetPosition()))
        return cameraToTarget, focalDistance

    def setCameraFocalPointToTarget(self):
        self.camera.SetFocalPoint(self.getTargetPosition())
        self.view.render()

    def getProperties():
        return self.properties

    def setup(self):
        pass

    def reset(self):
        pass

    def update(self):
        pass

    def onAction(self, actionName):
        pass

    def getMinimumUpdateRate(self):
        return 0

    def onPropertyChanged(self, propertySet, propertyName):
        pass


class PositionTracker(CameraTracker):

    def setup(self):
        self.actions = ['Re-center']

    def onAction(self, actionName):
        if actionName == 'Re-center':
            self.setCameraFocalPointToTarget()

    def reset(self):
        self.lastTargetPosition = self.getTargetPosition()
        self.lastTargetQuaternion = self.getTargetQuaternion()

    def update(self):

        newTargetPosition = self.getTargetPosition()

        delta = newTargetPosition - self.lastTargetPosition

        followAxes = [True, True, True]
        for i in range(3):
            if not followAxes[i]:
                delta[i] = 0.0

        self.lastTargetPosition = newTargetPosition

        c = self.camera

        oldFocalPoint = np.array(c.GetFocalPoint())
        oldPosition = np.array(c.GetPosition())

        c.SetFocalPoint(oldFocalPoint + delta)
        c.SetPosition(oldPosition + delta)
        self.view.render()


class LookAtTracker(CameraTracker):

    def update(self):
        self.setCameraFocalPointToTarget()

    def reset(self):
        pass


class OrbitTracker(PositionTracker):

    def setup(self):
        super(OrbitTracker, self).setup()
        self.properties.addProperty('Orbit Time (s)', 20, attributes=propertyset.PropertyAttributes(minimum=1, maximum=100, singleStep=1))

    def update(self):
        super(OrbitTracker, self).update()
        orbitTime = self.properties.getProperty('Orbit Time (s)')
        speed = 360.0 / orbitTime
        degrees = self.dt * speed
        self.view.camera().Azimuth(degrees)
        self.view.render()

    def getMinimumUpdateRate(self):
        return 60


class PositionOrientationTracker(CameraTracker):

    def storeTargetPose(self):
        self.lastTargetPosition = self.getTargetPosition()
        self.lastTargetQuaternion = self.getTargetQuaternion()

    def reset(self):
        self.storeTargetPose()

        targetToWorld = transformUtils.copyFrame(self.targetFrame.transform)
        cameraToWorld = self.getCameraTransform()

        cameraToTarget = transformUtils.concatenateTransforms([cameraToWorld, targetToWorld.GetLinearInverse()])

        self.boomTransform = cameraToTarget
        self.focalDistance = np.linalg.norm(np.array(self.camera.GetFocalPoint()) - np.array(self.camera.GetPosition()))

    def update(self):

        previousTargetFrame = transformUtils.transformFromPose(self.lastTargetPosition, self.lastTargetQuaternion)
        self.storeTargetPose()

        cameraToTarget, focalDistance = self.getCameraToTargetTransform(previousTargetFrame)

        targetToWorld = self.targetFrame.transform
        #cameraToTarget = self.boomTransform
        cameraToWorld = transformUtils.concatenateTransforms([cameraToTarget, targetToWorld])

        c = self.camera

        focalPoint = cameraToWorld.TransformPoint([self.focalDistance, 0, 0])
        focalPoint = targetToWorld.GetPosition()

        #print 'focal distance:', self.focalDistance
        #print 'cameraToTarget pos:', cameraToTarget.GetPosition()
        #print 'cameraToWorld pos:', cameraToWorld.GetPosition()
        #print 'targetToWorld pos:', targetToWorld.GetPosition()
        #print 'focal pos:', focalPoint

        c.SetPosition(cameraToWorld.GetPosition())
        c.SetFocalPoint(focalPoint)
        self.view.render()


class SmoothFollowTracker(CameraTracker):

    def getMinimumUpdateRate(self):
        return 30

    def setup(self):
        self.properties.addProperty('Smooth Time (s)', 0.5, attributes=propertyset.PropertyAttributes(decimals=1, minimum=0.1, maximum=5, singleStep=0.1))
        self.properties.addProperty('Distance (m)', 15, attributes=propertyset.PropertyAttributes(decimals=1, minimum=0.5, maximum=1000.0, singleStep=1))
        self.properties.addProperty('Elevation (deg)', 10, attributes=propertyset.PropertyAttributes(minimum=-90, maximum=90, singleStep=2))
        self.properties.addProperty('Azimuth (deg)', 0, attributes=propertyset.PropertyAttributes(minimum=-180, maximum=180, singleStep=10))

    def reset(self):
        self.currentVelocity = np.array([0.0, 0.0, 0.0])

    def update(self):

        if not self.targetFrame:
            return


        r = self.properties.getProperty('Distance (m)')
        theta = np.radians(90 - self.properties.getProperty('Elevation (deg)'))
        phi = np.radians(180 - self.properties.getProperty('Azimuth (deg)'))

        x = r * np.cos(phi) * np.sin(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(theta)


        c = self.camera
        targetToWorld = self.targetFrame.transform

        currentPosition = np.array(c.GetPosition())
        desiredPosition = np.array(targetToWorld.TransformPoint([x, y, z]))

        smoothTime = self.properties.getProperty('Smooth Time (s)')

        newPosition, self.currentVelocity = smoothDamp(currentPosition, desiredPosition, self.currentVelocity, smoothTime, maxSpeed=100, deltaTime=self.dt)

        trackerToWorld = transformUtils.getLookAtTransform(targetToWorld.GetPosition(), newPosition)

        c.SetFocalPoint(targetToWorld.GetPosition())
        c.SetPosition(trackerToWorld.GetPosition())
        self.view.render()


class TargetFrameConverter(object):

    def __init__(self):
        self.targetFrame = None

    def getTargetFrame(self):
        return self.targetFrame

    @classmethod
    def canConvert(cls, obj):
        return False


class CameraTrackerManager(object):

    def __init__(self):
        self.target = None
        self.targetFrame = None
        self.trackerClass = None
        self.camera = None
        self.view = None
        self.timer = TimerCallback()
        self.timer.callback = self.updateTimer
        self.addTrackers()
        self.initTracker()

    def updateTimer(self):

        tNow = time.time()
        dt = tNow - self.tLast
        if dt < self.timer.elapsed/2.0:
            return

        self.update()

    def setView(self, view):
        self.view = view
        self.camera = view.camera()

    def setTarget(self, target):
        '''
        target should be an instance of TargetFrameConverter or
        any object that provides a method getTargetFrame().
        '''

        if target == self.target:
            return

        self.disableActiveTracker()

        if not target:
            return

        self.target = target
        self.targetFrame = target.getTargetFrame()
        self.callbackId = self.targetFrame.connectFrameModified(self.onTargetFrameModified)

        self.initTracker()

    def disableActiveTracker(self):

        if self.targetFrame:
            self.targetFrame.disconnectFrameModified(self.callbackId)

        self.target = None
        self.targetFrame = None
        self.initTracker()

    def update(self):

        tNow = time.time()
        dt = tNow - self.tLast
        self.tLast = tNow

        if self.activeTracker:
            self.activeTracker.dt = dt
            self.activeTracker.update()

    def reset(self):
        self.tLast = time.time()
        if self.activeTracker:
            self.activeTracker.reset()


    def getModeActions(self):
        if self.activeTracker:
            return self.activeTracker.actions
        return []

    def onModeAction(self, actionName):
        if self.activeTracker:
            self.activeTracker.onAction(actionName)

    def getModeProperties(self):
        if self.activeTracker:
            return self.activeTracker.properties
        return None

    def onTargetFrameModified(self, frame):
        self.update()

    def initTracker(self):

        self.timer.stop()
        self.activeTracker = self.trackerClass(self.view, self.targetFrame) if (self.trackerClass and self.targetFrame) else None
        self.reset()
        self.update()

        if self.activeTracker:
            minimumUpdateRate = self.activeTracker.getMinimumUpdateRate()
            if minimumUpdateRate > 0:
                self.timer.targetFps = minimumUpdateRate
                self.timer.start()

    def addTrackers(self):
        self.trackers = OrderedDict([
          ['Off', None],
          ['Position', PositionTracker],
          ['Position & Orientation', PositionOrientationTracker],
          ['Smooth Follow', SmoothFollowTracker],
          ['Look At', LookAtTracker],
          ['Orbit', OrbitTracker],
        ])

    def setTrackerMode(self, modeName):
        assert modeName in self.trackers
        self.trackerClass = self.trackers[modeName]
        self.initTracker()


def smoothDamp(current, target, currentVelocity, smoothTime, maxSpeed, deltaTime):
    '''
    Based on Unity3D SmoothDamp
    See: http://answers.unity3d.com/answers/310645/view.html
    '''
    smoothTime = max(0.0001, smoothTime)

    num = 2.0 / smoothTime;
    num2 = num * deltaTime;
    num3 = 1.0 / (1.0 + num2 + 0.48 * num2 * num2 + 0.235 * num2 * num2 * num2)
    num4 = current - target
    num5 = target
    num6 = maxSpeed * smoothTime
    num4 = np.clip(num4, -num6, num6)
    target = current - num4
    num7 = (currentVelocity + num * num4) * deltaTime
    currentVelocity = (currentVelocity - num * num7) * num3
    num8 = target + (num4 + num7) * num3
    for i in range(len(current)):
        if (num5[i] - current[i] > 0.0 == num8[i] > num5[i]):
            num8[i] = num5[i]
            currentVelocity[i] = (num8[i] - num5[i]) / deltaTime

    return num8, currentVelocity


class RobotModelFollower(object):

    def __init__(self, view, robotModel, jointController):
        self.view = view
        self.robotModel = robotModel
        self.jointController = jointController
        self.followAxes = [True, True, True]
        self.callbackId = None

    def start(self):
        self.callbackId = self.robotModel.connectModelChanged(self.onModelChanged)
        self.lastTrackPosition = np.array(self.jointController.q[:3])

    def stop(self):
        self.robotModel.disconnectModelChanged(self.callbackId)

    def getCameraCopy(self):
        camera = vtk.vtkCamera()
        camera.DeepCopy(self.view.camera())
        return camera

    def onModelChanged(self, model):
        newTrackPosition = np.array(self.jointController.q[:3])

        delta = newTrackPosition - self.lastTrackPosition
        for i in range(3):
            if not self.followAxes[i]:
                delta[i] = 0.0

        self.lastTrackPosition = newTrackPosition

        c = self.view.camera()

        oldFocalPoint = np.array(c.GetFocalPoint())
        oldPosition = np.array(c.GetPosition())

        c.SetFocalPoint(oldFocalPoint + delta)
        c.SetPosition(oldPosition + delta)

        self.view.render()
