import vtk
import time
import numpy as np
from ddapp.timercallback import TimerCallback

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
        c.SetViewUp([0.0, 0.0, 1.0])

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
        for i in xrange(3):
            if not self.followAxes[i]:
                delta[i] = 0.0

        self.lastTrackPosition = newTrackPosition

        c = self.view.camera()

        oldFocalPoint = np.array(c.GetFocalPoint())
        oldPosition = np.array(c.GetPosition())

        c.SetFocalPoint(oldFocalPoint + delta)
        c.SetPosition(oldPosition + delta)

        self.view.render()
