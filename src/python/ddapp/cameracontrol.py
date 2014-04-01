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
        self.positionZoom = 0.2

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

        c.SetFocalPoint(newFocalPoint)
        c.SetPosition(newPosition)
        c.SetViewUp([0.0, 0.0, 1.0])

        self.interp.AddCamera(1.0, c)
        self.startTime = time.time()
        self.start()


    def tick(self):

        elapsed = time.time() - self.startTime
        t = elapsed / float(self.flyTime)

        self.interp.InterpolateCamera(t, self.view.camera())
        self.view.render()

        if t > 1.0:
            return False
