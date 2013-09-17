import vtk
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
