from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director.simpletimer import FPSCounter
from director import filterUtils
from director import transformUtils
from director.debugVis import DebugData

from optitrack import optitrack_frame_t
from optitrack import optitrack_marker_set_t
from optitrack import optitrack_marker_t
from optitrack import optitrack_rigid_body_t


class OptitrackVisualizer(object):

    def __init__(self, channel='OPTITRACK_FRAMES', name='Optitrack Visualier'):
        self.name = name
        self.channel = channel
        self.subscriber = None

        self.optitrackToWorld = transformUtils.frameFromPositionAndRPY([0,0,0],[90,0,90])
        self.subscriber = None
        self.fpsCounter = FPSCounter()
        self.fpsCounter.printToConsole = False

    def _addSubscriber(self):
        assert self.subscriber is None
        self.subscriber = lcmUtils.addSubscriber(self.channel, optitrack_frame_t, self.onMessage)
        self.subscriber.setSpeedLimit(100)

    def _removeSubscriber(self):
        assert self.subscriber is not None
        lcmUtils.removeSubscriber(self.subscriber)
        self.subscriber = None

    def isEnabled(self):
        return self.subscriber is not None

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self._addSubscriber()
        elif not enabled and self.isEnabled():
            self._removeSubscriber()
            self.removeRootFolder()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def getRootFolder(self):
        folder = om.getOrCreateContainer(self.name)
        return folder

    def removeRootFolder(self):
        om.removeFromObjectModel(self.getRootFolder())

    def _makeMarkers(self, points, radius=0.01):
        d = DebugData()
        for p in points:
            d.addSphere(p, radius=radius, resolution=8)
        return d.getPolyData()

    def _handleRigidBodies(self, bodies):

        if not bodies:
            return

        folder = self.getRootFolder()

        for body in bodies:
            name = 'Body ' + str(body.id)

            x,y,z,w = body.quat
            quat = (w,x,y,z)
            bodyToOptitrack = transformUtils.transformFromPose(body.xyz, quat)

            bodyToWorld = transformUtils.concatenateTransforms((bodyToOptitrack, self.optitrackToWorld))


            obj = folder.findChild(name)
            if not obj:
                geometry = self._makeMarkers(body.marker_xyz)
                geometry = filterUtils.transformPolyData(geometry, bodyToOptitrack.GetLinearInverse())
                obj = vis.showPolyData(geometry, name, parent=folder, color=[1,0,0])
                frameObj = vis.addChildFrame(obj)
                frameObj.setProperty('Scale', 0.2)
                frameObj.setProperty('Visible', True)

            obj.getChildFrame().copyFrame(bodyToWorld)


    def onMessage(self, msg):
        self.fpsCounter.tick()
        self.lastMessage = msg
        self._handleRigidBodies(msg.rigid_bodies)


if __name__ == '__main__':
    from director import applogic
    optitrackVis = OptitrackVisualizer()
    optitrackVis.setEnabled(True)
    applogic.MenuActionToggleHelper('Tools', optitrackVis.name, optitrackVis.isEnabled, optitrackVis.setEnabled)
