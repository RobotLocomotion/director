from ddapp import objectmodel as om
from ddapp import visualization as vis

class AffordanceGraspUpdater(object):

    def __init__(self, robotModel, extraModels=None):
        self.robotModel = robotModel
        self.frameSyncs = {}

        models = [robotModel]
        if extraModels:
            models.extend(extraModels)

        for model in models:
            model.connectModelChanged(self.onRobotModelChanged)

    def onRobotModelChanged(self, model):
        for linkName in ['l_hand', 'r_hand']:
            self.updateLinkFrame(model, linkName, create=False)

    def getAffordanceFrame(self, affordanceName):
        frame = om.findObjectByName(affordanceName + ' frame')
        assert frame
        return frame

    def updateLinkFrame(self, robotModel, linkName, create=True):

        linkFrameName = '%s frame' % linkName

        if not create and not om.findObjectByName(linkFrameName):
            return

        t = robotModel.getLinkFrame(linkName)
        return vis.updateFrame(t, linkFrameName, scale=0.2, visible=False, parent=self.robotModel)

    def graspAffordance(self, affordanceName, side):

        if affordanceName in self.frameSyncs:
            return

        affordanceFrame = self.getAffordanceFrame(affordanceName)

        linkName = 'l_hand' if side == 'left' else 'r_hand'
        linkFrame = self.updateLinkFrame(self.robotModel, linkName)

        frameSync = vis.FrameSync()
        frameSync.addFrame(linkFrame)
        frameSync.addFrame(affordanceFrame)

        self.frameSyncs[affordanceName] = frameSync

    def ungraspAffordance(self, affordanceName):
        try:
            del self.frameSyncs[affordanceName]
        except KeyError:
            pass

        if not self.frameSyncs:
            om.removeFromObjectModel(om.findObjectByName('l_hand frame'))
            om.removeFromObjectModel(om.findObjectByName('r_hand frame'))
