from director import applogic
from director import objectmodel as om
from director import visualization as vis



class RobotLinkSelector(object):

    def __init__(self):
        self.selectedLink = None
        self.setupMenuAction()

    def setupMenuAction(self):
        self.action = applogic.addMenuAction('Tools', 'Robot Link Selector')
        self.action.setCheckable(True)
        self.action.checked = False

    def enabled(self):
        return self.action.checked == True

    def setEnabled(self, enabled):
        if (enabled and not self.action.checked) or (not enabled and self.action.checked):
            self.action.trigger()

    def onLeftDoubleClick(self, displayPoint, view, event):
        return self.selectLink(displayPoint, view)

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
