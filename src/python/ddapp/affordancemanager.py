from ddapp import objectmodel as om
from ddapp import affordanceitems
from ddapp import affordancecollection as ac
from ddapp import visualization as vis
from ddapp.timercallback import TimerCallback
from ddapp import vtkAll as vtk
from ddapp.thirdparty import numpyjsoncoder


class AffordanceObjectModelManager(object):

    def __init__(self, affordanceCollection, view):
        self.affordanceCollection = affordanceCollection
        self.affordanceCollection.connectDescriptionUpdated(self._onDescriptionUpdated)
        self.affordanceCollection.connectDescriptionRemoved(self._onDescriptionRemoved)
        self.view = view
        self.notifyFrequency = 5 # throttle to 5 lcm messages per second for affordance updates
        self._ignoreChanges = False

        self._pendingUpdates = set()
        self.timer = TimerCallback()
        self.timer.callback = self._notifyPendingUpdates


    def getAffordances(self):
        return [obj for obj in om.getObjects() if isinstance(obj, affordanceitems.AffordanceItem)]

    def getAffordanceId(self, aff):
        return aff.getProperty('uuid')

    def getAffordanceById(self, affordanceId):
        for aff in self.getAffordances():
            if self.getAffordanceId(aff) == affordanceId:
                return aff

    def getAffordanceDescription(self, aff):
        return aff.getDescription()

    def registerAffordance(self, aff, notify=True):
        aff.properties.connectPropertyChanged(self._onAffordancePropertyChanged)
        aff.getChildFrame().connectFrameModified(self._onAffordanceFrameChanged)
        if notify:
            self.notifyAffordanceUpdate(aff)

    def removeAffordance(self, aff):
        self.affordanceCollection.removeDescription(aff.getProperty('uuid'))

    def notifyAffordanceUpdate(self, aff):

        shouldNotify = not self._pendingUpdates and not self.timer.singleShotTimer.isActive()
        self._pendingUpdates.add(aff)
        if shouldNotify:
            self._notifyPendingUpdates()

    def _notifyPendingUpdates(self):

        if self._pendingUpdates:
            self.timer.singleShot(1.0/self.notifyFrequency)

        for aff in self._pendingUpdates:
            self.affordanceCollection.updateDescription(self.getAffordanceDescription(aff), notify=False)
        self._pendingUpdates.clear()

    def _onAffordancePropertyChanged(self, propertySet, propertyName):
        if self._ignoreChanges:
            return
        self.notifyAffordanceUpdate(self.getAffordanceById(propertySet.getProperty('uuid')))

    def _onAffordanceFrameChanged(self, frameObj):
        if self._ignoreChanges:
            return
        aff = frameObj.parent()
        self.notifyAffordanceUpdate(aff)

    def _loadAffordanceFromDescription(self, desc):
        className = desc['classname']
        cls = getattr(affordanceitems, className)
        aff = cls(desc['Name'], self.view)
        om.addToObjectModel(aff, parentObj=om.getOrCreateContainer('affordances'))
        vis.addChildFrame(aff)
        aff.loadDescription(desc)
        self.registerAffordance(aff, notify=False)

    def _onDescriptionUpdated(self, affordanceCollection, descriptionId):
        aff = self.getAffordanceById(descriptionId)
        desc = affordanceCollection.collection[descriptionId]

        if aff:
            self._ignoreChanges = True
            aff.loadDescription(desc)
            self._ignoreChanges = False

        else:
            aff = self._loadAffordanceFromDescription(desc)

    def _onDescriptionRemoved(self, affordanceCollection, descriptionId):
        om.removeFromObjectModel(self.getAffordanceById(descriptionId))
