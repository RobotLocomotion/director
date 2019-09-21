from director import objectmodel as om
from director import affordanceitems
from director import lcmobjectcollection
from director import visualization as vis
from director.timercallback import TimerCallback
from director.uuidutil import newUUID
from director import vtkAll as vtk
from director.thirdparty import numpyjsoncoder
import traceback

class AffordanceObjectModelManager(object):

    def __init__(self, view):
        self.collection = lcmobjectcollection.LCMObjectCollection(channel='AFFORDANCE_COLLECTION_COMMAND')
        self.collection.connectDescriptionUpdated(self._onDescriptionUpdated)
        self.collection.connectDescriptionRemoved(self._onDescriptionRemoved)
        self.view = view
        self.notifyFrequency = 30 # throttle lcm messages per second sent for affordance updates
        self._ignoreChanges = False

        self._pendingUpdates = set()
        self.timer = TimerCallback()
        self.timer.callback = self._notifyPendingUpdates

        self.affordanceUpdater = None

    def setAffordanceUpdater(self, affordanceUpdater):
        self.affordanceUpdater = affordanceUpdater

    def getAffordances(self):
        return [obj for obj in om.getObjects() if isinstance(obj, affordanceitems.AffordanceItem)]


    def getCollisionAffordances(self):
        affs = []
        for aff in self.getAffordances():
            if aff.getProperty('Collision Enabled'):
                affs.append(aff)
        return affs

    def getAffordanceId(self, aff):
        return aff.getProperty('uuid')

    def newAffordanceFromDescription(self, desc):
        if 'uuid' not in desc:
            desc['uuid'] = newUUID()
        self.collection.updateDescription(desc)
        return self.getAffordanceById(desc['uuid'])

    def getAffordanceById(self, affordanceId):
        for aff in self.getAffordances():
            if self.getAffordanceId(aff) == affordanceId:
                return aff

    def getAffordanceDescription(self, aff):
        return aff.getDescription()

    def registerAffordance(self, aff, notify=True):
        aff.connectRemovedFromObjectModel(self._onAffordanceRemovedFromObjectModel)
        aff.properties.connectPropertyChanged(self._onAffordancePropertyChanged)
        aff.getChildFrame().connectFrameModified(self._onAffordanceFrameChanged)
        if notify:
            self.notifyAffordanceUpdate(aff)

    def removeAffordance(self, aff):
        self.collection.removeDescription(aff.getProperty('uuid'), notify=False)

    def notifyAffordanceUpdate(self, aff):

        if not isinstance(aff, affordanceitems.AffordanceItem):
            return

        shouldNotify = not self._pendingUpdates and not self.timer.singleShotTimer.isActive()
        self._pendingUpdates.add(aff)
        if shouldNotify:
            self._notifyPendingUpdates()

    def _notifyPendingUpdates(self):

        if self._pendingUpdates:
            self.timer.singleShot(1.0/self.notifyFrequency)

        for aff in self._pendingUpdates:
            try:
                self.collection.updateDescription(self.getAffordanceDescription(aff), notify=False)
            except:
                print(traceback.format_exc())

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

    def _onAffordanceRemovedFromObjectModel(self, objectModel, aff):
        if self._ignoreChanges:
            return
        self.removeAffordance(aff)

    def _loadAffordanceFromDescription(self, desc):
        className = desc['classname']
        cls = getattr(affordanceitems, className)
        aff = cls(desc['Name'], self.view)
        om.addToObjectModel(aff, parentObj=om.getOrCreateContainer('affordances'))
        frame = vis.addChildFrame(aff)
        frame.setProperty('Deletable', False)
        aff.loadDescription(desc, copyMode=aff.COPY_MODE_ALL)
        self.registerAffordance(aff, notify=False)

    def _onDescriptionUpdated(self, collection, descriptionId):
        aff = self.getAffordanceById(descriptionId)
        desc = collection.getDescription(descriptionId)

        if aff:
            self._ignoreChanges = True
            aff.loadDescription(desc, copyMode=aff.COPY_MODE_SKIP_LOCAL)
            self._ignoreChanges = False

        else:
            aff = self._loadAffordanceFromDescription(desc)

    def _onDescriptionRemoved(self, collection, descriptionId):
        self._ignoreChanges = True
        om.removeFromObjectModel(self.getAffordanceById(descriptionId))
        self._ignoreChanges = False
