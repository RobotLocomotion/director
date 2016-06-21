import director.vtkAll as vtk
import director.objectmodel as om
from director import lcmUtils

# if bot_lcmgl cannot be important than this module will not be able to
# support lcmgl, but it can still be imported in a disabled state
try:
    import bot_lcmgl
    import vs as lcmCollections
    LCMGL_AVAILABLE = True
except ImportError:
    LCMGL_AVAILABLE = False


class CollectionInfo():
    def __init__(self, collectionId, collectionName, collectionType, collectionShow):
        self.id =   collectionId
        self.name = collectionName
        self.type = collectionType
        self.show = collectionShow


class CollectionInfoObject(om.ObjectModelItem):

    def __init__(self, collectionInfo, actor):

        om.ObjectModelItem.__init__(self, collectionInfo.name, om.Icons.Robot)

        self.actor = actor
        self.collectionInfo = collectionInfo
        #self.actor.SetUseBounds(False)
        self.addProperty('Visible', actor.GetVisibility())
        self.views = []

        self.collectionsObject = self.getDrawObject("COLLECTIONS")

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            makeVisible = self.getProperty(propertyName)
            self.actor.SetVisibility(makeVisible)
            self.collectionsObject.setCollectionEnable(self.collectionInfo.id, makeVisible)
            self.collectionsObject.renderAllViews()


    def getDrawObject(self, name):
        parent = om.getOrCreateContainer('Collections')
        return parent.findChild(name)

    def addToView(self, view):
        if view in self.views:
            return
        self.views.append(view)
        view.renderer().AddActor(self.actor)
        view.render()

    def onRemoveFromObjectModel(self):
        self.collectionsObject.removeIdFromCollections(self.collectionInfo.id)
        self.collectionsObject.getCollectionsInfo()
        self.collectionsObject.renderAllViews()


class CollectionsObject(om.ObjectModelItem):

    def __init__(self, name, actor):

        om.ObjectModelItem.__init__(self, name, om.Icons.Collections)


        self.actor = actor
        self.actor.SetUseBounds(False)
        self.addProperty('Visible', actor.GetVisibility())
        self.addProperty('Start', 0.0, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.01))
        self.addProperty('End', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.01))
        self.addProperty('Points Alpha', 0.8, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1))

        self.addProperty('Fill Scans', False)

        self.addProperty('Point Width', 1.0, attributes=om.PropertyAttributes(decimals=0, minimum=5, maximum=30.0, singleStep=1))
        self.addProperty('Pose Width', 1.0, attributes=om.PropertyAttributes(decimals=1, minimum=0.1, maximum=30.0, singleStep=0.1))
        self.addProperty('Color by Time', False)

        self.addProperty('Elevation by Time', False)
        self.addProperty('Elevation by Collection', False)
        self.addProperty('Max Elevation', 0.0, attributes=om.PropertyAttributes(decimals=1, minimum=0, maximum=100.0, singleStep=0.1))

        self.views = []

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            self.actor.SetVisibility(self.getProperty(propertyName))
            makeVisible = self.getProperty(propertyName)

            parent = om.getOrCreateContainer('COLLECTIONS')
            for coll in self.children():
                coll.setProperty('Visible',makeVisible)

            #for coll in self.collectionInfos:
            #    self.actor.SetVisibility(makeVisible)
            #    self.actor.setProperty('Visible',makeVisible)
            #    self.actor.setEnabled(coll.id, makeVisible)

        elif propertyName == 'Start':
            self.actor.setRangeStart(self.getProperty(propertyName))
        elif propertyName == 'End':
            self.actor.setRangeEnd(self.getProperty(propertyName))
        elif propertyName == 'Points Alpha':
            self.actor.setAlphaPoints(self.getProperty(propertyName))
        elif propertyName == 'Fill Scans':
            self.actor.setFillScans(self.getProperty(propertyName))
        elif propertyName == 'Point Width':
            self.actor.setPointWidth(self.getProperty(propertyName))
        elif propertyName == 'Pose Width':
            self.actor.setPoseWidth(self.getProperty(propertyName))
        elif propertyName == 'Color by Time':
            self.actor.setColorByTime(self.getProperty(propertyName))
        elif propertyName == 'Elevation by Time':
            self.actor.setElevationByTime(self.getProperty(propertyName))
        elif propertyName == 'Elevation by Collection':
            self.actor.setElevationByCollection(self.getProperty(propertyName))
        elif propertyName == 'Max Elevation':
            self.actor.setMaxElevation(self.getProperty(propertyName))

        self.renderAllViews()

    def addToView(self, view):
        if view in self.views:
            return
        self.views.append(view)
        view.renderer().AddActor(self.actor)
        view.render()

    def renderAllViews(self):
        for view in self.views:
            view.render()

    def onRemoveFromObjectModel(self):
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)
        assert len(self.views) == 0

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        view.renderer().RemoveActor(self.actor)
        view.render()

    def getCollectionsInfo(self):
        numberOfCollections = self.actor.getCollectionsSize()

        self.collectionInfos = []
        for i in range(0,numberOfCollections):
            cId = self.actor.getCollectionsId(i)
            cName = self.actor.getCollectionsName(i)
            cType = self.actor.getCollectionsType(i)
            cShow = self.actor.getCollectionsShow(i)
            cInfo = CollectionInfo(cId, cName, cType, cShow )
            self.collectionInfos.append(cInfo)

    def disableOne(self):
        self.setCollectionEnable(1,False)

    def removeIdFromCollections(self, cId):
        self.actor.removeIdFromCollections(cId)

    def setCollectionEnable(self, cId, enable):
        self.actor.setEnabled(cId, enable)

    def on_obj_collection_data(self, msgBytes):
        self.actor.on_obj_collection_data(msgBytes.data())
        self.getCollectionsInfo()
        self.renderAllViews()

    def on_link_collection_data(self, msgBytes):
        self.actor.on_link_collection_data(msgBytes.data())
        self.getCollectionsInfo()
        self.renderAllViews()

    def on_points_collection_data(self, msgBytes):
        self.actor.on_points_collection_data(msgBytes.data())
        self.getCollectionsInfo()
        self.renderAllViews()

managerInstance = None

class CollectionsManager(object):

    def __init__(self, view):
        assert LCMGL_AVAILABLE
        self.view = view
        self.subscriber = None
        self.enable()

    def isEnabled(self):
        return self.subscriber is not None

    def setEnabled(self, enabled):
        if enabled and not self.subscriber:
            #self.subscriber = lcmUtils.addSubscriber('LCMGL.*', callback=self.onMessage)
            self.subscriber = lcmUtils.addSubscriber('OBJECT_COLLECTION', callback=self.on_obj_collection_data)
            self.subscriber1 = lcmUtils.addSubscriber('LINK_COLLECTION', callback=self.on_link_collection_data)
            self.subscriber2 = lcmUtils.addSubscriber('POINTS_COLLECTION', callback=self.on_points_collection_data)
        elif not enabled and self.subscriber:
            lcmUtils.removeSubscriber(self.subscriber)
            self.subscriber = None

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def on_obj_collection_data(self, msgBytes, channel):
        msg = lcmCollections.object_collection_t.decode(msgBytes.data())
        drawObject = self.getDrawObject("COLLECTIONS")
        if not drawObject:
            drawObject = self.addDrawObject("COLLECTIONS", msgBytes)
        drawObject.on_obj_collection_data(msgBytes)

    def on_link_collection_data(self, msgBytes, channel):
        msg = lcmCollections.link_collection_t.decode(msgBytes.data())
        drawObject = self.getDrawObject("COLLECTIONS")
        if not drawObject:
            drawObject = self.addDrawObject("COLLECTIONS", msgBytes)
        drawObject.on_link_collection_data(msgBytes)

    def on_points_collection_data(self, msgBytes, channel):
        msg = lcmCollections.point3d_list_collection_t.decode(msgBytes.data())
        drawObject = self.getDrawObject("COLLECTIONS")
        if not drawObject:
            drawObject = self.addDrawObject("COLLECTIONS", msgBytes)
        drawObject.on_points_collection_data(msgBytes)

        self.addAllObjects()

    def getDrawObject(self, name):
        parent = om.getOrCreateContainer('Collections')
        return parent.findChild(name)

    def addDrawObject(self, name, msgBytes):
        actor = vtk.vtkCollections()
        obj = CollectionsObject(name, actor)
        om.addToObjectModel(obj, om.getOrCreateContainer('Collections'))
        obj.addToView(self.view)
        return obj

    def addAllObjects(self):
        drawObject = self.getDrawObject("COLLECTIONS")
        if not drawObject:
            return

        for coll in drawObject.collectionInfos:

            # If the icon exists, don't re-add it
            for existingCollection in drawObject.children():
                if coll.id == existingCollection.collectionInfo.id:
                    return

            actor = vtk.vtkCollections()
            obj = CollectionInfoObject(coll, actor)
            om.addToObjectModel(obj, drawObject)
            obj.addToView(self.view)


def init(view):
    if not hasattr(vtk, 'vtkCollections'):
        return None

    global managerInstance
    managerInstance = CollectionsManager(view)

    return managerInstance
