from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import vtkAll as vtk
from ddapp import transformUtils
from ddapp import filterUtils
from ddapp.timercallback import TimerCallback


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


class AffordanceInCameraUpdater(object):

    def __init__(self, affordanceManager, imageView):
        self.affordanceManager = affordanceManager
        self.extraObjects = []

        self.imageView = imageView
        self.imageQueue = imageView.imageManager.queue
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.update


    def getOverlayRenderer(self, imageView):

        if not hasattr(imageView, 'overlayRenderer'):
            renWin = imageView.view.renderWindow()
            renWin.SetNumberOfLayers(2)
            ren = vtk.vtkRenderer()
            ren.SetLayer(1)
            ren.SetActiveCamera(imageView.view.camera())
            renWin.AddRenderer(ren)
            imageView.overlayRenderer = ren
        return imageView.overlayRenderer

    def addActorToImageOverlay(self, obj, imageView):

        obj.addToView(imageView.view)
        imageView.view.renderer().RemoveActor(obj.actor)

        renderers = obj.extraViewRenderers.setdefault(imageView.view, [])
        overlayRenderer = self.getOverlayRenderer(imageView)
        if overlayRenderer not in renderers:
            overlayRenderer.AddActor(obj.actor)
            renderers.append(overlayRenderer)

    def setupObjectInCamera(self, obj):

        imageView = self.imageView
        obj = vis.updatePolyData(vtk.vtkPolyData(), self.getTransformedName(obj), view=imageView.view, color=obj.getProperty('Color'), parent='camera overlay', visible=obj.getProperty('Visible'))
        self.addActorToImageOverlay(obj, imageView)
        return obj

    def getTransformedName(self, obj):
        return 'overlay ' + obj.getProperty('Name')

    def getFootsteps(self):
        plan = om.findObjectByName('footstep plan')
        if plan:
            return [child for child in plan.children() if child.getProperty('Name').startswith('step ')]
        else:
            return []

    def getObjectsToUpdate(self):
        objs = self.affordanceManager.getAffordances()
        objs += self.getFootsteps()
        objs += self.extraObjects
        return objs

    def getObjectInCamera(self, obj):
        overlayObj = om.findObjectByName(self.getTransformedName(obj))
        return overlayObj or self.setupObjectInCamera(obj)

    def update(self):

        imageView = self.imageView

        if not imageView.imageInitialized:
            return

        if not imageView.view.isVisible():
            return

        updated = set()

        for obj in self.getObjectsToUpdate():
            cameraObj = self.getObjectInCamera(obj)
            self.updateObjectInCamera(obj, cameraObj)
            updated.add(cameraObj)

        folder = om.findObjectByName('camera overlay')
        if folder:
            for child in folder.children():
                if child not in updated:
                    om.removeFromObjectModel(child)

    def updateObjectInCamera(self, obj, cameraObj):

        imageView = self.imageView

        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform() or vtk.vtkTransform())

        localToCameraT = vtk.vtkTransform()
        self.imageQueue.getTransform('local', imageView.imageName, localToCameraT)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(objToLocalT)
        t.Concatenate(localToCameraT)

        pd = filterUtils.transformPolyData(obj.polyData, t)

        '''
        normals = pd.GetPointData().GetNormals()
        cameraToImageT = vtk.vtkTransform()
        imageQueue.getCameraProjectionTransform(imageView.imageName, cameraToImageT)
        pd = filterUtils.transformPolyData(pd, cameraToImageT)
        pts = vnp.getNumpyFromVtk(pd, 'Points')
        pts[:,0] /= pts[:,2]
        pts[:,1] /= pts[:,2]
        pd.GetPointData().SetNormals(normals)
        '''

        self.imageQueue.projectPoints(imageView.imageName, pd)

        cameraObj.setPolyData(pd)

        self.addActorToImageOverlay(cameraObj, imageView)
