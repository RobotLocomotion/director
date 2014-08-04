import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp.terrainitem import TerrainRegionItem
try:
    import mosek
    from ddapp.terrain import TerrainSegmentation
    _mosekEnabled = True
except ImportError:
    print "Warning: mosek python bindings not found. Terrain segmentation disabled."
    _mosekEnabled = False

import numpy as np

def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)

class FootstepsPanel(object):

    def __init__(self, driver, robotModel, jointController, mapServerSource):

        self.driver = driver
        self.robotModel = robotModel
        self.jointController = jointController

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFootsteps.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.depth_provider = mapServerSource
        if _mosekEnabled:
            self.terrain_segmentation = TerrainSegmentation(bounding_box_width=2)
            self.driver.contact_slices = self.terrain_segmentation.contact_slices
        else:
            self.terrain_segmentation = None
        self.region_seed_frames = []

        self.ui = WidgetDict(self.widget.children())

        self.ui.walkingGoalButton.connect("clicked()", self.onNewWalkingGoal)
        self.ui.walkingPlanButton.connect("clicked()", self.onShowWalkingPlan)
        self.ui.executeButton.connect("clicked()", self.onExecute)
        self.ui.stopButton.connect("clicked()", self.onStop)
        self.ui.BDIDefaultsButton.connect("clicked()", lambda: self.applyDefaults('BDI'))
        self.ui.drakeDefaultsButton.connect("clicked()", lambda: self.applyDefaults('drake'))
        self.ui.showWalkingVolumesCheck.connect("clicked()", self.onShowWalkingVolumes)

        ### BDI frame logic
        #self.ui.hideBDIButton.setVisible(False)
        #self.ui.showBDIButton.setVisible(False)
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)
        self.ui.newRegionSeedButton.connect("clicked()", self.onNewRegionSeed)
        self._setupPropertiesPanel()


    def onShowWalkingVolumes(self):
        self.driver.show_contact_slices = self.ui.showWalkingVolumesCheck.checked

        folder = om.findObjectByName('walking volumes')
        for obj in folder.children():
            obj.setProperty('Visible', self.driver.show_contact_slices)

    def onNewRegionSeed(self):
        if not _mosekEnabled:
            print "Warning: Mosek python bindings not found. Terrain segmentation is disabled."
            return
        if self.terrain_segmentation is not None:
            heights, world2px = self.depth_provider.getSceneHeightData()
            if heights is None:
                heights = np.zeros((100,100))
                world2px = np.eye(4)
            heights[np.isinf(heights)] = np.nan
            px2world = np.linalg.inv(world2px)
            self.terrain_segmentation.setHeights(heights, px2world)

            t = self.newWalkingGoalFrame(self.robotModel, distanceForward=0.5)
            view = app.getCurrentRenderView()
            item = TerrainRegionItem('IRIS region', view, t,
                                     self.terrain_segmentation)
            parentObj = om.getOrCreateContainer('Safe terrain regions')
            om.addToObjectModel(item, parentObj)

    def _setupPropertiesPanel(self):
        l = QtGui.QVBoxLayout(self.ui.paramsContainer)
        l.setMargin(0)
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.propertiesPanel.setBrowserModeToWidget()
        om.PropertyPanelHelper.addPropertiesToPanel(self.driver.params.properties, self.propertiesPanel)
        l.addWidget(self.propertiesPanel)
        self.propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', self.onPropertyChanged)
        PythonQt.dd.ddGroupBoxHider(self.ui.paramsContainer)

    def onPropertyChanged(self, prop):
        self.driver.params.setProperty(prop.propertyName(), prop.value())
        self.driver.updateRequest()

    def applyDefaults(self, set_name):
        for k, v in self.driver.default_step_params[set_name].iteritems():
            self.driver.params.setProperty(k, v)
            om.PropertyPanelHelper.onPropertyValueChanged(self.propertiesPanel, self.driver.params.properties, k)

    def newWalkingGoalFrame(self, robotModel, distanceForward=1.0):
        t = self.driver.getFeetMidPoint(robotModel)
        t.PreMultiply()
        t.Translate(distanceForward, 0.0, 0.0)
        t.PostMultiply()
        return t


    def onNewWalkingGoal(self, walkingGoal=None):

        walkingGoal = walkingGoal or self.newWalkingGoalFrame(self.robotModel)
        frameObj = vis.updateFrame(walkingGoal, 'walking goal', parent='planning', scale=0.25)
        frameObj.setProperty('Edit', True)
        frameObj.connectFrameModified(self.onWalkingGoalModified)

        rep = frameObj.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)
        frameObj.widget.HandleRotationEnabledOff()

        self.onWalkingGoalModified(frameObj)

    def onWalkingGoalModified(self, frame):

        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        request = self.driver.constructFootstepPlanRequest(self.jointController.q, frame.transform)
        self.driver.sendFootstepPlanRequest(request)

    def onExecute(self):
        self.driver.commitFootstepPlan(self.driver.lastFootstepPlan)

    def onShowWalkingPlan(self):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.driver.sendWalkingPlanRequest(self.driver.lastFootstepPlan, startPose, waitForResponse=True)

    def onStop(self):
        self.driver.sendStopWalking()


    ### BDI frame logic
    def onHideBDIButton(self):
        print "hide bdi"
        self.driver.showBDIPlan = False
        self.driver.bdiRobotModel.setProperty('Visible', False)
        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)
        folder = om.getOrCreateContainer("BDI adj footstep plan")
        om.removeFromObjectModel(folder)

    def onShowBDIButton(self):
        print "show bdi"
        self.driver.showBDIPlan = True
        self.driver.bdiRobotModel.setProperty('Visible', True)
        self.driver.drawBDIFootstepPlan()
        self.driver.drawBDIFootstepPlanAdjusted()

def _getAction():
    return app.getToolBarActions()['ActionFootstepPanel']

def init(*args, **kwargs):

    global panel
    global dock

    panel = FootstepsPanel(*args, **kwargs)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
