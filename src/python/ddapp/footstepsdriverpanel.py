import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp import objectmodel as om
from ddapp import propertyset
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp.pointpicker import PlacerWidget
from ddapp import segmentation
from ddapp import filterUtils
from ddapp import vtkNumpy as vnp

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

    def __init__(self, driver, robotModel, jointController, irisDriver):

        self.driver = driver
        self.robotModel = robotModel
        self.jointController = jointController

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFootsteps.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.placer = None

        self.irisDriver = irisDriver
        self.region_seed_frames = []

        self.ui = WidgetDict(self.widget.children())

        self.ui.walkingGoalButton.connect("clicked()", self.onNewWalkingGoal)
        self.ui.walkingPlanButton.connect("clicked()", self.onShowWalkingPlan)
        self.ui.stopButton.connect("clicked()", self.onStop)
        self.ui.simulateDrakeButton.connect("clicked()", self.onSimulateDrake)
        self.ui.haltSimulationDrakeButton.connect("clicked()", self.onHaltSimulationDrake)
        self.ui.restoreDefaultsButton.connect("clicked()", lambda: self.applyDefaults())
        self.ui.showWalkingVolumesCheck.connect("clicked()", self.onShowWalkingVolumes)

        ### BDI frame logic
        self.ui.hideBDIButton.setVisible(False)
        self.ui.showBDIButton.setVisible(False)
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)

        self.ui.newRegionSeedButton.connect("clicked()", self.onNewRegionSeed)
        self.ui.autoIRISSegmentationButton.connect("clicked()", self.onAutoIRISSegmentation)
        self._setupPropertiesPanel()


    def onShowWalkingVolumes(self):
        self.driver.show_contact_slices = self.ui.showWalkingVolumesCheck.checked

        folder = om.findObjectByName('walking volumes')
        for obj in folder.children():
            obj.setProperty('Visible', self.driver.show_contact_slices)

    def onNewRegionSeed(self):
        t = self.newWalkingGoalFrame(self.robotModel, distanceForward=0.5)
        self.irisDriver.newTerrainItem(t)

    def onAutoIRISSegmentation(self):
        self.irisDriver.autoIRISSegmentation()

    def _setupPropertiesPanel(self):
        l = QtGui.QVBoxLayout(self.ui.paramsContainer)
        l.setMargin(0)
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.propertiesPanel.setBrowserModeToWidget()
        l.addWidget(self.propertiesPanel)
        self.panelConnector = propertyset.PropertyPanelConnector(self.driver.params.properties, self.propertiesPanel)
        self.driver.params.properties.connectPropertyChanged(self.onPropertyChanged)
        PythonQt.dd.ddGroupBoxHider(self.ui.paramsContainer)

    def onPropertyChanged(self, propertySet, propertyName):
        self.driver.updateRequest()
        if propertyName == 'Defaults':
            self.applyDefaults()

    def applyDefaults(self):
        set_name = self.driver.defaults_map[self.driver.params.properties.defaults]
        for k, v in self.driver.default_step_params[set_name].iteritems():
            self.driver.params.setProperty(k, v)

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


        rep = frameObj.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)
        frameObj.widget.HandleRotationEnabledOff()

        if self.placer:
            self.placer.stop()

        terrain = om.findObjectByName('HEIGHT_MAP_SCENE')
        if terrain:

            pos = np.array(frameObj.transform.GetPosition())

            polyData = filterUtils.removeNonFinitePoints(terrain.polyData)
            if polyData.GetNumberOfPoints():
                polyData = segmentation.labelDistanceToLine(polyData, pos, pos+[0,0,1])
                polyData = segmentation.thresholdPoints(polyData, 'distance_to_line', [0.0, 0.1])
                if polyData.GetNumberOfPoints():
                    pos[2] = np.nanmax(vnp.getNumpyFromVtk(polyData, 'Points')[:,2])
                    frameObj.transform.Translate(pos - np.array(frameObj.transform.GetPosition()))

            d = DebugData()
            d.addSphere((0,0,0), radius=0.03)
            handle = vis.showPolyData(d.getPolyData(), 'walking goal terrain handle', parent=frameObj, visible=True, color=[1,1,0])
            handle.actor.SetUserTransform(frameObj.transform)
            self.placer = PlacerWidget(app.getCurrentRenderView(), handle, terrain)

            def onFramePropertyModified(propertySet, propertyName):
                if propertyName == 'Edit':
                    if propertySet.getProperty(propertyName):
                        self.placer.start()
                    else:
                        self.placer.stop()

            frameObj.properties.connectPropertyChanged(onFramePropertyModified)
            onFramePropertyModified(frameObj, 'Edit')

        frameObj.connectFrameModified(self.onWalkingGoalModified)
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

    def onSimulateDrake(self):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.robotModel.setProperty('Visible', False);
        self.driver.sendWalkingPlanRequest(self.driver.lastFootstepPlan, startPose, waitForResponse=False, req_type='simulate_drake')

    def onHaltSimulationDrake(self):
        om.findObjectByName("drake viewer").findChild("robot 1").setProperty("Visible", False)
        self.robotModel.setProperty('Visible', True);
        self.driver.sendHaltSimulationDrakeRequest()

    ### BDI frame logic
    def onHideBDIButton(self):
        self.driver.showBDIPlan = False
        self.driver.bdiRobotModel.setProperty('Visible', False)
        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)
        folder = om.getOrCreateContainer("BDI adj footstep plan")
        om.removeFromObjectModel(folder)

    def onShowBDIButton(self):
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
