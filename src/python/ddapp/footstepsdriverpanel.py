import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import roboturdf
from ddapp import visualization as vis
from ddapp.timercallback import TimerCallback
from ddapp.debugVis import DebugData
from irispy.utils import lcon_to_vert, InfeasiblePolytopeError
from scipy.spatial import ConvexHull
from ddapp import segmentation
import ddapp.vtkNumpy as vnp
try:
    import mosek
    from irispy.terrain import TerrainSegmentation
    _mosekEnabled = True
except ImportError:
    print "Warning: mosek python bindings not found. Terrain segmentation disabled."
    _mosekEnabled = False

import numpy as np
import math


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
        self.ui.BDIExecuteButton.connect("clicked()", self.onBDIExecute)
        self.ui.drakeExecuteButton.connect("clicked()", self.onDrakeExecute)
        self.ui.stopButton.connect("clicked()", self.onStop)
        self.ui.BDIDefaultsButton.connect("clicked()", lambda: self.applyDefaults('BDI'))
        self.ui.drakeDefaultsButton.connect("clicked()", lambda: self.applyDefaults('drake'))
        self.ui.showWalkingVolumesCheck.connect("clicked()", self.onShowWalkingVolumes)

        ### BDI frame logic
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)
        self.ui.newRegionSeedButton.connect("clicked()", self.onNewRegionSeed)
        self._setupPropertiesPanel()


    def onShowWalkingVolumes(self):
        self.driver.show_contact_slices = self.ui.showWalkingVolumesCheck.checked

        # TODO: instead of deleting or regernating these, we can just
        # show/hide them in the om. This requires a feature that Pat will
        # implement soon, to show/hide an entire folder's contents
        if self.ui.showWalkingVolumesCheck.checked:
            self.driver.updateRequest()
        else:
            om.removeFromObjectModel(om.findObjectByName('walking volumes'))

    def onNewRegionSeed(self):
        if not _mosekEnabled:
            print "Warning: Mosek python bindings not found. Terrain segmentation is disabled."
            return
        if self.terrain_segmentation is not None:
            t = self.newWalkingGoalFrame(self.robotModel)
            idx = len(self.region_seed_frames)
            frameObj = vis.updateFrame(t, 'region seed {:d}'.format(idx), parent='planning', scale=0.25)
            frameObj.index = idx
            frameObj.setProperty('Edit', True)
            frameObj.connectFrameModified(self.onRegionSeedModified)
            self.region_seed_frames.append(frameObj)
            heights, world2px = self.depth_provider.getSceneHeightData()
            heights[np.isinf(heights)] = np.nan
            print heights.shape
            print world2px
            px2world = np.linalg.inv(world2px)
            self.terrain_segmentation.setHeights(heights, px2world)

            self.onRegionSeedModified(frameObj)

    def onRegionSeedModified(self, frame):
        pos, wxyz = transformUtils.poseFromTransform(frame.transform)
        rpy = transformUtils.rollPitchYawFromTransform(frame.transform)
        pose = np.hstack((pos, rpy))
        safe_region = self.terrain_segmentation.findSafeRegion(pose, iter_limit=2)
        debug = DebugData()
        om_name = 'IRIS region boundary {:d}'.format(frame.index)
        om.removeFromObjectModel(om.findObjectByName(om_name))
        try:
            V = lcon_to_vert(safe_region.A, safe_region.b)
            hull = ConvexHull(V[:2,:].T)
            z = pos[2]
            for j, v in enumerate(hull.vertices):
                p1 = np.hstack((V[:2,hull.vertices[j]], z))
                if j < len(hull.vertices) - 1:
                    p2 = np.hstack((V[:2,hull.vertices[j+1]], z))
                else:
                    p2 = np.hstack((V[:2,hull.vertices[0]], z))
                debug.addLine(p1, p2, color=[.8,.8,.2])
            vis.showPolyData(debug.getPolyData(), om_name, parent='planning', color=[.8,.8,.2])
        except InfeasiblePolytopeError:
            print "Infeasible polytope"

        if frame.index < len(self.driver.safe_terrain_regions):
            self.driver.safe_terrain_regions[frame.index] = safe_region
        else:
            assert frame.index == len(self.driver.safe_terrain_regions)
            self.driver.safe_terrain_regions.append(safe_region)


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
        t = transformUtils.frameFromPositionAndRPY(t.GetPosition(), [0.0, 0.0, t.GetOrientation()[2]])
        t.PreMultiply()
        t.Translate(distanceForward, 0.0, 0.0)
        t.PostMultiply()
        return t


    def onNewWalkingGoal(self):

        t = self.newWalkingGoalFrame(self.robotModel)
        frameObj = vis.updateFrame(t, 'walking goal', parent='planning', scale=0.25)
        frameObj.setProperty('Edit', True)
        frameObj.connectFrameModified(self.onWalkingGoalModified)
        self.onWalkingGoalModified(frameObj)

    def onWalkingGoalModified(self, frame):

        request = self.driver.constructFootstepPlanRequest(self.jointController.q, frame.transform)
        self.driver.sendFootstepPlanRequest(request)

    def onBDIExecute(self):
        self.driver.commitFootstepPlan(self.driver.lastFootstepPlan)

    def onDrakeExecute(self):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.driver.sendWalkingControllerRequest(self.driver.lastFootstepPlan, startPose, waitForResponse=True)

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
