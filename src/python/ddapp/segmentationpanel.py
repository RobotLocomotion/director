from ddapp.segmentation import *

from ddapp import mapsregistrar
from ddapp import drilltaskpanel
import PythonQt
from PythonQt import QtCore, QtGui



def getLumberDimensions(lumberId):

    dimensions = [
                  [0.089, 0.038], # 2x4
                  [0.140, 0.038], # 2x6
                  [0.089, 0.089], # 4x4
                 ]

    return dimensions[lumberId]


def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b


class SegmentationPanel(object):

    def __init__(self):
        self.panel = QtGui.QWidget()
        self.taskSelection = PythonQt.dd.ddTaskSelection()
        self.debrisWizard = self._makeDebrisWizard()
        self.terrainWizard = self._makeTerrainWizard()
        self.firehoseWizard = self._makeFirehoseWizard()
        self.valveWizard = self._makeValveWizard()
        self.drillWizard = self._makeDrillWizard()
        self.backButton = self._makeBackButton()
        self.taskSelection.connect('taskSelected(int)', self.onTaskSelected)

        l = QtGui.QVBoxLayout(self.panel)
        l.addWidget(self.backButton)
        l.addWidget(self.taskSelection)
        l.addWidget(self.debrisWizard)
        l.addWidget(self.terrainWizard)
        l.addWidget(self.firehoseWizard)
        l.addWidget(self.drillWizard)
        l.addWidget(self.valveWizard)
        self.backButton.hide()
        self.debrisWizard.hide()
        self.terrainWizard.hide()
        self.firehoseWizard.hide()
        self.valveWizard.hide()
        self.drillWizard.hide()

    def _makeDebrisWizard(self):
        debrisWizard = QtGui.QWidget()
        lumberSelection = PythonQt.dd.ddLumberSelection()
        lumberSelection.connect('lumberSelected(int)', self.onDebrisLumberSelected)
        l = QtGui.QVBoxLayout(debrisWizard)
        l.addWidget(lumberSelection)
        l.addWidget(_makeButton('segment cinderblock wall', startSegmentDebrisWall))
        l.addWidget(_makeButton('segment cinderblock wall manual', startSegmentDebrisWallManual))
        l.addStretch()
        return debrisWizard

    def _makeFirehoseWizard(self):
        firehoseWizard = QtGui.QWidget()
        segmentButton = QtGui.QToolButton()
        segmentButton.setIcon(QtGui.QIcon(':/images/wye.png'))
        segmentButton.setIconSize(QtCore.QSize(60,60))
        segmentButton.connect('clicked()', self.onSegmentWye)
        l = QtGui.QVBoxLayout(firehoseWizard)
        l.addWidget(segmentButton)
        l.addWidget(_makeButton('segment hose nozzle', startHoseNozzleSegmentation))
        l.addStretch()
        return firehoseWizard


    def _makeValveWizard(self):
        wizard = QtGui.QWidget()
        l = QtGui.QVBoxLayout(wizard)
        l.addWidget(_makeButton('segment valve', functools.partial(startValveSegmentationByWallPlane, 0.195)))
        l.addWidget(_makeButton('segment small valve', functools.partial(startValveSegmentationByWallPlane, 0.10)))
        l.addWidget(_makeButton('segment bar', functools.partial(startInteractiveLineDraw, [0.015, 0.015])))

        hw = QtGui.QFrame()
        hl = QtGui.QHBoxLayout(hw)
        hl.addWidget(_makeButton('request valve circle plan', self.requestValveCirclePlan))
        self.circlePlanAngle = QtGui.QSpinBox()
        self.circlePlanAngle.setMinimum(-360)
        self.circlePlanAngle.setMaximum(360)
        self.circlePlanAngle.setSingleStep(5)
        hl.addWidget(self.circlePlanAngle)
        hl.addWidget(QtGui.QLabel('degrees'))
        l.addWidget(hw)

        l.addStretch()
        return wizard


    def _makeDrillWizard(self):
        drillWizard = QtGui.QGroupBox('Drill Segmentation')
        l = QtGui.QVBoxLayout(drillWizard)
        l.addWidget(_makeButton('segment drill on table', startDrillAutoSegmentation))
        l.addWidget(_makeButton('segment drill in hand', startDrillInHandSegmentation))
        l.addWidget(_makeButton('segment wall', startDrillWallSegmentation))
        l.addWidget(_makeButton('select tooltip', startSelectToolTip))
        l.addWidget(QtGui.QLabel(''))

        self.drillTaskPanel = drilltaskpanel.DrillTaskPanel()
        l.addWidget(self.drillTaskPanel.widget)

        l.addStretch()
        return drillWizard


    def _makeTerrainWizard(self):
        terrainWizard = QtGui.QWidget()

        self.cinderBlockButton = QtGui.QToolButton()
        self.cinderBlock2Button = QtGui.QToolButton()
        self.cinderBlockButton.setIcon(QtGui.QIcon(':/images/cinderblock.png'))
        self.cinderBlock2Button.setIcon(QtGui.QIcon(':/images/cinderblock_double.png'))
        self.cinderBlockButton.setIconSize(QtCore.QSize(60,60))
        self.cinderBlock2Button.setIconSize(QtCore.QSize(60,60))

        self.cinderBlockButton.connect('clicked()', functools.partial(self.onTerrainCinderblockSelected, self.cinderBlockButton))
        self.cinderBlock2Button.connect('clicked()', functools.partial(self.onTerrainCinderblockSelected, self.cinderBlock2Button))

        buttons = QtGui.QWidget()
        l = QtGui.QHBoxLayout(buttons)
        l.addStretch()
        l.addWidget(self.cinderBlockButton)
        l.addWidget(self.cinderBlock2Button)
        l.addStretch()

        l = QtGui.QVBoxLayout(terrainWizard)
        l.addWidget(buttons)
        l.addWidget(_makeButton('double wide', functools.partial(startInteractiveLineDraw, [0.1905*2, 0.149225])))


        l.addStretch()
        return terrainWizard

    def _makeBackButton(self):
        w = QtGui.QPushButton()
        w.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_ArrowBack))
        w.connect('clicked()', self.onBackButton)

        frame = QtGui.QWidget()
        l = QtGui.QHBoxLayout(frame)
        l.addWidget(w)
        l.addStretch()
        return frame


    def onBackButton(self):
        self.cancelCurrentTask()

    def onSegmentWye(self):
        startWyeSegmentation()

    def onDebrisLumberSelected(self, lumberId):
        blockDimensions = getLumberDimensions(lumberId)
        startInteractiveLineDraw(blockDimensions)

    def onTerrainCinderblockSelected(self, button):
        if button == self.cinderBlockButton:
            blockDimensions = [0.1905, 0.149225]
        elif button == self.cinderBlock2Button:
            blockDimensions = [0.1905, 0.29845]
        startInteractiveLineDraw(blockDimensions)

    def _showTaskWidgets(self, w):
        self.taskSelection.hide()
        self.backButton.show()
        w.show()

    def startDebrisTask(self):
        self._showTaskWidgets(self.debrisWizard)

    def startTerrainTask(self):
        self._showTaskWidgets(self.terrainWizard)

    def startFirehoseTask(self):
        self._showTaskWidgets(self.firehoseWizard)

    def startDrillTask(self):
        self._showTaskWidgets(self.drillWizard)

    def startValveTask(self):
        self._showTaskWidgets(self.valveWizard)

    def requestValveCirclePlan(self):
        self.drillTaskPanel.valveCirclePlan(self.circlePlanAngle.value)

    def cancelCurrentTask(self):
        self.debrisWizard.hide()
        self.terrainWizard.hide()
        self.firehoseWizard.hide()
        self.drillWizard.hide()
        self.valveWizard.hide()
        self.backButton.hide()
        self.taskSelection.show()

    def onTaskSelected(self, taskId):

        taskFunctions = {
                         2:self.startTerrainTask,
                         4:self.startDebrisTask,
                         8:self.startFirehoseTask,
                         6:self.startDrillTask,
                         7:self.startValveTask,
                        }

        taskFunction = taskFunctions.get(taskId+1)

        if taskFunction:
            taskFunction()
        else:
            app.showInfoMessage('Sorry, not impemented yet.')



def createDockWidget():
    global _segmentationPanel
    try: _segmentationPanel
    except NameError:
        _segmentationPanel = SegmentationPanel()
        app.addWidgetToDock(_segmentationPanel.panel)



def getOrCreateSegmentationView():

    viewManager = app.getViewManager()
    segmentationView = viewManager.findView('Segmentation View')
    if not segmentationView:
        segmentationView = viewManager.createView('Segmentation View')
        installEventFilter(segmentationView, segmentationViewEventFilter)

    viewManager.switchToView('Segmentation View')
    return segmentationView


def onSegmentationViewDoubleClicked(displayPoint):


    action = 'zoom_to'


    if om.findObjectByName('major planes'):
        action = 'select_actor'


    if action == 'zoom_to':

        zoomToDisplayPoint(displayPoint)

    elif action == 'select_with_ray':

        extractPointsAlongClickRay(displayPoint)

    elif action == 'select_actor':
        selectActor(displayPoint)


eventFilters = {}

def segmentationViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]

    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        onSegmentationViewDoubleClicked(mapMousePosition(obj, event))

    else:

        for picker in viewPickers:
            if not picker.enabled:
                continue

            if event.type() == QtCore.QEvent.MouseMove:
                picker.onMouseMove(mapMousePosition(obj, event), event.modifiers())
                eventFilter.setEventHandlerResult(True)
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                picker.onMousePress(mapMousePosition(obj, event), event.modifiers())
                eventFilter.setEventHandlerResult(True)


def drcViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]
    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        activateSegmentationMode()


def installEventFilter(view, func):

    global eventFilters
    eventFilter = PythonQt.dd.ddPythonEventFilter()

    qvtkwidget = view.vtkWidget()
    qvtkwidget.installEventFilter(eventFilter)
    eventFilters[qvtkwidget] = eventFilter

    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonRelease)
    eventFilter.connect('handleEvent(QObject*, QEvent*)', func)



def activateSegmentationMode(debug=False):

    if debug:
        polyData = getDebugRevolutionData()
    else:
        polyData = getCurrentMapServerData() or getCurrentRevolutionData()

    if not polyData:
        return

    segmentationView = getOrCreateSegmentationView()

    perspective()

    mapsregistrar.storeInitialTransform()

    thresholdWorkspace = False
    doRemoveGround = False

    if thresholdWorkspace:
        polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 2.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-3.0, 3.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_z', [-10.0, 1.5])

    if doRemoveGround:
        groundPoints, polyData = removeGround(polyData)
        segmentationObj = updatePolyData(groundPoints, 'ground', alpha=0.3, visible=False)

    segmentationObj = updatePolyData(polyData, 'pointcloud snapshot', alpha=0.3)
    segmentationObj.headAxis = perception._multisenseItem.model.getAxis('head', [1,0,0])

    segmentationView.camera().DeepCopy(app.getDRCView().camera())
    segmentationView.render()

    createDockWidget()


def init():

    installEventFilter(app.getViewManager().findView('DRC View'), drcViewEventFilter)

    mapsregistrar.initICPCallback()

    #activateSegmentationMode(debug=True)
