from director.segmentation import *
from director import perception
from director import mapsregistrar
import PythonQt
from PythonQt import QtCore, QtGui



def getLumberDimensions(lumberId):

    dimensions = [
                  [0.089, 0.038], # 2x4
                  [0.140, 0.038], # 2x6
                  [0.089, 0.089], # 4x4
                 ]

def getBalsaLumberDimensions(lumberId):

    dimensions = [
                  [0.0762, 0.0381], # 2x4 actually (1.5x3)
                  [0.140, 0.038], # 2x6, not used
                  [0.0762, 0.0762], # 4x4 actually (3x3)
                 ]

    return dimensions[lumberId]


def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b


class SegmentationPanel(object):

    def __init__(self):
        self.panel = QtGui.QWidget()
        self.panel.setWindowTitle('Segmentation Tools')

        self.taskSelection = PythonQt.dd.ddTaskSelection()
        self.taskSelection.connect('taskSelected(int)', self.onTaskSelected)

        l = QtGui.QVBoxLayout(self.panel)
        self.backButton = self._makeBackButton()
        l.addWidget(self.backButton)
        l.addWidget(self.taskSelection)
        self.backButton.hide()

        wizards = {
                    'driving' : self._makeDrivingWizard,
                    'terrain' : self._makeTerrainWizard,
                    'ladder' : self._makeLadderWizard,
                    'debris' : self._makeDebrisWizard,
                    'door' : self._makeDoorWizard,
                    'drill' : self._makeDrillWizard,
                    'valve' : self._makeValveWizard,
                    'firehose' : self._makeFirehoseWizard,
                  }

        self.wizards = {}
        for name, func in wizards.items():
          widget = func()
          self.wizards[name] = widget
          l.addWidget(widget)
          widget.hide()

    def _makeDebrisWizard(self):
        debrisWizard = QtGui.QWidget()
        lumberSelection = PythonQt.dd.ddLumberSelection()
        lumberSelection.connect('lumberSelected(int)', self.onDebrisLumberSelected)
        l = QtGui.QVBoxLayout(debrisWizard)
        l.addWidget(lumberSelection)
        #l.addWidget(_makeButton('segment cinderblock wall', startSegmentDebrisWall))
        #l.addWidget(_makeButton('segment cinderblock wall manual', startSegmentDebrisWallManual))
        l.addWidget(_makeButton('segment truss', startTrussSegmentation))

        self.lockAffordanceButton = _makeButton('lock affordance to hand', self.onLockAffordanceToHand)
        self.lockAffordanceButton.checkable = True
        l.addWidget(self.lockAffordanceButton)

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

    def _makeDoorWizard(self):
        wizard = QtGui.QWidget()
        l = QtGui.QVBoxLayout(wizard)
        l.addWidget(_makeButton('segment door handle - left', functools.partial(startDoorHandleSegmentation, 'door_handle_left')))
        l.addWidget(_makeButton('segment door handle - right', functools.partial(startDoorHandleSegmentation, 'door_handle_right')))
        l.addWidget(_makeButton('segment door frame', functools.partial(startDoorHandleSegmentation, 'doorframe')))
        l.addStretch()
        return wizard

    def _makeDrivingWizard(self):
        wizard = QtGui.QWidget()
        l = QtGui.QVBoxLayout(wizard)
        l.addStretch()
        return wizard

    def _makeLadderWizard(self):
        wizard = QtGui.QWidget()
        l = QtGui.QVBoxLayout(wizard)
        l.addStretch()
        return wizard

    def _makeValveWizard(self):
        wizard = QtGui.QWidget()
        l = QtGui.QVBoxLayout(wizard)


        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        self.valveMethodCombo = QtGui.QComboBox()
        self.valveMethodCombo.addItem('auto wall')
        self.valveMethodCombo.addItem('manual')
        hl.addWidget(QtGui.QLabel('method:'))
        hl.addWidget(self.valveMethodCombo)
        l.addWidget(hw)

        l.addWidget(_makeButton('segment large valve', self.segmentLargeValve))
        l.addWidget(_makeButton('segment small valve', self.segmentSmallValve))
        l.addWidget(_makeButton('segment bar', self.segmentLeverValve))

        hw = QtGui.QFrame()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        hl.addWidget(_makeButton('segment value radius:', self.segmentValveCustomRadius))
        self.valveRadiusSpin = QtGui.QSpinBox()
        self.valveRadiusSpin.setMinimum(0)
        self.valveRadiusSpin.setMaximum(99)
        self.valveRadiusSpin.setSingleStep(1)
        hl.addWidget(self.valveRadiusSpin)
        hl.addWidget(QtGui.QLabel('cm'))
        l.addWidget(hw)

        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('refit wall', startRefitWall))

        hw = QtGui.QFrame()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
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

        l.addWidget(_makeButton('segment drill aligned with table', startDrillAutoSegmentationAlignedWithTable))
        l.addWidget(_makeButton('segment target using wall center', segmentDrillWallFromWallCenter))

        self.drillUpdater = TimerCallback()
        self.drillUpdater.targetFps = 30
        self.drillUpdater.callback = self.moveDrillToHand

        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        self.moveDrillToHandButton = _makeButton('move drill to hand', self.onMoveDrillToHandClicked)
        self.moveDrillToHandButton.checkable = True
        hl.addWidget(self.moveDrillToHandButton)
        self.handCombo = QtGui.QComboBox()
        self.handCombo.addItem('left')
        self.handCombo.addItem('right')
        hl.addWidget(self.handCombo)
        hl.addWidget(_makeButton('flip z', self.flipDrill))
        self.drillFlip = False
        l.addWidget(hw)

        # In Degrees
        self.drillRotationSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.drillRotationSlider.setMinimum(-90)
        self.drillRotationSlider.setMaximum(270)
        self.drillRotationSlider.setValue(90)

        # All in MM
        self.drillOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.drillOffsetSlider.setMinimum(-100)
        self.drillOffsetSlider.setMaximum(100)
        self.drillOffsetSlider.setValue(30)

        self.drillDepthOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.drillDepthOffsetSlider.setMinimum(-10)
        self.drillDepthOffsetSlider.setMaximum(90)
        self.drillDepthOffsetSlider.setValue(40)

        self.drillLateralOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.drillLateralOffsetSlider.setMinimum(-50)
        self.drillLateralOffsetSlider.setMaximum(50)
        self.drillLateralOffsetSlider.setValue(-23)

        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        hl.addWidget(self.drillRotationSlider)
        hl.addWidget(self.drillOffsetSlider)
        hw.connect(self.drillRotationSlider, 'valueChanged(int)', self.moveDrillToHand)
        hw.connect(self.drillOffsetSlider, 'valueChanged(int)', self.moveDrillToHand)
        l.addWidget(hw)

        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        hl.addWidget(self.drillDepthOffsetSlider)
        hl.addWidget(self.drillLateralOffsetSlider)
        hw.connect(self.drillDepthOffsetSlider, 'valueChanged(int)', self.moveDrillToHand)
        hw.connect(self.drillLateralOffsetSlider, 'valueChanged(int)', self.moveDrillToHand)
        l.addWidget(hw)



        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.setMargin(0)
        hl.addWidget( _makeButton('segment button', startDrillButtonSegmentation) )
        hl.addWidget( _makeButton('segment tip', startPointerTipSegmentation) )
        self.drillFlip = False
        l.addWidget(hw)
        l.addWidget(QtGui.QLabel(''))


        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.addWidget(QtGui.QLabel('right angle:'))
        hl.setMargin(0)
        self.rightAngleCombo = QtGui.QComboBox()
        self.rightAngleCombo.addItem(DRILL_TRIANGLE_BOTTOM_LEFT)
        self.rightAngleCombo.addItem(DRILL_TRIANGLE_BOTTOM_RIGHT)
        self.rightAngleCombo.addItem(DRILL_TRIANGLE_TOP_LEFT)
        self.rightAngleCombo.addItem(DRILL_TRIANGLE_TOP_RIGHT)
        hl.addWidget(self.rightAngleCombo)
        l.addWidget(hw)

        l.addWidget(_makeButton('segment drill on table', startDrillAutoSegmentation))
        l.addWidget(_makeButton('segment wall', self.segmentDrillWallConstrained))
        l.addWidget(_makeButton('refit wall', startRefitWall))



        l.addWidget(QtGui.QLabel(''))

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
        l.setMargin(0)
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
        l.setMargin(0)
        l.addWidget(w)
        l.addStretch()
        return frame


    def onBackButton(self):
        self.cancelCurrentTask()

    def onSegmentWye(self):
        startWyeSegmentation()

    def onDebrisLumberSelected(self, lumberId):
        blockDimensions = getBalsaLumberDimensions(lumberId)
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

    def startTask(self, taskName):
        self._showTaskWidgets(self.wizards[taskName])

    def startValveSegmentation(self, radius):

        method = self.valveMethodCombo.currentText

        if method == 'auto wall':
            func = startValveSegmentationByWallPlane
        elif method == 'manual':
            func = startValveSegmentationManual

        func(radius)


    def segmentLargeValve(self):

        radiusInInches = 7.7
        inchesToMeters = 0.0254
        self.startValveSegmentation(radiusInInches * inchesToMeters)

    def segmentSmallValve(self):

        radiusInInches = 4.0
        inchesToMeters = 0.0254
        self.startValveSegmentation(radiusInInches * inchesToMeters)

    def segmentValveCustomRadius(self):
        valveRadius = self.valveRadiusSpin.value / 100.0
        self.startValveSegmentation(valveRadius)

    def segmentLeverValve(self):
        startLeverValveSegmentation()

    def segmentDrillWallConstrained(self):
        rightAngleLocation = str(self.rightAngleCombo.currentText)
        startDrillWallSegmentationConstrained(rightAngleLocation)

    def getDrillInHandParams(self):
        ''' All linear dimensions are stored as mm '''
        rotation = self.drillRotationSlider.value
        offset = self.drillOffsetSlider.value/1000.0
        depthOffset = self.drillDepthOffsetSlider.value/1000.0
        lateralOffset = self.drillLateralOffsetSlider.value/1000.0
        flip=self.drillFlip
        return rotation, offset, depthOffset, lateralOffset, flip

    def setDrillInHandParams(self,rotation, offset, depthOffset, lateralOffset, flip):
        self.drillRotationSlider.value = rotation
        self.drillOffsetSlider.value = offset*1000.0
        self.drillDepthOffsetSlider.value = depthOffset*1000.0
        self.drillLateralOffsetSlider.value = lateralOffset*1000.0
        self.drillFlip = flip

    def moveDrillToHand(self):
        hand = self.handCombo.currentText
        rotation, offset, depthoffset, lateralOffset, flip = self.getDrillInHandParams()

        self.drillOffset = getDrillInHandOffset(zRotation=rotation, zTranslation=offset, xTranslation=depthoffset, yTranslation=lateralOffset, flip=flip)
        moveDrillToHand(self.drillOffset, hand)


    def onMoveDrillToHandClicked(self):
        enabled = self.moveDrillToHandButton.isChecked()
        if enabled:
            self.drillUpdater.start()
        else:
            self.drillUpdater.stop()

    def onLockAffordanceToHand(self):
        lockEnabled = self.lockAffordanceButton.isChecked()
        if lockEnabled:
            lockToHandOn()
        else:
            lockToHandOff()

    def flipDrill(self):
        self.drillFlip = not self.drillFlip
        self.moveDrillToHand()

    def cancelCurrentTask(self):
        for w in list(self.wizards.values()):
            w.hide()
        self.backButton.hide()
        self.taskSelection.show()

    def onTaskSelected(self, taskId):

        tasks = {
                   1: 'driving',
                   2: 'terrain',
                   3: 'ladder',
                   4: 'debris',
                   5: 'door',
                   6: 'drill',
                   7: 'valve',
                   8: 'firehose',
                  }

        taskName = tasks[taskId+1]
        self.startTask(taskName)


def createDockWidget():
    global _segmentationPanel, _dock
    try: _segmentationPanel
    except NameError:
        _segmentationPanel = SegmentationPanel()
        _dock = app.addWidgetToDock(_segmentationPanel.panel)
        _dock.hide()



def getOrCreateSegmentationView():

    viewManager = app.getViewManager()
    segmentationView = viewManager.findView('Segmentation View')
    if not segmentationView:
        segmentationView = viewManager.createView('Segmentation View', 'VTK View')
        installEventFilter(segmentationView, segmentationViewEventFilter)

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



def activateSegmentationMode(polyData=None, debug=False):

    if debug:
        polyData = getDebugRevolutionData()
    elif polyData is None:
        if not perception._multisenseItem.getProperty('Visible'):
            return False
        polyData = getCurrentMapServerData() or getCurrentRevolutionData()

    if not polyData:
        return False

    segmentationView = getOrCreateSegmentationView()
    app.getViewManager().switchToView('Segmentation View')

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

    _dock.show()

    return True


def init():

    getOrCreateSegmentationView()
    createDockWidget()
    #mapsregistrar.initICPCallback()

    #activateSegmentationMode(debug=True)
