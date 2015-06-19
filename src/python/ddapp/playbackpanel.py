from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.applogic as app
import math
import numpy as np
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp.debugVis import DebugData
from ddapp import robotstate
import ddapp.visualization as vis
import ddapp.vtkAll as vtk
import scipy.interpolate


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def clearLayout(w):
    children = w.findChildren(QtGui.QWidget)
    for child in children:
        child.delete()




class PlaybackPanel(object):

    def __init__(self, planPlayback, playbackRobotModel, playbackJointController, robotStateModel, robotStateJointController, manipPlanner):

        self.planPlayback = planPlayback
        self.playbackRobotModel = playbackRobotModel
        self.playbackJointController = playbackJointController
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.manipPlanner = manipPlanner
        manipPlanner.connectPlanCommitted(self.onPlanCommitted)
        manipPlanner.connectUseSupports(self.updateButtonColor)

        self.autoPlay = True
        self.useOperationColors()

        self.planFramesObj = None
        self.plan = None
        self.poseInterpolator = None
        self.startTime = 0.0
        self.endTime = 1.0
        self.animationTimer = TimerCallback()
        self.animationTimer.targetFps = 60
        self.animationTimer.callback = self.updateAnimation
        self.animationClock = SimpleTimer()

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddPlaybackPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())

        self.ui.viewModeCombo.connect('currentIndexChanged(const QString&)', self.viewModeChanged)
        self.ui.playbackSpeedCombo.connect('currentIndexChanged(const QString&)', self.playbackSpeedChanged)
        self.ui.interpolationCombo.connect('currentIndexChanged(const QString&)', self.interpolationChanged)

        self.ui.samplesSpinBox.connect('valueChanged(int)', self.numberOfSamplesChanged)
        self.ui.playbackSlider.connect('valueChanged(int)', self.playbackSliderValueChanged)

        self.ui.animateButton.connect('clicked()', self.animateClicked)
        self.ui.hideButton.connect('clicked()', self.hideClicked)
        self.ui.executeButton.connect('clicked()', self.executeClicked)
        self.ui.executeButton.setShortcut(QtGui.QKeySequence('Ctrl+Return'))
        self.ui.stopButton.connect('clicked()', self.stopClicked)

        self.ui.executeButton.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.ui.executeButton.connect('customContextMenuRequested(const QPoint&)', self.showExecuteContextMenu)


        self.setPlan(None)
        self.hideClicked()


    def useDevelopmentColors(self):
        self.robotStateModelDisplayAlpha = 0.1
        self.playbackRobotModelUseTextures = True
        self.playbackRobotModelDisplayAlpha = 1

    def useOperationColors(self):
        self.robotStateModelDisplayAlpha = 1
        self.playbackRobotModelUseTextures = False
        self.playbackRobotModelDisplayAlpha = 0.5
    def showExecuteContextMenu(self, clickPosition):

        globalPos = self.ui.executeButton.mapToGlobal(clickPosition)

        menu = QtGui.QMenu()
        menu.addAction('Visualization Only')

        if not self.isPlanFeasible():
            menu.addSeparator()
            if self.isPlanAPlanWithSupports():
                menu.addAction('Execute infeasible plan with supports')
            else:
                menu.addAction('Execute infeasible plan')
        elif self.isPlanAPlanWithSupports():
            menu.addSeparator()
            menu.addAction('Execute plan with supports')

        selectedAction = menu.exec_(globalPos)
        if not selectedAction:
            return

        if selectedAction.text == 'Visualization Only':
            self.executePlan(visOnly=True)
        elif selectedAction.text == 'Execute infeasible plan':
            self.executePlan(overrideInfeasibleCheck=True)
        elif selectedAction.text == 'Execute plan with supports':
            self.executePlan(overrideSupportsCheck=True)
        elif selectedAction.text == 'Execute infeasible plan with supports':
            self.executePlan(overrideInfeasibleCheck=True, overrideSupportsCheck=True)


    def getViewMode(self):
        return str(self.ui.viewModeCombo.currentText)

    def setViewMode(self, mode):
        '''
        Set the mode of the view widget. input arg: 'continous', 'frames', 'hidden'
        e.g. can hide all plan playback with 'hidden'
        '''
        self.ui.viewModeCombo.setCurrentIndex(self.ui.viewModeCombo.findText(mode))

    def getPlaybackSpeed(self):
      s = str(self.ui.playbackSpeedCombo.currentText).replace('x', '')
      if '/' in s:
          n, d = s.split('/')
          return float(n)/float(d)
      return float(s)

    def getInterpolationMethod(self):
        return str(self.ui.interpolationCombo.currentText)

    def getNumberOfSamples(self):
        return self.ui.samplesSpinBox.value

    def viewModeChanged(self):
        viewMode = self.getViewMode()
        if viewMode == 'continuous':
            playbackVisible = True
            samplesVisible = False
        else:
            playbackVisible = False
            samplesVisible = True

        self.ui.samplesLabel.setEnabled(samplesVisible)
        self.ui.samplesSpinBox.setEnabled(samplesVisible)

        self.ui.playbackSpeedLabel.setVisible(playbackVisible)
        self.ui.playbackSpeedCombo.setVisible(playbackVisible)
        self.ui.playbackSlider.setEnabled(playbackVisible)
        self.ui.animateButton.setVisible(playbackVisible)
        self.ui.timeLabel.setVisible(playbackVisible)

        if self.plan:

            if self.getViewMode() == 'continuous' and self.autoPlay:
                self.startAnimation()
            else:
                self.ui.playbackSlider.value = 0
                self.stopAnimation()
                self.updatePlanFrames()


    def playbackSpeedChanged(self):
        self.planPlayback.playbackSpeed = self.getPlaybackSpeed()


    def getPlaybackTime(self):
        sliderValue = self.ui.playbackSlider.value
        return (sliderValue / 1000.0) * self.endTime

    def updateTimeLabel(self):
        playbackTime = self.getPlaybackTime()
        self.ui.timeLabel.text = 'Time: %.2f s' % playbackTime

    def playbackSliderValueChanged(self):
        self.updateTimeLabel()
        self.showPoseAtTime(self.getPlaybackTime())

    def interpolationChanged(self):

        methods = {'linear' : 'slinear',
                   'cubic spline' : 'cubic',
                   'pchip' : 'pchip' }
        self.planPlayback.interpolationMethod = methods[self.getInterpolationMethod()]
        self.poseInterpolator = self.planPlayback.getPoseInterpolatorFromPlan(self.plan)
        self.updatePlanFrames()

    def numberOfSamplesChanged(self):
        self.updatePlanFrames()

    def animateClicked(self):
        self.startAnimation()

    def hideClicked(self):

        if self.ui.hideButton.text == 'hide':
            self.ui.playbackFrame.setEnabled(False)
            self.hidePlan()
            self.ui.hideButton.text = 'show'
            self.ui.executeButton.setEnabled(False)

            if not self.plan:
                self.ui.hideButton.setEnabled(False)
        else:
            self.ui.playbackFrame.setEnabled(True)
            self.ui.hideButton.text = 'hide'
            self.ui.hideButton.setEnabled(True)
            self.ui.executeButton.setEnabled(True)

            self.viewModeChanged()

        self.updateButtonColor()


    def executeClicked(self):
        self.executePlan()


    def executePlan(self, visOnly=False, overrideInfeasibleCheck=False, overrideSupportsCheck=False):
        if visOnly:
            _, poses = self.planPlayback.getPlanPoses(self.plan)
            self.onPlanCommitted(self.plan)
            self.robotStateJointController.setPose('EST_ROBOT_STATE', poses[-1])
        else:
            if (self.isPlanFeasible() or overrideInfeasibleCheck) and (not self.isPlanAPlanWithSupports() or overrideSupportsCheck):
                self.manipPlanner.commitManipPlan(self.plan)


    def onPlanCommitted(self, plan):
        self.setPlan(None)
        self.hideClicked()


    def stopClicked(self):
        self.stopAnimation()
        self.manipPlanner.sendPlanPause()


    def isPlanFeasible(self):
        plan = robotstate.asRobotPlan(self.plan)
        return plan is not None and max(plan.plan_info) < 10

    def getPlanInfo(self, plan):
        plan = robotstate.asRobotPlan(self.plan)
        return max(plan.plan_info)


    def isPlanAPlanWithSupports(self):
        return hasattr(self.plan, 'support_sequence') or self.manipPlanner.publishPlansWithSupports

    def updatePlanFrames(self):

        if self.getViewMode() != 'frames':
            return

        numberOfSamples = self.getNumberOfSamples()

        meshes = self.planPlayback.getPlanPoseMeshes(self.plan, self.playbackJointController, self.playbackRobotModel, numberOfSamples)
        d = DebugData()

        startColor = [0.8, 0.8, 0.8]
        endColor = [85/255.0, 255/255.0, 255/255.0]
        colorFunc = scipy.interpolate.interp1d([0, numberOfSamples-1], [startColor, endColor], axis=0, kind='slinear')

        for i, mesh in reversed(list(enumerate(meshes))):
            d.addPolyData(mesh, color=colorFunc(i))

        pd = d.getPolyData()
        clean = vtk.vtkCleanPolyData()
        clean.SetInput(pd)
        clean.Update()
        pd = clean.GetOutput()

        self.planFramesObj = vis.updatePolyData(d.getPolyData(), 'robot plan', alpha=1.0, visible=False, colorByName='RGB255', parent='planning')
        self.showPlanFrames()


    def showPlanFrames(self):
        self.planFramesObj.setProperty('Visible', True)
        self.robotStateModel.setProperty('Visible', False)
        self.playbackRobotModel.setProperty('Visible', False)


    def startAnimation(self):
        self.showPlaybackModel()
        self.stopAnimation()
        self.ui.playbackSlider.value = 0
        self.animationClock.reset()
        self.animationTimer.start()
        self.updateAnimation()


    def stopAnimation(self):
        self.animationTimer.stop()

    def showPlaybackModel(self):
        self.robotStateModel.setProperty('Visible', True)
        self.playbackRobotModel.setProperty('Visible', True)
        self.playbackRobotModel.setProperty('Textures', self.playbackRobotModelUseTextures)
        self.robotStateModel.setProperty('Alpha', self.robotStateModelDisplayAlpha)
        self.playbackRobotModel.setProperty('Alpha', self.playbackRobotModelDisplayAlpha)
        if self.planFramesObj:
            self.planFramesObj.setProperty('Visible', False)

    def hidePlan(self):
        self.stopAnimation()
        if self.planFramesObj:
            self.planFramesObj.setProperty('Visible', False)
        if self.playbackRobotModel:
            self.playbackRobotModel.setProperty('Visible', False)

        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 1.0)


    def showPoseAtTime(self, time):
        pose = self.poseInterpolator(time)
        self.playbackJointController.setPose('plan_playback', pose)


    def updateAnimation(self):

        tNow = self.animationClock.elapsed() * self.planPlayback.playbackSpeed
        if tNow > self.endTime:
            tNow = self.endTime

        sliderValue = int(1000.0 * tNow / self.endTime)

        self.ui.playbackSlider.blockSignals(True)
        self.ui.playbackSlider.value = sliderValue
        self.ui.playbackSlider.blockSignals(False)
        self.updateTimeLabel()

        self.showPoseAtTime(tNow)
        return tNow < self.endTime


    def updateButtonColor(self):
        if self.ui.executeButton.enabled and self.plan and not self.isPlanFeasible():
            styleSheet = 'background-color:red'
        elif self.ui.executeButton.enabled and self.plan and self.isPlanAPlanWithSupports():
            styleSheet = 'background-color:orange'
        else:
            styleSheet = ''

        self.ui.executeButton.setStyleSheet(styleSheet)


    def setPlan(self, plan):

        self.ui.playbackSlider.value = 0
        self.ui.timeLabel.text = 'Time: 0.00 s'
        self.ui.planNameLabel.text = ''
        self.plan = plan
        self.endTime = 1.0
        self.updateButtonColor()

        if not self.plan:
            return

        planText = 'Plan: %d.  %.2f seconds' % (plan.utime, self.planPlayback.getPlanElapsedTime(plan))
        self.ui.planNameLabel.text = planText

        self.startTime = 0.0
        self.endTime = self.planPlayback.getPlanElapsedTime(plan)
        self.interpolationChanged()
        info = self.getPlanInfo(plan)
        app.displaySnoptInfo(info)

        if self.ui.hideButton.text == 'show':
            self.hideClicked()
        else:
            self.viewModeChanged()

        self.updateButtonColor()

        if self.autoPlay and self.widget.parent() is not None:
            self.widget.parent().show()



def init(planPlayback, playbackRobotModel, playbackJointController, robotStateModel, robotStateJointController, manipPlanner):
    global panel
    panel = PlaybackPanel(planPlayback, playbackRobotModel, playbackJointController, robotStateModel, robotStateJointController, manipPlanner)

    #app.getMainWindow().viewManager().layout().addWidget(panel.widget)
    dock = app.addWidgetToDock(panel.widget, dockArea=QtCore.Qt.BottomDockWidgetArea)
    dock.hide()

    return panel
