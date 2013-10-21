from PythonQt import QtCore, QtGui, QtUiTools
import math
import numpy as np
from ddapp import midi
from ddapp.timercallback import TimerCallback


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def updateComboStrings(combo, strings, defaultSelection):
    currentText = combo.currentText if combo.count else defaultSelection
    combo.clear()
    for text in strings:
        combo.addItem(text)
        if text == currentText:
            combo.setCurrentIndex(combo.count - 1)


def clearLayout(w):
    pass


class IKEditor(object):

    def __init__(self, mainWindow, server, poseCollection, costCollection):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddIKEditor.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile, mainWindow)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.ui.UpdateIKButton.connect('clicked()', self.updateIKClicked)
        self.ui.AutoUpdateCheck.connect('clicked()', self.autoUpdateClicked)

        self.ui.TrajButton.connect('clicked()', self.trajClicked)
        self.ui.IKTrajButton.connect('clicked()', self.ikTrajClicked)
        self.ui.StorePoseButton.connect('clicked()', self.storePoseClicked)
        self.ui.TrajSlider.connect('valueChanged(int)', self.trajSliderChanged)

        self.ui.SeedCombo.connect('currentIndexChanged(const QString&)', self.seedComboChanged)
        self.ui.NominalCombo.connect('currentIndexChanged(const QString&)', self.nominalComboChanged)
        self.ui.CostsCombo.connect('currentIndexChanged(const QString&)', self.costsComboChanged)
        self.ui.GrabCurrentButton.connect('clicked()', self.grabCurrentClicked)

        self.ui.LeftFootEnabled.connect('clicked()', self.leftFootEnabledClicked)
        self.ui.RightFootEnabled.connect('clicked()', self.rightFootEnabledClicked)
        self.ui.ShrinkFactor.connect('valueChanged(double)', self.shrinkFactorChanged)

        self.ui.TargetX.connect('valueChanged(double)', self.positionTargetChanged)
        self.ui.TargetY.connect('valueChanged(double)', self.positionTargetChanged)
        self.ui.TargetZ.connect('valueChanged(double)', self.positionTargetChanged)

        self.ui.OffsetX.connect('valueChanged(double)', self.positionOffsetChanged)
        self.ui.OffsetY.connect('valueChanged(double)', self.positionOffsetChanged)
        self.ui.OffsetZ.connect('valueChanged(double)', self.positionOffsetChanged)

        self.ui.PositionXEnabled.connect('clicked()', self.positionTargetChanged)
        self.ui.PositionYEnabled.connect('clicked()', self.positionTargetChanged)
        self.ui.PositionZEnabled.connect('clicked()', self.positionTargetChanged)

        self.server = server
        self.poseCollection = poseCollection
        self.costCollection = costCollection
        self.midiEditor = MidiOffsetEditor(self)
        self.midiEditor.start()

        self.motionAccumulator = TDxMotionAccumulator(self)
        self.motionAccumulator.start()

        self.frameWidget = None

        poseCollection.connect('itemAdded(const QString&)', self.onPoseAdded)

        self.rebuild()
        self._updateBlocked = False


    def makeFrameWidget(self, view):
        self.frameWidget = WidgetCallback(self, view)
        self.frameWidget.start()

    def onPoseAdded(self):

        def updateCombo(combo):

            currentText = combo.currentText if combo.count else 'q_nom'

            combo.clear()
            for poseName in sorted(self.poseCollection.map().keys()):
                combo.addItem(poseName)
                if poseName == currentText:
                    combo.setCurrentIndex(combo.count - 1)

        updateComboStrings(self.ui.SeedCombo, sorted(self.poseCollection.map().keys()), 'q_end')
        updateComboStrings(self.ui.NominalCombo, sorted(self.poseCollection.map().keys()), 'q_nom')
        updateComboStrings(self.ui.TrajStartPoseCombo, sorted(self.poseCollection.map().keys()), 'q_start')
        updateComboStrings(self.ui.TrajEndPoseCombo, sorted(self.poseCollection.map().keys()), 'q_end')

    def onCostAdded(self):
        updateComboStrings(self.ui.CostsCombo, sorted(self.costCollection.map().keys()), 'default_cost')

    def updateIKClicked(self):
        self.server.updateIk()

    def autoUpdateClicked(self):
        print 'auto update enabled:', self.ui.AutoUpdateCheck.checked
        self.midiEditor.enabled = self.ui.AutoUpdateCheck.checked

    def seedComboChanged(self, name):
        print 'seed:', name
        self.server.seedName = name

    def nominalComboChanged(self, name):
        print 'nominal:', name
        self.server.nominalName = name

    def costsComboChanged(self, name):
        print 'costs:', name

    def grabCurrentClicked(self):
        print 'grab current'

        linkName = self.ui.PositionLinkNameCombo.currentText
        assert linkName == self.server.activePositionConstraint

        self.server.grabCurrentLinkPose(linkName)
        self.positionConstraintComboChanged(linkName)

        if self.frameWidget:
            self.frameWidget.showWithLink(linkName)

    def updateQuasistaticConstraint(self):
        s = self.server

        if self.ui.LeftFootEnabled.checked and self.ui.RightFootEnabled.checked:
            qsc = 'both_feet_qsc'
            feet = ['l_foot', 'r_foot']
        elif self.ui.LeftFootEnabled.checked:
            qsc = 'left_foot_qsc'
            feet = ['l_foot']
        elif self.ui.RightFootEnabled.checked:
            qsc = 'right_foot_qsc'
            feet = ['r_foot']
        else:
            raise Exception('quasistatic constraint requires at least one foot enabled')

        commands = []
        commands.append('%s = QuasiStaticConstraint(r);' % qsc)
        commands.append('%s = %s.setShrinkFactor(%f);' % (qsc, qsc, self.ui.ShrinkFactor.value))
        for foot in feet:
            commands.append('%s = %s.addContact(%s, %s_pts);' % (qsc, qsc, foot, foot))
        commands.append('%s = %s.setActive(true);' % (qsc, qsc))

        self.server.quasiStaticConstraintName = qsc
        self.server.comm.sendCommands(commands)

    def positionTargetChanged(self):
        linkName = self.ui.PositionLinkNameCombo.currentText
        target = np.array([self.ui.TargetX.value, self.ui.TargetY.value, self.ui.TargetZ.value])
        enabledState = [self.ui.PositionXEnabled.checked, self.ui.PositionYEnabled.checked, self.ui.PositionZEnabled.checked]
        self.server.setPositionTarget(linkName, target)
        self.updateIk()

    def positionOffsetChanged(self):
        offset = [self.ui.OffsetX.value, self.ui.OffsetY.value, self.ui.OffsetZ.value]
        linkName = self.ui.PositionLinkNameCombo.currentText
        self.server.setPositionOffset(linkName, offset)
        self.updateIk()

    def updateIk(self):
        if self.ui.AutoUpdateCheck.checked and not self._updateBlocked:
            self.server.updateIk()

    def leftFootEnabledClicked(self):
        self.updateQuasistaticConstraint()

    def rightFootEnabledClicked(self):
        self.updateQuasistaticConstraint()

    def shrinkFactorChanged(self):
        self.updateQuasistaticConstraint()

    def onConstraintClicked(self):

        activeConstraints = []
        for checkBox in self.ui.ActiveConstraintsGroup.findChildren(QtGui.QCheckBox):
            if checkBox.checked:
                activeConstraints.append(checkBox.text)
        self.server.activeConstraintNames = activeConstraints

    def updateActiveConstraints(self):

        s = self.server

        for name in s.constraintNames:
            check = QtGui.QCheckBox(name)
            if name in s.activeConstraintNames:
                check.checked = True
            check.connect('clicked()', self.onConstraintClicked)
            self.ui.ActiveConstraintsGroup.layout().addWidget(check)

    def setPositionOffset(self, offset):
        self._updateBlocked = True
        self.ui.OffsetX.value = offset[0]
        self.ui.OffsetY.value = offset[1]
        self.ui.OffsetZ.value = offset[2]
        self._updateBlocked = False

    def setPositionTarget(self, target):
        self._updateBlocked = True
        self.ui.TargetX.value = target[0]
        self.ui.TargetY.value = target[1]
        self.ui.TargetZ.value = target[2]
        self._updateBlocked = False

    def positionConstraintComboChanged(self, linkName):
        print 'edit position constraint:', linkName
        self.server.activePositionConstraint = linkName
        target = self.server.getPositionTarget(linkName)
        offset = self.server.getPositionOffset(linkName)
        self.setPositionTarget(target)
        self.setPositionOffset(offset)

        if self.frameWidget:
            self.frameWidget.hide()

    def updateEditPositionConstraint(self):

        s = self.server

        linkNames = []

        for name in s.constraintNames:
            if name.endswith('_position_constraint'):
                linkNames.append(name.replace('_position_constraint', ''))

        updateComboStrings(self.ui.PositionLinkNameCombo, linkNames, 'l_foot')
        self.ui.PositionLinkNameCombo.connect('currentIndexChanged(const QString&)', self.positionConstraintComboChanged)

    def handleTDxMotionEvent(self, motionInfo):
        self.motionAccumulator.handleMotionEvent(motionInfo)

    def trajClicked(self):
        poseStart = self.ui.TrajStartPoseCombo.currentText
        poseEnd = self.ui.TrajEndPoseCombo.currentText
        self.server.makePoseTraj(poseStart, poseEnd)

    def ikTrajClicked(self):
        poseStart = self.ui.TrajStartPoseCombo.currentText
        poseEnd = self.ui.TrajEndPoseCombo.currentText
        self.server.runIkTraj(poseStart, poseEnd)

    def trajSliderChanged(self, value):
        t = value / 100.0
        self.server.sampleTraj(t)

    def storePoseClicked(self):
        poseName = self.ui.StoredPoseName.text
        self.server.storeCurrentPose(poseName)

    def rebuildConstraints(self):
        clearLayout(self.ui.ActiveConstraintsGroup)
        self.updateActiveConstraints()

    def rebuild(self):
        s = self.server

        self.rebuildConstraints()
        self.onPoseAdded()
        self.onCostAdded()
        self.updateEditPositionConstraint()



class MidiOffsetEditor(TimerCallback):

    def __init__(self, editor):
        TimerCallback.__init__(self)
        self.editor = editor
        self.enabled = True
        self.midiMap = [0.0, 0.5]
        self.channelsX = [midi.TriggerFinger.faders[0], midi.TriggerFinger.pads[0], midi.TriggerFinger.dials[0]]
        self.channelsY = [midi.TriggerFinger.faders[1], midi.TriggerFinger.pads[1], midi.TriggerFinger.dials[1]]
        self.channelsZ = [midi.TriggerFinger.faders[3], midi.TriggerFinger.pads[2], midi.TriggerFinger.dials[2]]
        self._initReader()

    def _initReader(self):

        try:
            self.reader = midi.MidiReader()
        except AssertionError:
            self.reader = None

    def _scaleMidiValue(self, midiValue):
        ''' Scales the input midiValue.  The midiValue is between 0 and 127. '''
        scaledValue = self.midiMap[0] + midiValue * (self.midiMap[1] - self.midiMap[0])/127.0
        return scaledValue

    def handleMidiEvents(self):

        messages = self.reader.getMessages()
        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        shouldUpdate = False
        for channel, value in targets.iteritems():

            value = self._scaleMidiValue(value)

            if channel in self.channelsX:
                self.editor.ui.OffsetX.value = value
            elif channel in self.channelsY:
                self.editor.ui.OffsetY.value = value
            elif channel in self.channelsZ:
                self.editor.ui.OffsetZ.value = value


    def tick(self):
        if self.reader:
            self.handleMidiEvents()



class TDxMotionAccumulator(TimerCallback):

    def __init__(self, editor):
        TimerCallback.__init__(self)
        self.editor = editor
        self.translationSensitivity = 0.0001
        self.translationCutoff = 0.0
        self.translation = np.zeros(3)

    def handleMotionEvent(self, motionInfo):

        translation = np.array(motionInfo[0:3])
        angleAxis = motionInfo[3:]

        # swap x and y translation
        translation[0], translation[1] = translation[1], translation[0]

        # flip y
        translation[1] *= -1

        translation *= self.translationSensitivity
        translation[np.abs(translation) < self.translationCutoff] = 0.0
        if translation.any():
            self.translation += translation


    def tick(self):

        if not self.translation.any():
            return


        ui = self.editor.ui
        offset = np.array([ui.OffsetX.value, ui.OffsetY.value, ui.OffsetZ.value])
        offset += self.translation
        self.translation = np.zeros(3)
        offset = np.clip(offset, -1.0, 1.0)

        self.editor.setPositionOffset(offset)
        self.editor.updateIk()



########

import perception

class WidgetCallback(TimerCallback):

    def __init__(self, editor, view):
        TimerCallback.__init__(self)
        self.editor = editor
        self.view = view
        self.transform = None
        self.widget = perception.drc.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.SetInteractor(view.renderWindow().GetInteractor())
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetWorldSize(0.5)

    def showWithLink(self, linkName):

        self.linkName = linkName
        self.transform = self.editor.server.getLinkFrame(linkName)
        self.lastMTime = self.transform.GetMTime()
        self.rep.SetTransform(self.transform)
        self.widget.EnabledOn()
        self.view.render()

    def hide(self):
        self.widget.EnabledOff()
        self.transform = None

    def updateTargets(self):
        wxyz = range(4)
        perception.drc.vtkMultisenseSource.GetBotQuaternion(self.transform, wxyz)
        pos = self.transform.GetPosition()

        self.editor.server.setPositionTarget(self.linkName, pos)
        self.editor.server.setOrientationTarget(self.linkName, wxyz)
        self.editor.updateIk()

    def tick(self):

        if not self.transform:
            return

        mtime = self.transform.GetMTime()
        if mtime == self.lastMTime:
            return

        self.lastMTime = mtime
        self.updateTargets()
