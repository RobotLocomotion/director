from PythonQt import QtCore, QtGui, QtUiTools
import math
import numpy as np


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


        self.ui.SeedCombo.connect('currentIndexChanged(const QString&)', self.seedComboChanged)
        self.ui.NominalCombo.connect('currentIndexChanged(const QString&)', self.nominalComboChanged)
        self.ui.CostsCombo.connect('currentIndexChanged(const QString&)', self.costsComboChanged)
        self.ui.GrabCurrentButton.connect('clicked()', self.grabCurrentClicked)

        self.ui.LeftFootEnabled.connect('clicked()', self.leftFootEnabledClicked)
        self.ui.RightFootEnabled.connect('clicked()', self.rightFootEnabledClicked)
        self.ui.ShrinkFactor.connect('valueChanged(double)', self.shrinkFactorChanged)
        self.server = server
        self.poseCollection = poseCollection
        self.costCollection = costCollection

        poseCollection.connect('itemAdded(const QString&)', self.onPoseAdded)

        self.rebuild()


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


    def onCostAdded(self):
        updateComboStrings(self.ui.CostsCombo, sorted(self.costCollection.map().keys()), 'default_cost')


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

        print '\n'.join(commands)

        s.quasiStaticConstraintName = qsc
        s.comm.sendCommands(commands)


    def leftFootEnabledClicked(self):
        print 'left foot:', self.ui.LeftFootEnabled.checked
        self.updateQuasistaticConstraint()

    def rightFootEnabledClicked(self):
        print 'right foot:', self.ui.RightFootEnabled.checked
        self.updateQuasistaticConstraint()

    def shrinkFactorChanged(self):
        print 'shrink factor:', self.ui.ShrinkFactor.value
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


    def rebuildConstraints(self):
        clearLayout(self.ui.ActiveConstraintsGroup)
        self.updateActiveConstraints()

    def rebuild(self):
        s = self.server

        self.rebuildConstraints()
        self.onPoseAdded()
        self.onCostAdded()
