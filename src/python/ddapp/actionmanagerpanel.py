import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math
from time import time
from copy import deepcopy

from actionmanager import sequences
from actionmanager import actionsequence

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class ActionManagerPanel(object):

    def __init__(self, actionSequence):

        #Store local variables
        self.sequenceController = actionSequence
        self.currentAction = ''

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddActionManager.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.targetFps = 5
        self.updateTimer.start()

        self.populateSequenceSelector()

        #Connect signals
        self.widget.sequenceSelector.sequenceTree.currentItemChanged.connect(self.selectionChanged)
        self.widget.sequenceSelector.updateButton.clicked.connect(self.updateClicked)

        self.widget.statusDisplay.createButton.clicked.connect(self.createActionSequence)
        self.widget.statusDisplay.visualizeButton.clicked.connect(self.visualizeActionSequence)
        self.widget.statusDisplay.executeButton.clicked.connect(self.runActionSequence)
        self.widget.statusDisplay.resetButton.clicked.connect(self.resetActionSequence)
        self.widget.statusDisplay.stopButton.clicked.connect(self.stopActionSequence)

	self.updatePanel()


    def updatePanel(self):
    	self.widget.statusDisplay.nameLabel.text = self.sequenceController.name

        if self.sequenceController.fsm:
            self.widget.statusDisplay.stateLabel.text = self.sequenceController.fsm.current
            if self.sequenceController.fsm.started:
                self.widget.statusDisplay.runningLabel.text = 'True'
            else:
                self.widget.statusDisplay.runningLabel.text = 'False'
        else:
            self.widget.statusDisplay.stateLabel.text = 'unknown'
            self.widget.statusDisplay.stateLabel.text = 'unknown'

    def createActionSequence(self):
        if self.currentAction != '':
            if actionsequence.checkArgsPopulated(sequences.sequenceDict[self.currentAction][0]):
                self.sequenceController.populate(self.currentAction,
                                                 sequences.sequenceDict[self.currentAction][0],
                                                 sequences.sequenceDict[self.currentAction][1])
                self.widget.statusDisplay.statusLabel.text = ''
            else:
                self.widget.statusDisplay.statusLabel.text = 'ERROR: Selected sequence has unpopulated user fields'

    def visualizeActionSequence(self):
        if self.currentAction != '':
            if self.sequenceController.fsm and self.sequenceController.fsm.current == 'goal':
                self.sequenceController.play()
            else:
                self.sequenceController.start(vizMode = True)


    def runActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.start(vizMode = False)

    def resetActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.reset()

    def stopActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.stop()


    def populateSequenceSelector(self):
        #Populate this widget with sequences from the python library
        self.treeItems = {}

        for (seqName, seq, startPoint, typeName) in sequences.sequenceList:
            if typeName == 'Primitive':
                highLevelArgs = actionsequence.generateUserArgDict(seq)
                item = QtGui.QTreeWidgetItem()
                item.setText(0, seqName)
                item.setText(1, typeName)
                if seq[seqName][3] != []:
                    item.setText(2, ", ".join(highLevelArgs.keys()))
                    item.setText(3, ", ".join([highLevelArgs[val] for val in highLevelArgs.keys()]))
                else:
                    item.setText(2, "None")
                self.widget.sequenceSelector.sequenceTree.addTopLevelItem(item)
                self.treeItems[seqName] = [item, None]
            else:
                highLevelArgs = actionsequence.generateUserArgList(seq)
                item = QtGui.QTreeWidgetItem()
                item.setText(0, seqName)
                item.setText(1, typeName)
                item.setText(2, ", ".join(highLevelArgs))
                self.widget.sequenceSelector.sequenceTree.addTopLevelItem(item)

                self.treeItems[seqName] = [item, {}]

                for subAction in seq.keys():
                    child = QtGui.QTreeWidgetItem(item)
                    child.setText(0, subAction)
                    child.setText(1, seq[subAction][0].__name__)
                    argDict = seq[subAction][3]
                    if argDict != {}:
                        inputNames = seq[subAction][3].keys()
                        inputArgs = seq[subAction]
                        child.setText(2, ", ".join(argDict.keys()))
                        child.setText(3, ", ".join([argDict[val] for val in argDict.keys()]))
                    else:
                        child.setText(2, "None")
                    self.treeItems[seqName][1][subAction] = child

    def updateClicked(self, event):

        #Extract the data from the edit boxes
        newDataDict = {}

        for i in range(len(self.widget.argumentEditor.findChildren(QtGui.QWidget))-1):
            children = self.widget.argumentEditor.findChildren(QtGui.QWidget)
            child1 = children[i]
            child2 = children[i+1]
            if isinstance(child1, QtGui.QLabel) and isinstance(child2, QtGui.QLineEdit):
                newDataDict[str(child1.text)] = str(child2.text)

        #Go to the entry for the current action in the sequence dictionary and save the data
        for subAction in sequences.sequenceDict[self.currentAction][0].keys():
            for arg in sequences.sequenceDict[self.currentAction][0][subAction][3]:
                if arg in newDataDict.keys():
                    sequences.sequenceDict[self.currentAction][0][subAction][3][arg] = newDataDict[arg]

        #Repopulate the sequence selector with this data
        self.widget.sequenceSelector.sequenceTree.clear()
        self.populateSequenceSelector()


    def clearArgEditor(self):
        for child in self.widget.argumentEditor.findChildren(QtGui.QWidget):
            child.delete()


    def selectionChanged(self, event):
        if not event == None:

            #Set the new action
            self.currentAction = str(event.text(0))

            #Clear the old arguments editor
            self.clearArgEditor()

            if self.currentAction in sequences.sequenceDict.keys():
                #Create a grid of arguments and boxes to edit them
                grid = self.widget.argumentEditor.layout()
                argList = actionsequence.generateUserArgDict(sequences.sequenceDict[self.currentAction][0])
                for count, key in enumerate(argList.keys()):
                    grid.addWidget(QtGui.QLabel(key),count,0)
                    grid.addWidget(QtGui.QLineEdit(argList[key]),count,1)

        else:
            self.currentAction = ''
            self.clearArgEditor()

        #self.widget.argumentEditor.affordanceList.clear()
        #self.widget.argumentEditor.constraintList.clear()

        #self.widget.argumentEditor.affordanceList.addItem(event.text(2))
        #self.widget.argumentEditor.constraintList.addItem(event.text(2))


def toggleWidgetShow():

    if dock.isVisible():
        dock.hide()
    else:
        dock.show()

def init(actionSeq):

    global dock

    panel = ActionManagerPanel(actionSeq)
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()

    actionName = 'ActionActionManagerPanel'
    action = app.getToolBarActions()[actionName]
    action.triggered.connect(toggleWidgetShow)


    return panel
