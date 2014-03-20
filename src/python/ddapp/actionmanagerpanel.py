import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math
import functools
from time import time
from copy import copy, deepcopy

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

        self.templateTreeItems = {}
        self.savedTreeItems = {}
        self.oldExecutionListLength = 0

        self.populateTreeWidget(self.widget.actionSequences.templateTree, sequences.sequenceDict, self.templateTreeItems)

        #Connect signals
        self.widget.actionSequences.templateTree.currentItemChanged.connect(self.selectionChanged)
        self.ui.saveButton.clicked.connect(self.saveClicked)

        self.widget.statusDisplay.createButton.clicked.connect(self.createActionSequence)
        self.widget.statusDisplay.visualizeButton.clicked.connect(self.visualizeActionSequence)
        self.widget.statusDisplay.executeButton.clicked.connect(self.runActionSequence)
        self.widget.statusDisplay.resetButton.clicked.connect(self.resetActionSequence)
        self.widget.statusDisplay.stopButton.clicked.connect(self.stopActionSequence)
        self.widget.statusDisplay.deleteButton.clicked.connect(self.deleteActionSequence)

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

        if len(self.sequenceController.executionList) != self.oldExecutionListLength:
            self.redrawExecutionList()

    def redrawExecutionList(self):

        self.clearActionGrid()
        self.executionListChildren = []

        lineNum = 0
        for [action, result, run] in self.sequenceController.executionList:

            actionObj = self.sequenceController.actionObjects[action]

            grid = self.ui.actionGrid.layout()
            grid.addWidget(QtGui.QLabel(action),lineNum,0)
            grid.addWidget(QtGui.QLabel(result),lineNum,1)

            if len(actionObj.animations) > 0:
                button = QtGui.QPushButton('Play: ' + str(run))
                grid.addWidget(button, lineNum, 2)

                button.connect('clicked()', functools.partial(actionObj.animate, run-1))

            lineNum += 1

        self.oldExecutionListLength = len(self.sequenceController.executionList)

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
            self.sequenceController.reset()
            self.sequenceController.start(vizMode = False)

    def resetActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.reset()

    def stopActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.stop()

    def deleteActionSequence(self):
        if self.currentAction != '':
            self.sequenceController.clear()

    def populateTreeWidget(self, treeWidget, dataDict, treeDict):
        #Populate this widget with sequences from the python library

        for seqName in dataDict.keys():
            seq, startPoints = dataDict[seqName]

            highLevelArgs = actionsequence.generateUserArgList(seq)
            item = QtGui.QTreeWidgetItem()
            item.setText(0, seqName)
            item.setText(2, ", ".join(highLevelArgs))
            treeWidget.addTopLevelItem(item)

            treeDict[seqName] = [item, {}]

            for subAction in seq.keys():
                child = QtGui.QTreeWidgetItem(item)
                child.setText(0, subAction)
                child.setText(1, seq[subAction][0].__name__)
                argDict = seq[subAction][3]
                if argDict != {}:
                    inputNames = seq[subAction][3].keys()
                    inputArgs = seq[subAction]
                    child.setText(2, ", ".join(argDict.keys()))
                    child.setText(3, ", ".join([argDict[key] if isinstance(argDict[key],str) else argDict[key].name for key in argDict.keys()]))
                else:
                    child.setText(2, "None")
                treeDict[seqName][1][subAction] = child

    def saveClicked(self, event):

        #Extract the data from the edit boxes
        newDictName = None
        newDataDict = {}

        headerName = None
        for i in range(len(self.widget.argumentEditor.findChildren(QtGui.QWidget))-1):
            children = self.widget.argumentEditor.findChildren(QtGui.QWidget)
            child1 = children[i]
            child2 = children[i+1]
            if isinstance(child1, QtGui.QLabel) and isinstance(child2, QtGui.QLabel):
                headerName = child1.text

                if child2.text in sequences.sequenceDict[self.currentAction][0].keys():
                    #both this label and the next one are keys.. that means we found and action with no args
                    #create an entry with empty arg dictionary
                    newDataDict[headerName] = {}

            elif isinstance(child1, QtGui.QLabel) and isinstance(child2, QtGui.QLineEdit):
                if str(child1.text) == 'Name: ':
                    if str(child2.text) != self.currentAction:
                        newDictName = str(child2.text)
                        print "New action requested, name: ", newDictName
                else:
                    if headerName and headerName not in newDataDict.keys():
                        newDataDict[headerName] = {}
                    newDataDict[headerName][str(child1.text)] = str(child2.text)

        #Go to the entry for the current action in the sequence dictionary and save the data
        #step 1, either make a new entry, or grab the current entry from sequences.sequenceDict
        name = None
        if newDictName:
            name = newDictName
            sequences.sequenceDict[name] = [{}, sequences.sequenceDict[self.currentAction][1]]
            sequences.sequenceDict[name][0] = copy(sequences.sequenceDict[self.currentAction][0])
        else:
            name = self.currentAction

        #step 2, copy the data into the args folders
        for subAction in sequences.sequenceDict[name][0].keys():
            for arg in newDataDict[subAction].keys():
                sequences.sequenceDict[name][0][subAction][3][arg] = newDataDict[subAction][arg]

        #Repopulate the sequence selector with this data
        self.widget.actionSequences.templateTree.clear()
        self.populateTreeWidget(self.widget.actionSequences.templateTree, sequences.sequenceDict, self.templateTreeItems)

    def clearArgEditor(self):
        for child in self.ui.scrollGrid.findChildren(QtGui.QWidget):
            child.delete()

    def clearActionGrid(self):
        for child in self.ui.actionGrid.findChildren(QtGui.QWidget):
            child.delete()


    def selectionChanged(self, event):
        if not event == None:

            #Set the new action
            self.currentAction = str(event.text(0))

            #Clear the old arguments editor
            self.clearArgEditor()

            if self.currentAction in sequences.sequenceDict.keys():
                #Add all the data of this dictionary into a grid on the UI
                grid = self.ui.scrollGrid.layout()
                lineNum = 0

                #First create a widget to display the name
                grid.addWidget(QtGui.QLabel('Name: '),lineNum,0)
                grid.addWidget(QtGui.QLineEdit(self.currentAction), lineNum, 2)
                lineNum += 1

                #Create a grid of arguments and boxes to edit them
                seq = sequences.sequenceDict[self.currentAction][0]

                for step in seq.keys():
                    grid.addWidget(QtGui.QLabel(step),lineNum,0)

                    if len(seq[step][3].keys()) == 0:
                        lineNum += 1
                    else:
                        for arg in seq[step][3].keys():
                            grid.addWidget(QtGui.QLabel(arg), lineNum, 1)
                            if isinstance(seq[step][3][arg], str):
                                grid.addWidget(QtGui.QLineEdit(seq[step][3][arg]), lineNum, 2)
                            else:
                                grid.addWidget(QtGui.QLineEdit(seq[step][3][arg].name), lineNum, 2)
                            lineNum += 1

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
