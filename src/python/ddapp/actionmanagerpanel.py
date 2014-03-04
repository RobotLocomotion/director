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

        #Populate this widget with sequences from the python library
        self.treeItems = {}

        for (seqName, seq, startPoint, typeName) in sequences.sequenceList:

            if typeName == 'Primitive':
                item = QtGui.QTreeWidgetItem()
                item.setText(0, seqName)
                item.setText(1, typeName)
                if seq[seqName][3] != [None]:
                    item.setText(2, ", ".join(seq[seqName][3]))
                else:
                    item.setText(2, "None")
                self.widget.sequenceSelector.sequenceTree.addTopLevelItem(item)
                self.treeItems[seqName] = [item, None]
            else:
                highLevelArgs = actionsequence.generateUserArgList(seq)
                item = QtGui.QTreeWidgetItem()
                item.setText(0, seqName)
                item.setText(1, typeName)
                item.setText(2, highLevelArgs)
                self.widget.sequenceSelector.sequenceTree.addTopLevelItem(item)

                self.treeItems[seqName] = [item, {}]

                for subAction in seq.keys():
                    child = QtGui.QTreeWidgetItem(item)
                    child.setText(0, subAction)
                    child.setText(1, seq[subAction][0].__name__)
                    if seq[subAction][3] != []:
                        child.setText(2, ", ".join(seq[subAction][3]))
                    else:
                        child.setText(2, "<None>")
                    self.treeItems[seqName][1][subAction] = child

        #Connect signals
        self.widget.sequenceSelector.sequenceTree.currentItemChanged.connect(self.selectionChanged)
        self.widget.sequenceSelector.selectButton.clicked.connect(self.selectClicked)
        self.widget.sequenceSelector.clearButton.clicked.connect(self.clearClicked)

	self.updatePanel()

    def updatePanel(self):
    	return

    def clearClicked(self, event):
        self.currentAction = ''
        self.widget.sequenceSelector.selectedAction.text = 'None'
        self.sequenceController.reset()

    def selectClicked(self, event):

        if self.currentAction != self.selectedActionName:
            #Clear the old action, just in case
            if not self.currentAction == '':
                self.sequenceController.reset()
            #Set the new action
            self.currentAction = self.selectedActionName
            #Display the action
            self.widget.sequenceSelector.selectedAction.text = self.currentAction
            #Populate the action manager to make it ready
            self.sequenceController.populate(sequences.sequenceDict[self.currentAction][0],
                                             sequences.sequenceDict[self.currentAction][1])


            #Create a grid of arguments and boxes to edit them
            grid = self.widget.argumentEditor.layout()
            for count, arg in enumerate(actionsequence.generateUserArgList(sequences.sequenceDict[self.currentAction][0])):
                grid.addWidget(QtGui.QLabel(arg),count,0)
                grid.addWidget(QtGui.QLineEdit(),count,1)

    def selectionChanged(self, event):
        self.selectedActionName = event.text(0)

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
