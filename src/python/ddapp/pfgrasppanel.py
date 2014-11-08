import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime

from time import time

from drc import data_request_t
from drc import data_request_list_t

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class PFGraspPanel(object):

    def __init__(self, pfgrasper, _prevParent, imageView, imagePicker, cameraview):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddPFGrasp.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True
        
        # main states
        self.pfgrasper = pfgrasper
        
        # for image overlay function
        self._prevParent = _prevParent
        self.imageView = imageView
        self.imagePicker = imagePicker
        self.cameraview = cameraview
        
        #Connect signals
        self.ui.showLHandCameraButton.clicked.connect(self.showLHandCameraButtonClicked) 
        self.ui.showRHandCameraButton.clicked.connect(self.showRHandCameraButtonClicked)  
        self.ui.startStepButton.clicked.connect(self.startStepButtonClicked)   
        self.ui.startAutoButton.clicked.connect(self.startAutoButtonClicked)  
        self.ui.runOneIterButton.clicked.connect(self.runOneIterButtonClicked)   
        self.ui.stopButton.clicked.connect(self.stopButtonClicked)  
        self.ui.startGraspButton.clicked.connect(self.startGraspButtonClicked)  
        self.ui.spawnDrillButton.clicked.connect(self.spawnDrillButtonClicked)  
        self.ui.projectDrillButton.clicked.connect(self.projectDrillButtonClicked)  
        self.ui.reachDrillButton.clicked.connect(self.reachDrillButtonClicked)  
        self.ui.graspDrillButton.clicked.connect(self.graspDrillButtonClicked)  
        self.ui.apply3DFitButton.clicked.connect(self.apply3DFitButtonClicked)  
        
    def showLHandCameraButtonClicked(self, event):
        if self.ui.showLHandCameraButton.checked:
            self.ui.selectHandBox.setCurrentIndex(0)
            self.showImageOverlay(size=400, viewName='CAMERALHAND')
            self.pfgrasper.setGraspingHand('left')
        else:
            self.hideImageOverlay()
            
    def showRHandCameraButtonClicked(self, event):
        if self.ui.showRHandCameraButton.checked:
            self.ui.selectHandBox.setCurrentIndex(1)
            self.showImageOverlay(size=400, viewName='CAMERARHAND')
            self.pfgrasper.setGraspingHand('right')
        else:
            self.hideImageOverlay()
           
    def startStepButtonClicked(self, event):
        self.pfgrasper.graspingHand = str(self.ui.selectHandBox.currentText)
        self.pfgrasper.start(autoMode=False)
        
    def startAutoButtonClicked(self, event):
        self.pfgrasper.graspingHand = str(self.ui.selectHandBox.currentText)
        self.pfgrasper.start(autoMode=True)
        
    def runOneIterButtonClicked(self, event):
        self.pfgrasper.autoMode = False
        self.pfgrasper.runoneiter()
    
    def stopButtonClicked(self, event):
        self.pfgrasper.autoMode = False
        self.pfgrasper.stop()
    
    def startGraspButtonClicked(self, event):
        self.pfgrasper.autoMode = False
        self.pfgrasper.guardedMoveForwardAndGraspHoldRetreat()
        
        
    def spawnDrillButtonClicked(self, event):
        self.pfgrasper.spawnDrillAffordance()
        
    def projectDrillButtonClicked(self, event):
        self.pfgrasper.drawDrill(mustVisible = True)
        
    def apply3DFitButtonClicked(self):
        self.pfgrasper.apply3DFit()
        
    def reachDrillButtonClicked(self, event):
        self.pfgrasper.planReach()
    
    def graspDrillButtonClicked(self, event):
        self.pfgrasper.planGraspLineMotion()
        
    ## adapted from startup
    def showImageOverlay(self, size=400, viewName='CAMERALHAND'):
        self.imageView = self.cameraview.views[viewName]
        _prevParent = self.imageView.view.parent()

        self.imageView.view.hide()
        view = app.getDRCView()
        
        self.imageView.view.setParent(view)
        self.imageView.view.resize(size, size)
        self.imageView.view.move(0,0)
        self.imageView.view.show()
        self.imagePicker.start()
    
    ## adapted from startup
    def hideImageOverlay(self):
        self.imageView.view.hide()
        self.imageView.view.setParent(self._prevParent)
        self.imageView.view.show()
        self.imagePicker.stop()
    
def _getAction():
    return app.getToolBarActions()['ActionPFGraspPanel']


def init(pfgrasper, _prevParent, imageView, imagePicker, cameraview):

    global panel
    global dock

    panel = PFGraspPanel(pfgrasper, _prevParent, imageView, imagePicker, cameraview)
    pfgrasper.ui = panel.ui
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel


