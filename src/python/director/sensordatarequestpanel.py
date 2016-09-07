import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import lcmUtils
from director import applogic as app
from director.utime import getUtime

from time import time

from maps import data_request_t
from maps import data_request_list_t

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class SensorDataRequestPanel(object):

    def __init__(self):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddSensorDataRequest.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.groups = {}
        self.registerType('camHead',data_request_t.CAMERA_IMAGE_HEAD_LEFT)
        self.registerType('camChestLeft',data_request_t.CAMERA_IMAGE_LCHEST)
        self.registerType('camChestRight',data_request_t.CAMERA_IMAGE_RCHEST)
        self.registerType('mapSceneHeight',data_request_t.HEIGHT_MAP_SCENE)
        self.registerType('mapSceneDepth',data_request_t.DEPTH_MAP_SCENE)
        self.registerType('mapWorkspaceDepth',data_request_t.DEPTH_MAP_WORKSPACE_C)
        self.registerType('mapFusedStereoHeight',data_request_t.FUSED_HEIGHT)
        self.registerType('mapWorkspaceOctomap',data_request_t.OCTREE_WORKSPACE)

        self.widget.applyAllButton.clicked.connect(self.applyAllButtonClicked)
        

    class TypeGroup:
        def __init__(self,name,msgType, widgets):
            self.name = name
            self.messageType = msgType
            self.nowButton = getattr(widgets, name + 'Button')
            self.streamCheck = getattr(widgets, name + 'Check')
            self.periodSpinner = getattr(widgets, name + 'Spinner')
            self.nowButton.clicked.connect(self.immediateRequestClicked)
            
        def immediateRequestClicked(self,event):
            req = self.getMessage()
            req.period = 0
            msg = data_request_list_t()
            msg.utime = getUtime()
            msg.requests = [req]
            msg.num_requests = len(msg.requests)
            lcmUtils.publish('DATA_REQUEST',msg)
            
        def getMessage(self):
            msg = data_request_t()
            msg.type = self.messageType
            if self.streamCheck.isChecked():
                msg.period = 10*self.periodSpinner.value
            else:
                msg.period = -10
            return msg
            

    def registerType(self,name,msgType):
        # TODO: can add widgets themselves programmatically rather than through .ui file
        self.groups[name] = self.TypeGroup(name, msgType, self.ui)

    def applyAllButtonClicked(self,event):
        msg = data_request_list_t()
        msg.utime = getUtime()
        msg.requests = []
        for name in self.groups:
            msg.requests.append(self.groups[name].getMessage())
        msg.num_requests = len(msg.requests)
        lcmUtils.publish('DATA_REQUEST',msg)

def _getAction():
    return app.getToolBarActions()['ActionSensorDataRequestPanel']


def init():

    global panel
    global dock

    panel = SensorDataRequestPanel()
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
