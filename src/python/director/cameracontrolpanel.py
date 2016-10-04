import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import objectmodel as om
from director import visualization as vis
from director import cameracontrol
from director import propertyset
from director import pointpicker

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


class CameraControlPanel(object):

    def __init__(self, view):

        self.view = view
        self.trackerManager = cameracontrol.CameraTrackerManager()
        self.trackerManager.setView(view)

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddCameraControlPanel.ui')
        assert uifile.open(uifile.ReadOnly)


        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        self.ui.targetNameLabel.setText('None')

        self.ui.setTargetButton.connect('clicked()', self.onSetTarget)

        for modeName in self.trackerManager.trackers.keys():
            self.ui.trackModeCombo.addItem(modeName)
        self.ui.trackModeCombo.connect('currentIndexChanged(const QString&)', self.onTrackModeChanged)

        self.ui.controlFrame.setEnabled(False)

        l = self.ui.propertiesFrame.layout()
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.propertiesPanel.setBrowserModeToWidget()
        l.addWidget(self.propertiesPanel)
        self.panelConnector = None

        self.objectPicker = pointpicker.ObjectPicker(self.view)
        self.objectPicker.callbackFunc = self.onPickObject
        self.objectPicker.abortFunc = self.onAbortPick


    def onPickObject(self, objs):
        if objs:
            self.setTarget(objs[0])
        else:
            self.onAbortPick()

    def onAbortPick(self):
        self.ui.selectedObjectNameLabel.setText('')
        self.ui.setTargetButton.setVisible(True)
        self.objectPicker.stop()

    def getSelectedTarget(self):
        obj = om.getActiveObject()
        return obj if obj and hasattr(obj, 'actor') else None

    def getObjectShortName(self, obj):
        name = obj.getProperty('Name')
        maxLength = 15
        if len(name) > maxLength:
            name = name[:maxLength-3] + '...'
        return name

    def onObjectRemoved(self, objectModel, obj):
        if obj == self.trackerManager.target:
            self.setTarget(None)

    def onSetTarget(self):
        self.ui.setTargetButton.setVisible(False)
        self.ui.selectedObjectNameLabel.setText('Click an object in the view...')
        self.objectPicker.start()

    def setTarget(self, obj):

        self.onAbortPick()

        if obj and not hasattr(obj, 'getChildFrame'):
            obj = None

        if obj:
            vis.addChildFrame(obj)
            obj.connectRemovedFromObjectModel(self.onObjectRemoved)

        self.trackerManager.setTarget(obj)

        name = self.getObjectShortName(obj) if obj else 'None'
        self.ui.targetNameLabel.setText(name)
        self.ui.controlFrame.setEnabled(obj is not None)

        if not obj:
            self.ui.trackModeCombo.setCurrentIndex(0)


    def onTrackModeChanged(self):
        mode = self.ui.trackModeCombo.currentText
        self.setTrackMode(mode)

    def setTrackMode(self, mode):
        self.trackerManager.setTrackerMode(mode)

        clearLayout(self.ui.actionsFrame)
        actions = self.trackerManager.getModeActions()

        for actionName in actions:
            def newActionButton(actionName):
                actionButton = QtGui.QPushButton(actionName)
                def onAction():
                    self.trackerManager.onModeAction(actionName)
                actionButton.connect('clicked()', onAction)
                return actionButton

            self.ui.actionsFrame.layout().addWidget(newActionButton(actionName))

        self.propertiesPanel.clear()
        if self.panelConnector:
            self.panelConnector.cleanup()
            self.panelConnector = None

        properties = self.trackerManager.getModeProperties()
        if properties:
            self.panelConnector = propertyset.PropertyPanelConnector(properties, self.propertiesPanel)
