import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import FPSCounter
from ddapp import cameraview
from ddapp import objectmodel as om
import ddapp.vtkAll as vtk


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class FrameUpdater(object):


    def __init__(self, folderName, listWidget):

        self.folderName = folderName
        self.listWidget = listWidget
        self.itemMap = {}
        self.initListWidget()

        self.trace = {}

    def getFrameTransform(self, frameName):
        return vtk.vtkTransform()

    def getFramesNames(self):
        return []

    def initListWidget(self):

        frameNames = self.getFramesNames()
        for name in frameNames:
            item = QtGui.QListWidgetItem(name)
            item.setData(QtCore.Qt.CheckStateRole, QtCore.Qt.Unchecked)
            self.listWidget.addItem(item)
            self.itemMap[name] = item

        self.listWidget.connect('itemChanged(QListWidgetItem*)', self.onItemChecked)

    def onItemChecked(self, item):
        name = str(item.text())
        isChecked = item.checkState() == QtCore.Qt.Checked

        for frameObj in self.getFrameObjects():
            if frameObj.getProperty('Name') == name:
                frameObj.setProperty('Visible', isChecked)
                break
        else:
            if isChecked:
                self.addFrame(name)

    def getEnabledFrameNames(self):
        enabledFrames = set()
        for name, item in self.itemMap.iteritems():
            isChecked = item.checkState() == QtCore.Qt.Checked
            if isChecked:
                enabledFrames.add(name)
        return set(enabledFrames)

    def updateFrame(self, frameName, frameObj):
        t = self.getFrameTransform(frameName)
        frameObj.copyFrame(t)

    def addFrame(self, frameName):
        t = self.getFrameTransform(frameName)
        folder = self.getFramesFolder()
        frame = vis.showFrame(t, frameName, parent=folder, scale=0.2)
        frame.setProperty('Trace', True)

    def getFramesFolder(self):
        return om.getOrCreateContainer(self.folderName)

    def getFrameObjects(self):
        if not om.findObjectByName(self.folderName):
            return []
        return self.getFramesFolder().children()

    def hideAllFrames(self):
        for frame in self.getFrameObjects():
            frame.setProperty('Visible', False)

    def updateFrames(self):

        frames = self.getFrameObjects()
        enabledFrames = self.getEnabledFrameNames()

        for frame in frames:

            frameName = frame.getProperty('Name')
            isChecked = QtCore.Qt.Checked if frame.getProperty('Visible') else QtCore.Qt.Unchecked
            self.itemMap[frameName].setCheckState(isChecked)

            try:
                enabledFrames.remove(frameName)
            except:
                pass

            if not frame.getProperty('Visible'):
                continue

            self.updateFrame(frameName, frame)

        # add new frames if needed
        for frameName in enabledFrames:
            self.addFrame(frameName)



class BotFrameUpdater(FrameUpdater):

    def __init__(self, listWidget):
        FrameUpdater.__init__(self, 'Bot Frames', listWidget)

    def getFrameTransform(self, frameName):
        t = vtk.vtkTransform()
        t.PostMultiply()
        cameraview.imageManager.queue.getTransform(frameName, 'local', t)
        return t

    def getFramesNames(self):
        return cameraview.imageManager.queue.getBotFrameNames()


class LinkFrameUpdater(FrameUpdater):

    def __init__(self, robotModel, listWidget):
        self.robotModel = robotModel
        FrameUpdater.__init__(self, 'Link Frames', listWidget)
        robotModel.connectModelChanged(self.onModelChanged)

    def getFrameTransform(self, frameName):
        return self.robotModel.getLinkFrame(frameName)

    def getFramesNames(self):
        return sorted(list(self.robotModel.model.getLinkNames()))

    def onModelChanged(self, model):
        self.updateFrames()


class FrameVisualizationPanel(object):

    def __init__(self, view):

        self.view = view

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFrameVisualization.ui')
        assert uifile.open(uifile.ReadOnly)


        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        self.botFrameUpdater = BotFrameUpdater(self.ui.botFramesListWidget)

        robotModel = om.findObjectByName('robot state model')
        self.linkFrameUpdater = LinkFrameUpdater(robotModel, self.ui.linkFramesListWidget)

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

        PythonQt.dd.ddGroupBoxHider(self.ui.botFramesGroup)
        PythonQt.dd.ddGroupBoxHider(self.ui.linkFramesGroup)

        self.updateTimer = TimerCallback(targetFps=60)
        self.updateTimer.callback = self.updateFrames
        self.updateTimer.start()

    def onEvent(self, obj, event):
        minSize = self.ui.scrollArea.widget().minimumSizeHint.width() + self.ui.scrollArea.verticalScrollBar().width
        self.ui.scrollArea.setMinimumWidth(minSize)

    def updateFrames(self):
        self.botFrameUpdater.updateFrames()

    def getNameFilter(self):
        return str(self.ui.botFramesFilterEdit.text)

    def onNameFilterChanged(self):
        filter = self.getNameFilter()



def _getAction():
    #return app.getToolBarActions()['ActionFrameVisualizationPanel']
    return None

def init(view):

    global panel
    global dock

    panel = FrameVisualizationPanel(view)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
