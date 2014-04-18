import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp import visualization as vis
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



class FrameVisualizationPanel(object):

    def __init__(self, view):

        self.view = view

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFrameVisualization.ui')
        assert uifile.open(uifile.ReadOnly)


        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())


        self.botFrameItems = {}
        self.initListWidget()

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

        PythonQt.dd.ddGroupBoxHider(self.ui.botFramesGroup)

        self.updateTimer = TimerCallback(targetFps=60)
        self.updateTimer.callback = self.updateFrames
        self.updateTimer.start()


    def onEvent(self, obj, event):
        minSize = self.ui.scrollArea.widget().minimumSizeHint.width() + self.ui.scrollArea.verticalScrollBar().width
        self.ui.scrollArea.setMinimumWidth(minSize)

    def initListWidget(self):

        listWidget = self.ui.botFramesListWidget
        frameNames = self.getBotFramesNames()
        for name in frameNames:
            item = QtGui.QListWidgetItem(name)
            item.setData(QtCore.Qt.CheckStateRole, QtCore.Qt.Unchecked)
            listWidget.addItem(item)
            self.botFrameItems[name] = item

        listWidget.connect('itemChanged(QListWidgetItem*)', self.onItemChecked)

    def onItemChecked(self, item):
        name = str(item.text())
        isChecked = item.checkState() == QtCore.Qt.Checked

        for frameObj in self.getBotFrameObjects():
            if frameObj.getProperty('Name') == name:
                frameObj.setProperty('Visible', isChecked)


    def getNameFilter(self):
        return str(self.ui.botFramesFilterEdit.text)

    def onNameFilterChanged(self):
        filter = self.getNameFilter()

    def getEnabledFrameNames(self):

        enabledFrames = set()
        for name, item in self.botFrameItems.iteritems():
            isChecked = item.checkState() == QtCore.Qt.Checked
            if isChecked:
                enabledFrames.add(name)

        return set(enabledFrames)

    def getBotFrameTransform(self, frameName):
        t = vtk.vtkTransform()
        t.PostMultiply()
        cameraview.imageManager.queue.getTransform(frameName, 'local', t)
        return t 

    def updateFrame(self, frameName, frameObj):
        t = self.getBotFrameTransform(frameName)
        frameObj.copyFrame(t)

    def addFrame(self, frameName):
        t = self.getBotFrameTransform(frameName)
        folder = self.getBotFramesFolder()
        vis.showFrame(t, frameName, parent=folder, scale=0.2)

    def getBotFramesNames(self):
        return cameraview.imageManager.queue.getBotFrameNames()

    def getBotFramesFolder(self):
        return om.getOrCreateContainer('Bot Frames')

    def getBotFrameObjects(self):
        if not om.findObjectByName('Bot Frames'):
            return []
        return self.getBotFramesFolder().children()

    def hideAllFrames(self):
        for frame in self.getBotFrameObjects():
            frame.setProperty('Visible', False)


    def updateFrames(self):
        frames = self.getBotFrameObjects()
        enabledFrames = self.getEnabledFrameNames()

        for frame in frames:

            frameName = frame.getProperty('Name')

            isChecked = QtCore.Qt.Checked if frame.getProperty('Visible') else QtCore.Qt.Unchecked
            self.botFrameItems[frameName].setCheckState(isChecked)

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
