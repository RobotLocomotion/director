from ddapp.consoleapp import ConsoleApp
from ddapp.screengrabberpanel import ScreenGrabberPanel
from ddapp.drakevisualizer import DrakeVisualizer
from ddapp import objectmodel as om
from ddapp import applogic

from PythonQt import QtCore, QtGui



class DrakeVisualizerApp(ConsoleApp):

    def __init__(self):
        ConsoleApp.__init__(self)

        self.view = self.createView()

        self.mainWindow = QtGui.QMainWindow()
        self.mainWindow.setCentralWidget(self.view)
        self.mainWindow.resize(768 * (16/9.0), 768)
        self.mainWindow.setWindowTitle('Drake Visualizer')
        self.mainWindow.setWindowIcon(QtGui.QIcon(':/images/drake_logo.png'))

        self.drakeVisualizer = DrakeVisualizer(self.view)

        self.screenGrabberPanel = ScreenGrabberPanel(self.view)
        self.screenGrabberDock = self.addWidgetToDock(self.screenGrabberPanel.widget, QtCore.Qt.RightDockWidgetArea)
        self.screenGrabberDock.setVisible(False)

        model = om.getDefaultObjectModel()
        model.getTreeWidget().setWindowTitle('Scene Browser')
        model.getPropertiesPanel().setWindowTitle('Properties Panel')

        self.sceneBrowserDock = self.addWidgetToDock(model.getTreeWidget(), QtCore.Qt.LeftDockWidgetArea)
        self.propertiesDock = self.addWidgetToDock(model.getPropertiesPanel(), QtCore.Qt.LeftDockWidgetArea)
        self.sceneBrowserDock.setVisible(False)
        self.propertiesDock.setVisible(False)

        applogic.addShortcut(self.mainWindow, 'Ctrl+Q', self.quit)
        applogic.addShortcut(self.mainWindow, 'F1', self.toggleObjectModel)
        applogic.addShortcut(self.mainWindow, 'F2', self.toggleScreenGrabber)


    def toggleObjectModel(self):
        self.sceneBrowserDock.setVisible(not self.sceneBrowserDock.visible)
        self.propertiesDock.setVisible(not self.propertiesDock.visible)

    def toggleScreenGrabber(self):
        self.screenGrabberDock.setVisible(not self.screenGrabberDock.visible)

    def addWidgetToDock(self, widget, dockArea):
        dock = QtGui.QDockWidget()
        dock.setWidget(widget)
        dock.setWindowTitle(widget.windowTitle)
        self.mainWindow.addDockWidget(dockArea, dock)
        return dock


def main():

    # use global so the variable is available in the python console
    global app

    app = DrakeVisualizerApp()
    app.setupGlobals(globals())
    app.mainWindow.show()
    app.start()


if __name__ == '__main__':
    main()
