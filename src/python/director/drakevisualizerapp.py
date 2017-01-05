##########################################################
from director.screengrabberpanel import ScreenGrabberPanel
from director import lcmgl
from director import objectmodel as om
from director import applogic
from director import appsettings
from director import viewbehaviors
from director import camerabookmarks
from director import cameracontrolpanel
from director import visualization as vis
from director import ioUtils
import PythonQt
from PythonQt import QtCore, QtGui


class DrakeVisualizerApp():

    def __init__(self, visualizerClass):

        self.applicationInstance().setOrganizationName('RobotLocomotionGroup')
        self.applicationInstance().setApplicationName('drake-visualizer')

        om.init()
        self.view = PythonQt.dd.ddQVTKWidgetView()

        # init grid
        self.gridObj = vis.showGrid(self.view, parent='scene')
        self.gridObj.setProperty('Surface Mode', 'Surface with edges')
        self.gridObj.setProperty('Color', [0,0,0])
        self.gridObj.setProperty('Alpha', 0.1)

        # init view options
        self.viewOptions = vis.ViewOptionsItem(self.view)
        om.addToObjectModel(self.viewOptions, parentObj=om.findObjectByName('scene'))
        self.viewOptions.setProperty('Background color', [0.3, 0.3, 0.35])
        self.viewOptions.setProperty('Background color 2', [0.95,0.95,1])

        # setup camera
        applogic.setCameraTerrainModeEnabled(self.view, True)
        applogic.resetCamera(viewDirection=[-1, 0, -0.3], view=self.view)

        # This setting improves the near plane clipping resolution.
        # Drake often draws a very large ground plane which is detrimental to
        # the near clipping for up close objects.  The trade-off is Z buffer
        # resolution but in practice things look good with this setting.
        self.view.renderer().SetNearClippingPlaneTolerance(0.0005)

        # add view behaviors
        self.viewBehaviors = viewbehaviors.ViewBehaviors(self.view)
        applogic._defaultRenderView = self.view

        self.mainWindow = QtGui.QMainWindow()
        self.mainWindow.setCentralWidget(self.view)
        self.mainWindow.resize(768 * (16/9.0), 768)
        self.mainWindow.setWindowTitle('Drake Visualizer')
        self.mainWindow.setWindowIcon(QtGui.QIcon(':/images/drake_logo.png'))

        self.settings = QtCore.QSettings()

        self.fileMenu = self.mainWindow.menuBar().addMenu('&File')
        self.viewMenu = self.mainWindow.menuBar().addMenu('&View')
        self.viewMenuManager = PythonQt.dd.ddViewMenu(self.viewMenu)

        self.drakeVisualizer = visualizerClass(self.view)
        self.lcmglManager = lcmgl.LCMGLManager(self.view) if lcmgl.LCMGL_AVAILABLE else None

        model = om.getDefaultObjectModel()
        model.getTreeWidget().setWindowTitle('Scene Browser')
        model.getPropertiesPanel().setWindowTitle('Properties Panel')

        self.sceneBrowserDock = self.addWidgetToDock(model.getTreeWidget(), QtCore.Qt.LeftDockWidgetArea, visible=False)
        self.propertiesDock = self.addWidgetToDock(self.wrapScrollArea(model.getPropertiesPanel()), QtCore.Qt.LeftDockWidgetArea, visible=False)

        self.addViewMenuSeparator()

        self.screenGrabberPanel = ScreenGrabberPanel(self.view)
        self.screenGrabberDock = self.addWidgetToDock(self.screenGrabberPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        self.cameraBookmarksPanel = camerabookmarks.CameraBookmarkWidget(self.view)
        self.cameraBookmarksDock = self.addWidgetToDock(self.cameraBookmarksPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        self.cameraControlPanel = cameracontrolpanel.CameraControlPanel(self.view)
        self.cameraControlDock = self.addWidgetToDock(self.cameraControlPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        act = self.fileMenu.addAction('&Quit')
        act.setShortcut(QtGui.QKeySequence('Ctrl+Q'))
        act.connect('triggered()', self.applicationInstance().quit)

        self.fileMenu.addSeparator()

        act = self.fileMenu.addAction('&Open Data...')
        act.setShortcut(QtGui.QKeySequence('Ctrl+O'))
        act.connect('triggered()', self._onOpenDataFile)

        applogic.addShortcut(self.mainWindow, 'F1', self._toggleObjectModel)
        applogic.addShortcut(self.mainWindow, 'F8', applogic.showPythonConsole)
        self.applicationInstance().connect('aboutToQuit()', self._onAboutToQuit)

        for obj in om.getObjects():
            obj.setProperty('Deletable', False)

        self.mainWindow.show()
        self._saveWindowState('MainWindowDefault')
        self._restoreWindowState('MainWindowCustom')

    def _onAboutToQuit(self):
        self._saveWindowState('MainWindowCustom')
        self.settings.sync()

    def _restoreWindowState(self, key):
        appsettings.restoreState(self.settings, self.mainWindow, key)

    def _saveWindowState(self, key):
        appsettings.saveState(self.settings, self.mainWindow, key)

    def _toggleObjectModel(self):
        self.sceneBrowserDock.setVisible(not self.sceneBrowserDock.visible)
        self.propertiesDock.setVisible(not self.propertiesDock.visible)

    def _toggleScreenGrabber(self):
        self.screenGrabberDock.setVisible(not self.screenGrabberDock.visible)

    def _toggleCameraBookmarks(self):
        self.cameraBookmarksDock.setVisible(not self.cameraBookmarksDock.visible)

    def _getOpenDataDirectory(self):
        return self.settings.value('OpenDataDir') or os.path.expanduser('~')

    def _storeOpenDataDirectory(self, filename):

        if os.path.isfile(filename):
            filename = os.path.dirname(filename)
        if os.path.isdir(filename):
            self.settings.setValue('OpenDataDir', filename)


    def _showErrorMessage(self, title, message):
        QtGui.QMessageBox.warning(self.mainWindow, title, message)


    def onOpenVrml(self, filename):
        meshes, color = ioUtils.readVrml(filename)
        folder = om.getOrCreateContainer(os.path.basename(filename), parentObj=om.getOrCreateContainer('files'))
        for i, pair in enumerate(zip(meshes, color)):
            mesh, color = pair
            obj = vis.showPolyData(mesh, 'mesh %d' % i, color=color, parent=folder)
            vis.addChildFrame(obj)

    def _openGeometry(self, filename):

        if filename.lower().endswith('wrl'):
            self.onOpenVrml(filename)
            return

        polyData = ioUtils.readPolyData(filename)

        if not polyData or not polyData.GetNumberOfPoints():
            self._showErrorMessage('Failed to read any data from file: %s' % filename, title='Reader error')
            return

        vis.showPolyData(polyData, os.path.basename(filename), parent='files')

    def _onOpenDataFile(self):
        fileFilters = "Data Files (*.obj *.ply *.stl *.vtk *.vtp *.wrl)";
        filename = QtGui.QFileDialog.getOpenFileName(self.mainWindow, "Open...", self._getOpenDataDirectory(), fileFilters)
        if not filename:
            return

        self._storeOpenDataDirectory(filename)
        self._openGeometry(filename)

    def wrapScrollArea(self, widget):
        w = QtGui.QScrollArea()
        w.setWidget(widget)
        w.setWidgetResizable(True)
        w.setWindowTitle(widget.windowTitle)
        #w.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        return w

    def addWidgetToViewMenu(self, widget):
        self.viewMenuManager.addWidget(widget, widget.windowTitle)

    def addViewMenuSeparator(self):
        self.viewMenuManager.addSeparator()

    def addWidgetToDock(self, widget, dockArea, visible=True):
        dock = QtGui.QDockWidget()
        dock.setWidget(widget)
        dock.setWindowTitle(widget.windowTitle)
        dock.setObjectName(widget.windowTitle + ' Dock')
        dock.setVisible(visible)
        self.mainWindow.addDockWidget(dockArea, dock)
        self.addWidgetToViewMenu(dock)
        return dock

    def quit(self):
        self.applicationInstance().quit()

    def applicationInstance(self):
        return QtCore.QCoreApplication.instance()

    def start(self, globalsDict=None):
        if globalsDict:
            globalsDict['app'] = self
            globalsDict['view'] = self.view
            globalsDict['quit'] = self.quit
            globalsDict['exit'] = self.quit

        return QtCore.QCoreApplication.instance().exec_()
