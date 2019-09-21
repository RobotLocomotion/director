import os
from director.componentgraph import ComponentFactory
from director import consoleapp
import director.objectmodel as om
import director.visualization as vis
from director.fieldcontainer import FieldContainer
from director import applogic
from director import appsettings
from director import drcargs

import functools
import PythonQt
from PythonQt import QtCore, QtGui

class MainWindowApp(object):

    def __init__(self):

        self.mainWindow = QtGui.QMainWindow()
        self.mainWindow.resize(768 * (16/9.0), 768)
        self.settings = QtCore.QSettings()

        self.fileMenu = self.mainWindow.menuBar().addMenu('&File')
        self.editMenu = self.mainWindow.menuBar().addMenu('&Edit')
        self.viewMenu = self.mainWindow.menuBar().addMenu('&View')
        self.toolbarMenu = self.viewMenu.addMenu('&Toolbars')
        self.toolsMenu = self.mainWindow.menuBar().addMenu('&Tools')
        self.helpMenu = self.mainWindow.menuBar().addMenu('&Help')
        self.viewMenuManager = PythonQt.dd.ddViewMenu(self.viewMenu)
        self.toolbarMenuManager = PythonQt.dd.ddViewMenu(self.toolbarMenu)

        self.quitAction = self.fileMenu.addAction('&Quit')
        self.quitAction.setShortcut(QtGui.QKeySequence('Ctrl+Q'))
        self.quitAction.connect('triggered()', self.quit)
        self.fileMenu.addSeparator()

        self.pythonConsoleAction = self.toolsMenu.addAction('&Python Console')
        self.pythonConsoleAction.setShortcut(QtGui.QKeySequence('F8'))
        self.pythonConsoleAction.connect('triggered()', self.showPythonConsole)
        self.toolsMenu.addSeparator()

        helpAction = self.helpMenu.addAction('Online Documentation')
        helpAction.connect('triggered()', self.showOnlineDocumentation)
        self.helpMenu.addSeparator()

        helpKeyboardShortcutsAction = self.helpMenu.addAction('Keyboard Shortcuts')
        helpKeyboardShortcutsAction.connect('triggered()', self.showOnlineKeyboardShortcuts)
        self.helpMenu.addSeparator()

    def quit(self):
        MainWindowApp.applicationInstance().quit()

    def exit(self, exitCode=0):
        MainWindowApp.applicationInstance().exit(exitCode)

    def start(self, enableAutomaticQuit=True, restoreWindow=True):
        if not consoleapp.ConsoleApp.getTestingEnabled() and restoreWindow:
            self.initWindowSettings()
        self.mainWindow.show()
        self.mainWindow.raise_()
        return consoleapp.ConsoleApp.start(enableAutomaticQuit)

    @staticmethod
    def applicationInstance():
        return QtCore.QCoreApplication.instance()

    def showPythonConsole(self):
        applogic.showPythonConsole()

    def showOnlineDocumentation(self):
        QtGui.QDesktopServices.openUrl(QtCore.QUrl('https://openhumanoids.github.io/director/'))

    def showOnlineKeyboardShortcuts(self):
        QtGui.QDesktopServices.openUrl(QtCore.QUrl('https://openhumanoids.github.io/director/user_guide/keyboard_shortcuts.html#director'))

    def showErrorMessage(self, message, title='Error'):
        QtGui.QMessageBox.warning(self.mainWindow, title, message)

    def showInfoMessage(self, message, title='Info'):
        QtGui.QMessageBox.information(self.mainWindow, title, message)

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

    def addToolBar(self, title, area=QtCore.Qt.TopToolBarArea):

        toolBar = QtGui.QToolBar(title)
        toolBar.objectName = toolBar.windowTitle
        self.mainWindow.addToolBar(area, toolBar)
        self.toolbarMenuManager.addWidget(toolBar, toolBar.windowTitle)
        return toolBar

    def addToolBarAction(self, toolBar, text, icon=None, callback=None):

        if isinstance(icon, str):
            icon = QtGui.QIcon(icon)
        action = toolBar.addAction(icon, text)

        if callback:
            action.connect('triggered()', callback)

        return action

    def registerStartupCallback(self, func, priority=1):
        consoleapp.ConsoleApp.registerStartupCallback(func, priority)

    def _restoreWindowState(self, key):
        appsettings.restoreState(self.settings, self.mainWindow, key)

    def _saveWindowState(self, key):
        appsettings.saveState(self.settings, self.mainWindow, key)
        self.settings.sync()

    def _saveCustomWindowState(self):
        self._saveWindowState('MainWindowCustom')

    def restoreDefaultWindowState(self):
        self._restoreWindowState('MainWindowDefault')

    def initWindowSettings(self):
        self._saveWindowState('MainWindowDefault')
        self._restoreWindowState('MainWindowCustom')
        self.applicationInstance().connect('aboutToQuit()', self._saveCustomWindowState)


class MainWindowAppFactory(object):

    def getComponents(self):

        components = {
            'View' : [],
            'Globals' : [],
            'GlobalModules' : ['Globals'],
            'ObjectModel' : [],
            'ViewOptions' : ['View', 'ObjectModel'],
            'MainToolBar' : ['View', 'Grid', 'ViewOptions', 'MainWindow'],
            'ViewBehaviors' : ['View'],
            'Grid': ['View', 'ObjectModel'],
            'MainWindow' : ['View', 'ObjectModel'],
            'AdjustedClippingRange' : ['View'],
            'RunScriptFunction' : ['Globals'],
            'ScriptLoader' : ['MainWindow', 'RunScriptFunction']}

        disabledComponents = []

        return components, disabledComponents

    def initView(self, fields):
        view = PythonQt.dd.ddQVTKWidgetView()
        applogic._defaultRenderView = view
        applogic.setCameraTerrainModeEnabled(view, True)
        applogic.resetCamera(viewDirection=[-1, -1, -0.3], view=view)
        return FieldContainer(view=view)

    def initObjectModel(self, fields):
        om.init()
        objectModel = om.getDefaultObjectModel()
        objectModel.getTreeWidget().setWindowTitle('Scene Browser')
        objectModel.getPropertiesPanel().setWindowTitle('Properties Panel')
        return FieldContainer(objectModel=objectModel)

    def initGrid(self, fields):
        gridObj = vis.showGrid(fields.view, parent='scene')
        gridObj.setProperty('Surface Mode', 'Surface with edges')
        gridObj.setProperty('Color', [0,0,0])
        gridObj.setProperty('Alpha', 0.1)
        applogic.resetCamera(viewDirection=[-1, -1, -0.3], view=fields.view)
        return FieldContainer(gridObj=gridObj)

    def initViewBehaviors(self, fields):
        from director import viewbehaviors
        viewBehaviors = viewbehaviors.ViewBehaviors(fields.view)
        return FieldContainer(viewBehaviors=viewBehaviors)

    def initViewOptions(self, fields):
        viewOptions = vis.ViewOptionsItem(fields.view)
        fields.objectModel.addToObjectModel(viewOptions, parentObj=fields.objectModel.findObjectByName('scene'))
        viewOptions.setProperty('Background color', [0.3, 0.3, 0.35])
        viewOptions.setProperty('Background color 2', [0.95,0.95,1])
        return FieldContainer(viewOptions=viewOptions)

    def initAdjustedClippingRange(self, fields):
        '''This setting improves the near plane clipping resolution.
        Drake often draws a very large ground plane which is detrimental to
        the near clipping for up close objects.  The trade-off is Z buffer
        resolution but in practice things look good with this setting.'''
        fields.view.renderer().SetNearClippingPlaneTolerance(0.0005)

    def initMainWindow(self, fields):

        organizationName = 'RobotLocomotion'
        applicationName = 'DirectorMainWindow'
        windowTitle = 'Director App'
        windowIcon = ':/images/drake_logo.png'

        if hasattr(fields, 'organizationName'):
            organizationName = fields.organizationName
        if hasattr(fields, 'applicationName'):
            applicationName = fields.applicationName
        if hasattr(fields, 'windowTitle'):
            windowTitle = fields.windowTitle
        if hasattr(fields, 'windowIcon'):
            windowIcon = fields.windowIcon

        MainWindowApp.applicationInstance().setOrganizationName(organizationName)
        MainWindowApp.applicationInstance().setApplicationName(applicationName)


        app = MainWindowApp()

        app.mainWindow.setCentralWidget(fields.view)
        app.mainWindow.setWindowTitle(windowTitle)
        app.mainWindow.setWindowIcon(QtGui.QIcon(windowIcon))

        sceneBrowserDock = app.addWidgetToDock(fields.objectModel.getTreeWidget(),
                              QtCore.Qt.LeftDockWidgetArea, visible=True)
        propertiesDock = app.addWidgetToDock(app.wrapScrollArea(fields.objectModel.getPropertiesPanel()),
                              QtCore.Qt.LeftDockWidgetArea, visible=True)

        app.addViewMenuSeparator()

        def toggleObjectModelDock():
            newState = not sceneBrowserDock.visible
            sceneBrowserDock.setVisible(newState)
            propertiesDock.setVisible(newState)

        applogic.addShortcut(app.mainWindow, 'F1', toggleObjectModelDock)
        #applogic.addShortcut(app.mainWindow, 'F8', app.showPythonConsole)

        return FieldContainer(
          app=app,
          mainWindow=app.mainWindow,
          sceneBrowserDock=sceneBrowserDock,
          propertiesDock=propertiesDock,
          toggleObjectModelDock=toggleObjectModelDock,
          commandLineArgs=drcargs.args()
          )


    def initMainToolBar(self, fields):

        from director import viewcolors

        app = fields.app
        toolBar = app.addToolBar('Main Toolbar')
        app.addToolBarAction(toolBar, 'Python Console', ':/images/python_logo.png', callback=app.showPythonConsole)
        toolBar.addSeparator()

        terrainModeAction = fields.app.addToolBarAction(toolBar, 'Camera Free Rotate', ':/images/camera_mode.png')

        lightAction = fields.app.addToolBarAction(toolBar, 'Background Light', ':/images/light_bulb_icon.png')


        app.addToolBarAction(toolBar, 'Reset Camera', ':/images/reset_camera.png', callback=applogic.resetCamera)

        def getFreeCameraMode():
            return not applogic.getCameraTerrainModeEnabled(fields.view)

        def setFreeCameraMode(enabled):
            applogic.setCameraTerrainModeEnabled(fields.view, not enabled)

        terrainToggle = applogic.ActionToggleHelper(terrainModeAction, getFreeCameraMode, setFreeCameraMode)

        viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(fields.viewOptions, fields.gridObj,
                                lightAction)

        return FieldContainer(viewBackgroundLightHandler=viewBackgroundLightHandler, terrainToggle=terrainToggle)

    def initGlobalModules(self, fields):

        from PythonQt import QtCore, QtGui
        from director import objectmodel as om
        from director import consoleapp
        from director import visualization as vis
        from director import applogic
        from director import transformUtils
        from director import filterUtils
        from director import ioUtils
        from director import vtkAll as vtk
        from director import vtkNumpy as vnp
        from director.debugVis import DebugData
        from director.timercallback import TimerCallback
        from director.fieldcontainer import FieldContainer
        import numpy as np
        import os
        import sys

        modules = dict(locals())
        del modules['fields']
        del modules['self']
        fields.globalsDict.update(modules)

    def initGlobals(self, fields):
        try:
            globalsDict = fields.globalsDict
        except AttributeError:
            globalsDict = dict()
        if globalsDict is None:
            globalsDict = dict()
        return FieldContainer(globalsDict=globalsDict)

    def initRunScriptFunction(self, fields):

        globalsDict = fields.globalsDict

        def runScript(filename, commandLineArgs=None):
            commandLineArgs = commandLineArgs or []
            args = dict(__file__=filename,
                        _argv=[filename] + commandLineArgs,
                        _fields=fields)
            prev_args = {}
            for k, v in args.items():
                if k in globalsDict:
                    prev_args[k] = globalsDict[k]
                globalsDict[k] = v
            try:
                code = compile(open(filename, 'r').read(), filename, 'exec')
                exec(code, globalsDict)
            finally:
                for k in args.keys():
                    del globalsDict[k]
                for k, v in prev_args.items():
                    globalsDict[k] = v

        return FieldContainer(runScript=runScript)


    def initScriptLoader(self, fields):
        def loadScripts():
            for scriptArgs in fields.commandLineArgs.scripts:
                fields.runScript(scriptArgs[0], scriptArgs[1:])
        fields.app.registerStartupCallback(loadScripts)


class MainWindowPanelFactory(object):

    def getComponents(self):

        components = {
            'OpenDataHandler' : ['MainWindow'],
            'ScreenGrabberPanel' : ['MainWindow'],
            'CameraBookmarksPanel' : ['MainWindow'],
            'CameraControlPanel' : ['MainWindow'],
            'MeasurementPanel' : ['MainWindow'],
            'OutputConsole' : ['MainWindow'],
            'UndoRedo' : ['MainWindow'],
            'DrakeVisualizer' : ['MainWindow'],
            'TreeViewer' : ['MainWindow'],
            'LCMGLRenderer' : ['MainWindow']}

        # these components depend on lcm and lcmgl
        # so they are disabled by default
        disabledComponents = [
            'DrakeVisualizer',
            'TreeViewer',
            'LCMGLRenderer']

        return components, disabledComponents


    def initOpenDataHandler(self, fields):
        from director import opendatahandler
        openDataHandler = opendatahandler.OpenDataHandler(fields.app)

        def loadData():
            for filename in drcargs.args().data_files:
                openDataHandler.openGeometry(filename)
        fields.app.registerStartupCallback(loadData)

        return FieldContainer(openDataHandler=openDataHandler)

    def initOutputConsole(self, fields):
        from director import outputconsole
        outputConsole = outputconsole.OutputConsole()
        outputConsole.addToAppWindow(fields.app, visible=False)

        return FieldContainer(outputConsole=outputConsole)

    def initMeasurementPanel(self, fields):
        from director import measurementpanel
        measurementPanel = measurementpanel.MeasurementPanel(fields.app, fields.view)
        measurementDock = fields.app.addWidgetToDock(measurementPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        return FieldContainer(
          measurementPanel=measurementPanel,
          measurementDock=measurementDock
          )

    def initScreenGrabberPanel(self, fields):

        from director.screengrabberpanel import ScreenGrabberPanel
        screenGrabberPanel = ScreenGrabberPanel(fields.view)
        screenGrabberDock = fields.app.addWidgetToDock(screenGrabberPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        return FieldContainer(
          screenGrabberPanel=screenGrabberPanel,
          screenGrabberDock=screenGrabberDock
          )

    def initCameraBookmarksPanel(self, fields):

        from director import camerabookmarks

        cameraBookmarksPanel = camerabookmarks.CameraBookmarkWidget(fields.view)
        cameraBookmarksDock = fields.app.addWidgetToDock(cameraBookmarksPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        return FieldContainer(
          cameraBookmarksPanel=cameraBookmarksPanel,
          cameraBookmarksDock=cameraBookmarksDock
          )

    def initCameraControlPanel(self, fields):

        from director import cameracontrolpanel
        cameraControlPanel = cameracontrolpanel.CameraControlPanel(fields.view)
        cameraControlDock = fields.app.addWidgetToDock(cameraControlPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        return FieldContainer(
          cameraControlPanel=cameraControlPanel,
          cameraControlDock=cameraControlDock
          )

    def initUndoRedo(self, fields):

      undoStack = QtGui.QUndoStack()
      undoView = QtGui.QUndoView(undoStack)
      undoView.setEmptyLabel('Start')
      undoView.setWindowTitle('History')
      undoDock = fields.app.addWidgetToDock(undoView, QtCore.Qt.LeftDockWidgetArea, visible=False)

      undoAction = undoStack.createUndoAction(undoStack)
      redoAction = undoStack.createRedoAction(undoStack)
      undoAction.setShortcut(QtGui.QKeySequence('Ctrl+Z'))
      redoAction.setShortcut(QtGui.QKeySequence('Ctrl+Shift+Z'))

      fields.app.editMenu.addAction(undoAction)
      fields.app.editMenu.addAction(redoAction)

      return FieldContainer(
        undoDock=undoDock,
        undoStack=undoStack,
        undoView=undoView,
        undoAction=undoAction,
        redoAction=redoAction
        )

    def initDrakeVisualizer(self, fields):

        from director import drakevisualizer
        drakeVisualizer = drakevisualizer.DrakeVisualizer(fields.view)

        applogic.MenuActionToggleHelper('Tools', drakeVisualizer.name, drakeVisualizer.isEnabled, drakeVisualizer.setEnabled)

        return FieldContainer(
          drakeVisualizer=drakeVisualizer
          )

    def initTreeViewer(self, fields):

        from director import treeviewer
        treeViewer = treeviewer.TreeViewer(fields.view)

        applogic.MenuActionToggleHelper('Tools', treeViewer.name, treeViewer.isEnabled, treeViewer.setEnabled)

        return FieldContainer(
          treeViewer=treeViewer
          )

    def initLCMGLRenderer(self, fields):

        from director import lcmgl
        if lcmgl.LCMGL_AVAILABLE:
            lcmglManager = lcmgl.LCMGLManager(fields.view)
            applogic.MenuActionToggleHelper('Tools', 'LCMGL Renderer', lcmglManager.isEnabled, lcmglManager.setEnabled)
        else:
            lcmglManager = None

        return FieldContainer(
          lcmglManager=lcmglManager
          )


def construct(globalsDict=None):
    fact = ComponentFactory()
    fact.register(MainWindowAppFactory)
    fact.register(MainWindowPanelFactory)
    return fact.construct(globalsDict=globalsDict)


def main(globalsDict=None):

    app = construct(globalsDict)

    if globalsDict is not None:
        globalsDict.update(**dict(app))

    app.app.start()


if __name__ == '__main__':
    main(globals())
