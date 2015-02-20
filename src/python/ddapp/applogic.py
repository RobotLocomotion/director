
import os
import time
import math
import ddapp.vtkAll as vtk
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui
from ddapp import getDRCBaseDir as getDRCBase
from ddapp import botspy
from ddapp import openscope
import functools

_mainWindow = None
_defaultRenderView = None

def getMainWindow():
    return _mainWindow


def quit():
    QtGui.QApplication.instance().quit()


def getViewManager():
    return getMainWindow().viewManager()


def getDRCView():
    return _defaultRenderView or getMainWindow().viewManager().findView('DRC View')


def getSpreadsheetView():
    return getMainWindow().viewManager().findView('Spreadsheet View')


def getCurrentView():
    return _defaultRenderView or getMainWindow().viewManager().currentView()


def getCurrentRenderView():
    view = getCurrentView()
    if hasattr(view, 'camera'):
        return view


def getOutputConsole():
    return getMainWindow().outputConsole()


def getPythonConsole():
    return PythonQt.dd._pythonManager.consoleWidget()


def showPythonConsole():
    getPythonConsole().show()


_exclusiveDockWidgets = {}

def hideDockWidgets(action):
    for a, w in _exclusiveDockWidgets.iteritems():
        if a is not action:
            dock, widget = w
            if not dock.isFloating():
                dock.hide()


def addWidgetToDock(widget, dockArea=QtCore.Qt.RightDockWidgetArea, action=None):

    dock = QtGui.QDockWidget()
    dock.setWidget(widget)
    dock.setWindowTitle(widget.windowTitle)
    getMainWindow().addDockWidget(dockArea, dock)

    if dockArea == QtCore.Qt.RightDockWidgetArea and action:
        _exclusiveDockWidgets[action] = (dock, widget)
        action.connect('triggered()', functools.partial(hideDockWidgets, action))

    if action is None:
        getMainWindow().addWidgetToViewMenu(dock)
    else:
        getMainWindow().addWidgetToViewMenu(dock, action)


    return dock


def resetCamera(viewDirection=None, view=None):

    view = view or getCurrentRenderView()
    assert(view)

    if viewDirection is not None:
        camera = view.camera()
        camera.SetPosition([0, 0, 0])
        camera.SetFocalPoint(viewDirection)
        camera.SetViewUp([0,0,1])

    view.resetCamera()
    view.render()


def setBackgroundColor(color, color2=None, view=None):
    view = view or getCurrentRenderView()
    assert(view)

    if color2 is None:
        color2 = color
    ren = view.backgroundRenderer()
    ren.SetBackground(color)
    ren.SetBackground2(color2)


def displaySnoptInfo(info):
    if getMainWindow() is not None:
        getMainWindow().statusBar().showMessage('Info: %d' % info)


def toggleStereoRender():
    view = getCurrentRenderView()
    assert(view)

    renderWindow = view.renderWindow()
    renderWindow.SetStereoRender(not renderWindow.GetStereoRender())
    view.render()


def getCameraTerrainModeEnabled(view):
    return isinstance(view.renderWindow().GetInteractor().GetInteractorStyle(), vtk.vtkInteractorStyleTerrain2)


def setCameraTerrainModeEnabled(view, enabled):

    if getCameraTerrainModeEnabled(view) == enabled:
        return

    if enabled:
        view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleTerrain2())
        view.camera().SetViewUp(0,0,1)
    else:
        view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    view.render()


def toggleCameraTerrainMode(view = None):

    view = view or getCurrentRenderView()
    assert view

    setCameraTerrainModeEnabled(view, not getCameraTerrainModeEnabled(view))
    updateToggleTerrainAction(view)


def findMenu(menuTitle, mainWindow=None):
    mainWindow = mainWindow or getMainWindow()
    menus = mainWindow.findChildren('QMenu')
    for menu in menus:
        if menu.title == menuTitle:
            return menu


def addMenuAction(menuTitle, actionName):
    menu = findMenu(menuTitle)
    assert menu
    return menu.addAction(actionName)


def getToolBarActions():
    return getActionsDict(getMainWindow().toolBarActions())


def getToolsMenuActions():
    return getActionsDict(getMainWindow().toolsMenu().actions())


def getActionsDict(actions):
    actionsDict = {}
    for action in actions:
        if action.name:
            actionsDict[action.name] = action
    return actionsDict


def updateToggleTerrainAction(view):
    if not getMainWindow():
        return
    isTerrainMode = False
    if hasattr(view, 'renderWindow'):
        isTerrainMode = isinstance(view.renderWindow().GetInteractor().GetInteractorStyle(), vtk.vtkInteractorStyleTerrain2)
    getToolBarActions()['ActionToggleCameraTerrainMode'].checked = isTerrainMode


class MenuActionToggleHelper(object):
    '''
    This class manages a checkable menu action and  forwards checked events
    to user selected callbacks.
    '''

    def __init__(self, menuName, actionName, getEnabledFunc, setEnabledFunc):
        self.getEnabled = getEnabledFunc
        self.setEnabled = setEnabledFunc

        self.action = addMenuAction(menuName, actionName)
        self.action.setCheckable(True)
        self.action.checked = getEnabledFunc()
        self.action.connect('triggered()', self.onActionChanged)

    def updateAction(self):
        self.action.checked = self.getEnabled()

    def onActionChanged(self):
        if self.action.checked:
            self.setEnabled(True)
        else:
            self.setEnabled(False)
        self.updateAction()


def onCurrentViewChanged(previousView, currentView):
    updateToggleTerrainAction(currentView)


def addToolbarMacro(name, func):

    toolbar = getMainWindow().macrosToolBar()
    action = toolbar.addAction(name)
    action.connect('triggered()', func)


def addShortcut(widget, keySequence, func):
    shortcut = QtGui.QShortcut(QtGui.QKeySequence(keySequence), widget)
    shortcut.connect('activated()', func)
    shortcut.connect('activatedAmbiguously()', func)
    return shortcut


def setupActions():
    botApyAction = getToolsMenuActions()['ActionBotSpy']
    botApyAction.connect(botApyAction, 'triggered()', botspy.startBotSpy)
    scopeAction = getToolsMenuActions()['ActionSignalScope']
    scopeAction.connect(scopeAction, 'triggered()', openscope.startSignalScope)


def showErrorMessage(message, title='Error'):
    QtGui.QMessageBox.warning(getMainWindow(), title, message)


def showInfoMessage(message, title='Info'):
    QtGui.QMessageBox.information(getMainWindow(), title, message)


def showViewTabContextMenu(view, tabBar, menuPosition):

    def onPopOut():
        getViewManager().popOut(view)

    menu = QtGui.QMenu(tabBar)
    menu.addAction('Pop out').connect('triggered()', onPopOut)
    menu.popup(menuPosition)


def onTabWidgetContextMenu(mouseClick):

    tabBar = getViewManager().findChildren('QTabBar')[0]
    tabIndex = tabBar.tabAt(mouseClick)
    viewName = tabBar.tabText(tabIndex)
    view = getViewManager().findView(viewName)
    if view:
        showViewTabContextMenu(view, tabBar, tabBar.mapToGlobal(mouseClick))


def setupViewManager():

    vm = getViewManager()
    vm.connect('currentViewChanged(ddViewBase*, ddViewBase*)', onCurrentViewChanged)

    tabBar = vm.findChildren('QTabBar')[0]
    tabBar.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    tabBar.connect('customContextMenuRequested(const QPoint &)', onTabWidgetContextMenu)


def startup(globals):

    global _mainWindow
    _mainWindow = globals['_mainWindow']

    if 'DRC_BASE' not in os.environ:
        showErrorMessage('DRC_BASE environment variable is not set')
        return

    if not os.path.isdir(getDRCBase()):
        showErrorMessage('DRC_BASE directory does not exist: ' + getDRCBase())
        return

    _mainWindow.connect('resetCamera()', resetCamera)
    _mainWindow.connect('toggleStereoRender()', toggleStereoRender)
    _mainWindow.connect('toggleCameraTerrainMode()', toggleCameraTerrainMode)

    setupActions()
    setupViewManager()
