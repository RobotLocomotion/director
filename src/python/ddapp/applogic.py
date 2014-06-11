
import os
import time
import math
import ddapp.vtkAll as vtk
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui
from ddapp import getDRCBaseDir as getDRCBase
from ddapp import botspy

_mainWindow = None

def getMainWindow():
    return _mainWindow


def quit():
    QtGui.QApplication.instance().quit()


def getViewManager():
    return getMainWindow().viewManager()


def getDRCView():
    return getMainWindow().viewManager().findView('DRC View')


def getSpreadsheetView():
    return getMainWindow().viewManager().findView('Spreadsheet View')


def getCurrentView():
    return getMainWindow().viewManager().currentView()


def getCurrentRenderView():
    view = getCurrentView()
    if hasattr(view, 'camera'):
        return view


def getOutputConsole():
    return getMainWindow().outputConsole()


def addWidgetToDock(widget, dockArea=QtCore.Qt.RightDockWidgetArea, action=None):

    dock = QtGui.QDockWidget()
    dock.setWidget(widget)
    dock.setWindowTitle(widget.windowTitle)
    getMainWindow().addDockWidget(dockArea, dock)
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
    getMainWindow().statusBar().showMessage('Info: %d' % info)


def toggleStereoRender():
    view = getCurrentRenderView()
    assert(view)

    renderWindow = view.renderWindow()
    renderWindow.SetStereoRender(not renderWindow.GetStereoRender())
    view.render()


def toggleCameraTerrainMode(view = None):

    view = view or getCurrentRenderView()
    assert(view)

    iren = view.renderWindow().GetInteractor()
    if isinstance(iren.GetInteractorStyle(), vtk.vtkInteractorStyleTerrain):
        iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    else:
        iren.SetInteractorStyle(vtk.vtkInteractorStyleTerrain())
        view.camera().SetViewUp(0,0,1)

    view.render()
    updateToggleTerrainAction(view)


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
        isTerrainMode = isinstance(view.renderWindow().GetInteractor().GetInteractorStyle(), vtk.vtkInteractorStyleTerrain)
    getToolBarActions()['ActionToggleCameraTerrainMode'].checked = isTerrainMode


def onCurrentViewChanged(previousView, currentView):
    updateToggleTerrainAction(currentView)


def addToolbarMacro(name, func):

    toolbar = getMainWindow().macrosToolBar()
    action = toolbar.addAction(name)
    action.connect('triggered()', func)


def setupActions():
    botApyAction = getToolsMenuActions()['ActionBotSpy']
    botApyAction.connect(botApyAction, 'triggered()', botspy.startBotSpy)


def showErrorMessage(message, title='Error'):
    QtGui.QMessageBox.warning(getMainWindow(), title, message)


def showInfoMessage(message, title='Info'):
    QtGui.QMessageBox.information(getMainWindow(), title, message)

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

    vm = getViewManager()
    vm.connect('currentViewChanged(ddViewBase*, ddViewBase*)', onCurrentViewChanged);
