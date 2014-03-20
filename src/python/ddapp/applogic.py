
import os
import time
import math
import ddapp.vtkAll as vtk
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui
from ddapp import botspy

_mainWindow = None

def getMainWindow():
    return _mainWindow

def quit():
    QtGui.QApplication.instance().quit()


def getDRCBase():
    return os.environ['DRC_BASE']


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


def getURDFModelDir():
    return os.path.join(getDRCBase(), 'software/models/mit_gazebo_models/mit_robot_drake')


def getNominalPoseMatFile():
    return os.path.join(getDRCBase(), 'software/drake/examples/Atlas/data/atlas_fp.mat')


def loadModelByName(name):
    filename = os.path.join(getURDFModelDir(), name)
    return getDRCView().loadURDFModel(filename)


def getDefaultDrakeModel():
    return getDRCView().models()[0]


def addWidgetToDock(widget, dockArea=QtCore.Qt.RightDockWidgetArea):

    dock = QtGui.QDockWidget()
    dock.setWidget(widget)
    dock.setWindowTitle(widget.windowTitle)
    getMainWindow().addDockWidget(dockArea, dock)
    getMainWindow().addWidgetToViewMenu(dock)
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
    ren = view.renderer()
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
    return getActions(getMainWindow().toolBar())


def getToolsMenuActions():
    return getActions(getMainWindow().toolsMenu())


def getActions(widget):
    actions = {}
    for action in widget.actions():
        if action.name:
            actions[action.name] = action
    return actions


def updateToggleTerrainAction(view):
    if not getMainWindow():
        return
    isTerrainMode = False
    if hasattr(view, 'renderWindow'):
        isTerrainMode = isinstance(view.renderWindow().GetInteractor().GetInteractorStyle(), vtk.vtkInteractorStyleTerrain)
    getToolBarActions()['ActionToggleCameraTerrainMode'].checked = isTerrainMode


def onCurrentViewChanged(previousView, currentView):
    updateToggleTerrainAction(currentView)


def setupToolBar():

    def onComboChanged(text):
        loadModelByName(text)


    combo = QtGui.QComboBox()
    combo.addItem('Load URDF...')
    combo.addItem('model.urdf')
    combo.addItem('model_minimal_contact.urdf')
    combo.addItem('model_minimal_contact_point_hands.urdf')
    combo.addItem('model_minimal_contact_fixedjoint_hands.urdf')

    combo.connect('currentIndexChanged(const QString&)', onComboChanged)
    toolbar = getMainWindow().toolBar()
    toolbar.addWidget(combo)


def addToolbarMacro(name, func):

    toolbar = getMainWindow().macrosToolBar()
    action = toolbar.addAction(name)
    action.connect('triggered()', func)


def setupActions():
    botApyAction = getToolsMenuActions()['ActionBotSpy']
    botApyAction.connect(botApyAction, 'triggered()', botspy.startBotSpy)


def loadRobotModelFromFile(filename):
    model = PythonQt.dd.ddDrakeModel()
    if not model.loadFromFile(filename):
        return None
    return model


def loadRobotModelFromString(xmlString):
    model = PythonQt.dd.ddDrakeModel()
    if not model.loadFromXML(xmlString):
        return None
    return model



def setupPackagePaths():

    searchPaths = [
        'ros_workspace/mit_drcsim_scripts',
        'ros_workspace/sandia-hand/ros/sandia_hand_description',
        'software/models/mit_gazebo_models/mit_robot',
        'software/models/mit_gazebo_models/irobot_hand',
        'software/models/mit_gazebo_models/multisense_sl',
        'software/models/mit_gazebo_models/handle_description',
        'software/models/mit_gazebo_models/hook_description',
        'software/models/mit_gazebo_models/hook_description',
        'software/models/mit_gazebo_models/robotiq_hand_description',
                  ]

    for path in searchPaths:
        PythonQt.dd.ddDrakeModel.addPackageSearchPath(os.path.join(getDRCBase(), path))

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

    setupPackagePaths()
    setupActions()
    #setupToolBar()

    vm = getViewManager()
    vm.connect('currentViewChanged(ddViewBase*, ddViewBase*)', onCurrentViewChanged);
