import os
from director import ioUtils
from director import objectmodel as om
from director import visualization as vis
from PythonQt import QtGui


class OpenDataHandler(object):

    def __init__(self, mainWindowApp):

        self.app = mainWindowApp
        self.rootFolderName = 'data files'

        self.openAction = QtGui.QAction('&Open Data...', self.app.fileMenu)
        self.app.fileMenu.insertAction(self.app.quitAction, self.openAction)
        self.app.fileMenu.insertSeparator(self.app.quitAction)
        self.openAction.setShortcut(QtGui.QKeySequence('Ctrl+O'))
        self.openAction.connect('triggered()', self.onOpenDataFile)


    def getRootFolder(self):
        return om.getOrCreateContainer(self.rootFolderName)

    def onOpenVrml(self, filename):
        meshes, color = ioUtils.readVrml(filename)
        folder = om.getOrCreateContainer(os.path.basename(filename), parentObj=self.getRootFolder())
        for i, pair in enumerate(zip(meshes, color)):
            mesh, color = pair
            obj = vis.showPolyData(mesh, 'mesh %d' % i, color=color, parent=folder)
            vis.addChildFrame(obj)

    def openGeometry(self, filename):

        if filename.lower().endswith('wrl'):
            self.onOpenVrml(filename)
            return

        polyData = ioUtils.readPolyData(filename)

        if not polyData or not polyData.GetNumberOfPoints():
            self.app.showErrorMessage('Failed to read any data from file: %s' % filename, title='Reader error')
            return

        obj = vis.showPolyData(polyData, os.path.basename(filename), parent=self.getRootFolder())
        vis.addChildFrame(obj)

    def onOpenDataFile(self):
        fileFilters = 'Data Files (*.obj *.ply *.stl *.vtk *.vtp *.wrl)';
        filename = QtGui.QFileDialog.getOpenFileName(self.app.mainWindow, 'Open...', self.getOpenDataDirectory(), fileFilters)
        if not filename:
            return

        self.storeOpenDataDirectory(filename)
        self.openGeometry(filename)

    def getOpenDataDirectory(self):
        return self.app.settings.value('OpenDataDir') or os.path.expanduser('~')

    def storeOpenDataDirectory(self, filename):

        if os.path.isfile(filename):
            filename = os.path.dirname(filename)
        if os.path.isdir(filename):
            self.app.settings.setValue('OpenDataDir', filename)
