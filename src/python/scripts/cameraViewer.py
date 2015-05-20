import argparse
from ddapp.consoleapp import ConsoleApp
from ddapp import cameraview
from ddapp import vtkAll as vtk
import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)

class CameraVisualizer(object):
    def __init__(self):
        self.imageManager = cameraview.ImageManager()

    def createCameraView(self, channel):
        self.imageManager.queue.addCameraStream(channel)
        self.imageManager.addImage(channel)
        view = PythonQt.dd.ddQVTKWidgetView()
        view.orientationMarkerWidget().Off()
        view.backgroundRenderer().SetBackground([0,0,0])
        view.backgroundRenderer().SetBackground2([0,0,0])
        cameraview.CameraImageView(self.imageManager, channel, view=view)
        return view

    def createUI(self):
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddCameraPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        self.widget.setWindowTitle("Camera Visualizer")
        self.ui.gridLayout_2.setSpacing(0)
        self.ui.gridLayout_2.setMargin(0)

        view = self.createCameraView('CAMERACHEST_LEFT')
        frame1Layout = QtGui.QVBoxLayout(self.ui.frame1)        
        frame1Layout.setSpacing(0)
        frame1Layout.setMargin(0)
        frame1Layout.addWidget(view)

        view = self.createCameraView('CAMERA_LEFT')
        frame2Layout = QtGui.QVBoxLayout(self.ui.frame2)        
        frame2Layout.setSpacing(0)
        frame2Layout.setMargin(0)
        frame2Layout.addWidget(view)

        view = self.createCameraView('CAMERACHEST_RIGHT')
        frame3Layout = QtGui.QVBoxLayout(self.ui.frame3)        
        frame3Layout.setSpacing(0)
        frame3Layout.setMargin(0)
        frame3Layout.addWidget(view)


    def showUI(self):
        self.widget.show()

def main():
    app = ConsoleApp()
    camVis = CameraVisualizer()
    camVis.createUI()
    camVis.showUI()    
    
    app.start()


if __name__ == '__main__':
    main()
