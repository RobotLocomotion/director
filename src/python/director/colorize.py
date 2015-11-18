import ddapp.applogic as app
import ddapp.objectmodel as om
from ddapp import cameraview

import functools

actionName = 'ActionColorizeLidar'



def setVisProperties(obj, colorModeEnabled):

    if colorModeEnabled:
        alpha = 1.0
        pointSize = 4.0
        colorBy = 'rgb'
    else:
        alpha = 0.5
        pointSize = 1.0
        colorBy = None

    obj.setProperty('Alpha', alpha)
    obj.setProperty('Point Size', pointSize)


def colorizePoints(polyData):
    cameras = ['CAMERACHEST_RIGHT', 'CAMERACHEST_LEFT', 'CAMERA_LEFT']
    for camera in cameras:
        cameraview.colorizePoints(polyData, camera)


def colorizeSegmentationLidar(enabled):

    obj = om.findObjectByName('pointcloud snapshot')
    if not obj:
        return

    if enabled:
        colorizePoints(obj.polyData)
    else:
        obj.polyData.GetPointData().RemoveArray('rgb')

    setVisProperties(obj, enabled)


_colorizeMapNames = ['HEIGHT_MAP_SCENE', 'SCANS_HALF_SWEEP']

def colorizeMapCallback(obj):
    if obj and obj.getProperty('Name') in _colorizeMapNames:
        colorizePoints(obj.polyData)
        obj._updateColorByProperty()
        obj.setProperty('Color By', 'rgb')


def colorizeMaps(enabled):

    if enabled:
        om.findObjectByName('Map Server').source.colorizeCallback = colorizeMapCallback
        for name in _colorizeMapNames:
            colorizeMapCallback(om.findObjectByName(name))
    else:
        om.findObjectByName('Map Server').source.colorizeCallback = None


def colorizeMultisense(enabled):
    obj = om.findObjectByName('Multisense')
    if not obj:
        return

    setVisProperties(obj, enabled)
    colorBy = 'Camera RGB' if enabled else 'Solid Color'
    obj.setProperty('Color By', colorBy)


def colorizeMapsOff():
    obj = om.findObjectByName('Map Server')
    obj.source.colorizeCallback = None
    alpha = 0.7
    pointSize = 1.0
    obj.setProperty('Alpha', alpha)
    obj.setProperty('Point Size', pointSize)

def onColorizeLidar():

    colorizeEnabled = app.getToolBarActions()[actionName].checked
    colorizeMaps(colorizeEnabled)
    colorizeMultisense(colorizeEnabled)
    colorizeSegmentationLidar(colorizeEnabled)

def initColorizeCallbacks():

    obj = om.findObjectByName('Multisense')
    assert(obj)

    def callback():
        colorizePoints(obj.model.polyDataObj.polyData)

    obj.model.colorizeCallback = callback


def init():
    action = app.getToolBarActions()[actionName]
    action.connect(action, 'triggered()', onColorizeLidar)
    initColorizeCallbacks()
