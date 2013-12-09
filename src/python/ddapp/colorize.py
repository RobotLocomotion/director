import ddapp.applogic as app
import ddapp.objectmodel as om
from ddapp import cameraview

actionName = 'ActionColorizeLidar'



def setVisProperties(obj, colorModeEnabled):

    if colorModeEnabled:
        alpha = 1.0
        pointSize = 5.0
        colorBy = 'rgb'
    else:
        alpha = 0.3
        pointSize = 1.0
        colorBy = None

    obj.colorBy(colorBy)
    obj.setProperty('Alpha', alpha)
    obj.setProperty('Point Size', pointSize)
    app.getCurrentRenderView().render()


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


def colorizeMapCallback():
    obj = om.findObjectByName('Map Server')
    colorizePoints(obj.polyData)
    setVisProperties(obj, True)


def colorizeMaps(enabled):
    mapServerObj = om.findObjectByName('Map Server')
    if not mapServerObj:
        return

    if enabled:
        colorizeMapCallback()
        mapServerObj.source.colorizeCallback = colorizeMapCallback
    else:
        mapServerObj.source.colorizeCallback = None

    setVisProperties(mapServerObj, enabled)


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
    colorizeSegmentationLidar(colorizeEnabled)


def init():
    action = app.getToolBarActions()[actionName]
    action.connect(action, 'triggered()', onColorizeLidar)
