
from vtkPointCloudUtils.debugVis import DebugData
from vtkPointCloudUtils import io


'''
segmentationObj = om.findObjectByName('pointcloud snapshot')
polyData = segmentationObj.polyData


locator = vtk.vtkCellLocator()
locator.SetDataSet(polyData)
locator.BuildLocator()
'''


picker = vtk.vtkPointPicker()

mousePos = segmentation.lastMovePos

picker.Pick(mousePos[0], mousePos[1], 0, app.getCurrentView().renderer())

pickPoints = picker.GetPickedPositions()

if pickPoints.GetNumberOfPoints():

    print pickPoints.GetNumberOfPoints()

    pickedPoint = picker.GetPickedPositions().GetPoint(0)

    d = segmentation.DebugData()
    d.addSphere(pickedPoint, radius=0.01)

    segmentation.updatePolyData(d.getPolyData(), 'pick point')
