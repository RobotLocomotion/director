inputObj = om.findObjectByName('pointcloud snapshot')
polyData = inputObj.polyData
polyData.GetPointData().RemoveArray('rgb')
cameraview.colorizePoints(polyData, 'CAMERACHEST_LEFT')
cameraview.colorizePoints(polyData, 'CAMERA_LEFT')
inputObj.colorBy('rgb')
app.getCurrentRenderView().render()
