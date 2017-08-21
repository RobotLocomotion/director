def onPointcloud(name):
    polyData = pointcloudMap.take(name)
    if polyData:
        obj = vis.updatePolyData(polyData, name, colorByName='intensity', parent='ROS')
        if not obj.getChildFrame():
            vis.addChildFrame(obj)
pointcloudMap.connect('objectAssigned(const QString&)', onPointcloud)
