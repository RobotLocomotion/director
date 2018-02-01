from director.debugVis import DebugData
from director import consoleapp
from director import vtkAll as vtk
from director import visualization as vis
from director import vtkNumpy as vnp
from director import applogic
import numpy as np



def show(data, offset=[0,0,0]):
    polyData = data.getPolyData()
    obj = vis.showPolyData(polyData, 'data', colorByName='RGB255')
    t = vtk.vtkTransform()
    t.Translate(offset)
    vis.addChildFrame(obj).copyFrame(t)
    return obj


def getHelixPoints(numberOfPoints=1000):
    theta = np.linspace(0, np.pi*10, numberOfPoints)
    x = theta
    z =  np.sin(theta)
    y =  np.cos(theta)
    pts = np.vstack((x, y, z)).T.copy()
    pts /= np.max(pts)
    return pts



app = consoleapp.ConsoleApp()
view = app.createView()
view.show()


d = DebugData()
d.addLine((0,0,0), (1,0,0), radius=0.03)
show(d, (0, 0, 0))


d = DebugData()
d.addPolygon([[0,0,0], [0.8, 0, 0], [1, 0.6, 0], [0.4, 1, 0], [-0.2, 0.6, 0]])
show(d, (2, 0, 0))


d = DebugData()
d.addPolyLine(getHelixPoints(), radius=0.01)
show(d, (4, 0, 0))


d = DebugData()
d.addSphere([0, 0, 0], radius=0.3)
show(d, (6, 0, 0))


d = DebugData()
d.addFrame(vtk.vtkTransform(), scale=0.5, tubeRadius=0.03)
show(d, (0, 2, 0))


d = DebugData()
d.addArrow((0, 0, 0), (0, 1, 0))
show(d, (2, 2, 0))


d = DebugData()
d.addEllipsoid((0, 0, 0), radii=(0.5, 0.35, 0.2))
show(d, (4, 2, 0))


d = DebugData()
d.addTorus(radius=0.5, thickness=0.2)
show(d, (6, 2, 0))


d = DebugData()
d.addCone(origin=(0,0,0), normal=(0,1,0), radius=0.3, height=0.8, color=[1, 1, 0])
show(d, (0, 4, 0))


d = DebugData()
d.addCube(dimensions=[0.8, 0.5, 0.3], center=[0, 0, 0], color=[0, 1, 1])
show(d, (2, 4, 0))


d = DebugData()
d.addPlane(origin=[0, 0, 0], normal=[0, 0, 1], width=0.8, height=0.7, resolution=10, color=[0, 1, 0])
show(d, (4, 4, 0)).setProperty('Surface Mode', 'Surface with edges')

d = DebugData()
d.addCapsule(center=[0, 0, 0], axis=[1, 0, 0], length=1.0, radius=0.1, color=[0.5, 0.5, 1])
show(d, (6, 4, 0))


d = DebugData()
polyData = vnp.numpyToPolyData(np.random.random((1000, 3)))
vnp.addNumpyToVtk(polyData, np.arange(polyData.GetNumberOfPoints()), 'point_ids')
d.addPolyData(polyData)
show(d, (2.5, 5, 0)).setProperty('Color By', 'point_ids')


applogic.resetCamera(viewDirection=[0, 0.1, -1])
app.start()
