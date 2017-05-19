from director import consoleapp
from director import pointselector
from director import visualization as vis
from director import vtkNumpy as vnp
import numpy as np

app = consoleapp.ConsoleApp()
view = app.createView()

randomPoints = np.random.random((1000,3))*3.0
polyData = vnp.numpyToPolyData(randomPoints, createVertexCells=True)
vis.showPolyData(polyData, 'pointcloud')

view.show()
view.forceRender()

selector = pointselector.PointSelector(view, polyData)
selector.pickArea((200,200), (300,300))

assert selector.getSelectedPoints().GetNumberOfPoints()

app.start()
