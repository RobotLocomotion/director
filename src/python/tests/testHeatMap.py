from director import consoleapp
from director import objectmodel as om
from director import visualization as vis
from director import vtkAll as vtk
from director import applogic
from director import vtkNumpy as vnp

import numpy as np




app = consoleapp.ConsoleApp()
view = app.createView()

cellSize = 0.5
numberOfCells = 20

grid = vtk.vtkGridSource()
grid.SetScale(cellSize)
grid.SetGridSize(numberOfCells)
grid.SetSurfaceEnabled(True)
grid.Update()
grid = grid.GetOutput()


pts = vnp.getNumpyFromVtk(grid, 'Points')
heatMap = np.random.randn(len(pts))

vnp.addNumpyToVtk(grid, heatMap, 'heat_map')

gridObj = vis.showPolyData(grid, 'heat map', colorByName='heat_map', parent='scene')
gridObj.setProperty('Surface Mode', 'Surface with edges')

applogic.resetCamera()

view.show()
app.start()
