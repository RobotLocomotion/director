from director import visualization as vis
from director import consoleapp
from director import vtkAll as vtk
import numpy as np

app = consoleapp.ConsoleApp()
view = app.createView()
view.show()

t = vtk.vtkTransform()
obj = vis.showFrame(t, 'frame')
obj.setProperty('Trace', True)


# move the frame along a spiral to create a path trace
for theta in np.linspace(0, 30, 1000):
    p1 = np.array(t.GetPosition())
    p2 = np.array([theta*0.03, np.sin(theta)*0.1, np.cos(theta)*0.1])
    t.Translate(p2 - p1)
    t.Modified()


view.resetCamera()
app.start()
