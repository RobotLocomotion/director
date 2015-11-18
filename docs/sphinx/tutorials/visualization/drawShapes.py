from ddapp.consoleapp import ConsoleApp
from ddapp.debugVis import DebugData
import ddapp.visualization as vis

# initialize application components
app = ConsoleApp()
view = app.createView()
view.showMaximized()

# create a sphere primitive
d = DebugData()
d.addSphere(center=(0,0,0), radius=0.5)

# show the sphere in the visualization window
sphere = vis.showPolyData(d.getPolyData(), 'sphere')
sphere.setProperty('Color', [0,1,0])

# start the application
app.start()
