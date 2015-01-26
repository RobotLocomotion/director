import os
import ddapp
from ddapp.consoleapp import ConsoleApp
from ddapp import applogic
from ddapp import visualization as vis
from ddapp import otdfmodel


app = ConsoleApp()

view = app.createView()
filename = os.path.join(ddapp.getDRCBaseDir(), 'software/models/otdf/cinderblockstep.otdf')
otdfmodel.openOtdf(filename, view)

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
