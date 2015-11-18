import os
import director
from director.consoleapp import ConsoleApp
from director import applogic
from director import visualization as vis
from director import otdfmodel


app = ConsoleApp()

view = app.createView()
filename = os.path.join(director.getDRCBaseDir(), 'software/models/otdf/cinderblockstep.otdf')
otdfmodel.openOtdf(filename, view)

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
