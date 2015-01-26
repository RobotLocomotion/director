from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import affordanceitems
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback

app = ConsoleApp()

app.setupGlobals(globals())
app.showPythonConsole()

view = app.createView()
view.show()


robotsystem.create(view, globals())


#playbackPanel.widget.show()
#teleopPanel.widget.show()



def makeBox():
    aff = affordanceitems.BoxAffordanceItem('test box', view)
    aff.setProperty('Dimensions', [0.5, 0.2, 0.1])
    om.addToObjectModel(aff)
    vis.addChildFrame(aff)
    return aff


def testRegister():
    aff = makeBox()
    affordanceObjectManager.registerAffordance(aff)
    return aff


assert len(affordanceCollection.collection) == 0
aff = testRegister()

assert len(affordanceCollection.collection) == 1
assert aff.getProperty('uuid') in affordanceCollection.collection

affordanceObjectManager.removeAffordance(aff)

assert len(affordanceCollection.collection) == 0


app.start()
