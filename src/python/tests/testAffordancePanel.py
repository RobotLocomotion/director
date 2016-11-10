from director.consoleapp import ConsoleApp
from director import mainwindowapp
from director import affordancemanager
from director import affordanceitems
from director import affordanceurdf
from director import affordancepanel
from director import objectmodel as om
from director import visualization as vis
from director import pointpicker
from director import viewbehaviors
from director.debugVis import DebugData

from PythonQt import QtCore, QtGui

def newBox():
    desc = dict(classname='BoxAffordanceItem', Name='test box', Dimensions=[0.5, 0.2, 0.1], pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def newSphere():
    desc = dict(classname='SphereAffordanceItem', Name='test sphere', Radius=0.2, pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def newCylinder():
    desc = dict(classname='CylinderAffordanceItem', Name='test cylinder', Radius=0.05, Length=0.5, pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def newCapsule():
    desc = dict(classname='CapsuleAffordanceItem', Name='test capsule', Radius=0.05, Length=0.5, pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def newCapsuleRing():
    desc = dict(classname='CapsuleRingAffordanceItem', Name='test capsule ring', pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def newMesh():
    d = DebugData()
    d.addArrow((0,0,0), (0,0,0.3))
    pd = d.getPolyData()
    meshId = affordanceitems.MeshAffordanceItem.getMeshManager().add(pd)
    desc = dict(classname='MeshAffordanceItem', Name='test mesh', Filename=meshId, pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceManager.newAffordanceFromDescription(desc)

def testAffordanceToUrdf():

    affs = [func() for func in newSphere, newBox, newCylinder, newCapsule, newMesh]
    print affordanceurdf.urdfStringFromAffordances(affs)

    #for aff in affs:
    #    om.removeFromObjectModel(aff)


def printAffordanceUrdf():
    affs = affordanceManager.getAffordances()
    print affordanceurdf.urdfStringFromAffordances(affs)


def onAffordancePick(objs):

    obj = objs[0]

    if obj == affordanceToAdd:
        return

    print affordanceToAdd.getProperty('Name')
    print obj.getProperty('Name')


    frameSync = obj.getChildFrame().getFrameSync()
    frameSync.addFrame(affordanceToAdd.getChildFrame(), ignoreIncoming=True)


def getAffordanceContextMenuActions(view, pickedObj, pickedPoint):

    if pickedObj not in affordanceManager.getAffordances():
        return []

    global affordanceToAdd
    affordanceToAdd = pickedObj

    def onSelectAffordanceParent():
        objectPicker.start()

    actions = [
      (None, None),
      ('Select parent...', onSelectAffordanceParent),
      ]

    return actions


viewbehaviors.registerContextMenuActions(getAffordanceContextMenuActions)


app = mainwindowapp.MainWindowAppFactory().construct()
view = app.view


affordanceManager = affordancemanager.AffordanceObjectModelManager(view)

if affordancemanager.lcmobjectcollection.USE_LCM:
    affordanceitems.MeshAffordanceItem.getMeshManager().collection.sendEchoRequest()
    affordanceManager.collection.sendEchoRequest()

objectPicker = pointpicker.ObjectPicker(view=view, callback=onAffordancePick, getObjectsFunction=affordanceManager.getAffordances)


panel = affordancepanel.AffordancePanel(view, affordanceManager)
dock = app.app.addWidgetToDock(panel.widget, QtCore.Qt.RightDockWidgetArea)

printButton = QtGui.QPushButton('Print URDF')
printButton.connect('clicked()', printAffordanceUrdf)
panel.ui.spawnTab.layout().addWidget(printButton)

app.app.start()
