from director.consoleapp import ConsoleApp
from director import robotsystem
from director import affordanceitems
from director import affordanceurdf
from director import objectmodel as om
from director import visualization as vis
from director import lcmUtils
from director import ioUtils
from director.debugVis import DebugData
from director.timercallback import TimerCallback
from director import segmentation
from director.uuidutil import newUUID
from director import geometryencoder
from director import sceneloader
import drc as lcmdrc
import os
import json
from director.utime import getUtime
app = ConsoleApp()

app.setupGlobals(globals())
app.showPythonConsole()

view = app.createView()
view.show()

robotsystem.create(view, globals())


def affordanceFromDescription(desc):
    affordanceManager.collection.updateDescription(desc)
    return affordanceManager.getAffordanceById(desc['uuid'])

def newBox():
    desc = dict(classname='BoxAffordanceItem', Name='test box', Dimensions=[0.5, 0.2, 0.1], uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)

def newSphere():
    desc = dict(classname='SphereAffordanceItem', Name='test sphere', Radius=0.2, uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)

def newCylinder():
    desc = dict(classname='CylinderAffordanceItem', Name='test cylinder', Radius=0.05, Length=0.5, uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)

def newCapsule():
    desc = dict(classname='CapsuleAffordanceItem', Name='test capsule', Radius=0.05, Length=0.5, uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)


def newCapsuleRing():
    desc = dict(classname='CapsuleRingAffordanceItem', Name='test capsule ring', uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)


def newMesh():

    d = DebugData()
    d.addArrow((0,0,0), (0,0,0.3))
    pd = d.getPolyData()
    meshId = affordanceitems.MeshAffordanceItem.getMeshManager().add(pd)

    desc = dict(classname='MeshAffordanceItem', Name='test mesh', Filename=meshId, uuid=newUUID(), pose=((0.5,0.0,1.0), (1,0,0,0)))
    return affordanceFromDescription(desc)


def loadTableTopPointCloud():
    dataDir = app.getTestingDataDirectory()
    polyData = ioUtils.readPolyData(os.path.join(dataDir, 'tabletop/table-and-door-scene.vtp'))
    return vis.showPolyData(polyData, 'pointcloud snapshot')

def segmentTableTopPointCloud():
    polyData = om.findObjectByName('pointcloud snapshot').polyData
    pickPoint = [-0.70420271,  0.64272028,  1.07856214]
    data = segmentation.segmentTableScene(polyData, pickPoint)
    vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')


def testCollection():

    affordanceCollection = affordanceManager.collection
    assert len(affordanceCollection.collection) == 0
    aff = newBox()

    assert len(affordanceCollection.collection) == 1
    assert aff.getProperty('uuid') in affordanceCollection.collection

    om.removeFromObjectModel(aff)

    assert len(affordanceCollection.collection) == 0


def testAffordanceToUrdf():

    affs = [func() for func in newSphere, newBox, newCylinder, newCapsule, newMesh]
    print affordanceurdf.urdfStringFromAffordances(affs)

    for aff in affs:
        om.removeFromObjectModel(aff)

def testSDF():
    print "Testind SDF loader"
    n_pre=len(affordanceManager.getAffordances())

    dataDir = app.getTestingDataDirectory()
    filename=os.environ['DRC_BASE'] + '/software/models/worlds/tabledemo.sdf'
    sc=sceneloader.SceneLoader()
    print "Loading "+filename
    sc.loadSDF(filename)

    n_post=len(affordanceManager.getAffordances())
    print "Number of affordances loaded: "+str(n_post-n_pre)
    assert n_post>n_pre


testCollection()
testAffordanceToUrdf()
loadTableTopPointCloud()
segmentTableTopPointCloud()
testSDF()

from director import affordancepanel
panel = affordancepanel.AffordancePanel(view, affordanceManager, robotStateJointController, raycastDriver)
panel.widget.show()


app.start()
