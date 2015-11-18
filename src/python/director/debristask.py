from segmentation import getFootFramesFromReferenceFrame
import math

from ddapp.debugVis import DebugData


def getBoardCorners(params):
    axes = [np.array(params[axis]) for axis in ['xaxis', 'yaxis', 'zaxis']]
    widths = [np.array(params[axis])/2.0 for axis in ['xwidth', 'ywidth', 'zwidth']]
    edges = [axes[i] * widths[i] for i in xrange(3)]
    origin = np.array(params['origin'])
    return [
            origin + edges[0] + edges[1] + edges[2],
            origin - edges[0] + edges[1] + edges[2],
            origin - edges[0] - edges[1] + edges[2],
            origin + edges[0] - edges[1] + edges[2],
            origin + edges[0] + edges[1] - edges[2],
            origin - edges[0] + edges[1] - edges[2],
            origin - edges[0] - edges[1] - edges[2],
            origin + edges[0] - edges[1] - edges[2],
           ]

def getPointDistances(target, points):
    return np.array([np.linalg.norm(target - p) for p in points])


def computeClosestCorner(aff, referenceFrame):
    corners = getBoardCorners(aff.params)
    dists = getPointDistances(np.array(referenceFrame.GetPosition()), corners)
    return corners[dists.argmin()]


def computeGroundFrame(aff, referenceFrame):
    refAxis = np.array([0,-1,0])
    referenceFrame.TransformVector(refAxis, refAxis)
    axes = [np.array(aff.params[axis]) for axis in ['xaxis', 'yaxis', 'zaxis']]
    axisProjections = np.array([np.abs(np.dot(axis, refAxis)) for axis in axes])
    boardAxis = axes[axisProjections.argmax()]
    if np.dot(boardAxis, refAxis) < 0:
        boardAxis = -boardAxis
    xaxis = boardAxis
    zaxis = [0,0,1]
    yaxis = np.cross(zaxis, xaxis)
    xaxis = np.cross(yaxis, zaxis)
    closestCorner = computeClosestCorner(aff, referenceFrame)
    groundFrame = getTransformFromAxes(xaxis, yaxis, zaxis)
    groundFrame.PostMultiply()
    groundFrame.Translate(closestCorner[0], closestCorner[1], 0.0)
    return groundFrame


def showBoardDebug(affs=None):
    referenceFrame = vtk.vtkTransform()
    referenceFrame.Translate(0, 0, 5.0)
    affs = affs or om.getObjects()
    for obj in affs:
        if isinstance(obj, BlockAffordanceItem):
            d = DebugData()
            d.addSphere(computeClosestCorner(obj, referenceFrame), radius=0.015)
            showPolyData(d.getPolyData(), 'closest corner', parent='board debug', visible=True)
            showFrame(computeGroundFrame(obj, referenceFrame), 'ground frame', parent='board debug', visible=True)


def loadFeet():
    meshDir = os.path.join(app.getDRCBase(), 'software/models/atlas_v3/meshes')
    meshes = []
    for foot in ['l', 'r']:
        d = DebugData()
        d.addPolyData(io.readPolyData(os.path.join(meshDir, '%s_talus.stl' % foot)))
        d.addPolyData(io.readPolyData(os.path.join(meshDir, '%s_foot.stl' % foot)))
        meshes.append(d.getPolyData())
    return meshes


import vs as lcmvs
from ddapp import lcmUtils
triadCount = 0

def publishTriad(transform, collectionId=1234):

    global triadCount
    triadCount += 1

    o = lcmvs.obj_t()

    xyz = transform.GetPosition()
    rpy = transformUtils.rollPitchYawFromTransform(transform)

    o.roll, o.pitch, o.yaw = rpy
    o.x, o.y, o.z = xyz
    o.id = triadCount

    m = vs.obj_collection_t()
    m.id = collectionId
    m.name = 'stance_triads'
    m.type = lcmvs.obj_collection_t.AXIS3D
    m.nobjs = 1
    m.reset = False
    m.objs = [o]

    lcmUtils.publish('OBJ_COLLECTION', m)



def generateFeetForDebris(affs=None):

    affs = affs or om.getObjects()
    affs = [aff for aff in affs if isinstance(aff, BlockAffordanceItem)]

    stanceWidth = 0.20

    nameMap = {
              'board A'  : '02_2x6_1',
              'board B'  : '03_2x6_3',
              'board C'  : '04_4x4x40_4',
              'board E'  : '07_4x4x20_4',
              'board F'  : '09_2x4_4',
              'board G'  : '10_4x4x40_2',
              'board H'  : '12_2x4_3',
              }

    stances = {
              '02_2x6_1' : (-0.323615,-0.334221,0.66,1.54697),
              '03_2x6_3' : (-0.48553,-0.437538,0.7,1.57),
              '04_4x4x40_4' : (-0.65425,-0.527123,0.66,1.58678),
              '07_4x4x20_4' : (-0.378337,-0.364018,0.66,1.62164),
              '09_2x4_4' : (-0.353197,-0.450592,0.79,1.54697),
              '10_4x4x40_2' : (-0.581364,-0.0076541,0.82,1.58029),
              '12_2x4_3' : (-0.353395,-0.450592,0.79,1.54649),
              '14_2x4_1' : (-0.598935,-0.498268,0.66,1.56923),
              }


    referenceFrame = vtk.vtkTransform()
    referenceFrame.Translate(0, 0, 5.0)

    stanceRotation = 0.0
    stanceOffset = [-0.48, -0.08, 0]

    for aff in affs:
        name = aff.getProperty('Name')
        if name not in nameMap:
            print 'name not found:', name
            continue

        affGroundFrame = computeGroundFrame(aff, referenceFrame)
        stanceOffsetX, stanceOffsetY, pelvisHeight, stanceRotation = stances[nameMap[name]]

        stanceFrame, lfootFrame, rfootFrame = getFootFramesFromReferenceFrame(affGroundFrame, stanceWidth, math.degrees(stanceRotation), [stanceOffsetX, stanceOffsetY, 0.0])

        publishTriad(stanceFrame)

        showFrame(affGroundFrame, name + ' ground frame', parent='board debug', scale=0.15, visible=True)
        lfootFrame = showFrame(lfootFrame, name + ' lfoot frame', parent='board debug', scale=0.07)
        rfootFrame = showFrame(rfootFrame, name + ' rfoot frame', parent='board debug', scale=0.07)

        lfoot, rfoot = loadFeet()
        lfoot = showPolyData(lfoot, name + ' l_foot', parent='board debug')
        lfoot.actor.SetUserTransform(lfootFrame.transform)
        rfoot = showPolyData(rfoot, name + ' r_foot', parent='board debug')
        rfoot.actor.SetUserTransform(rfootFrame.transform)


        #for obj in [lfootFrame, rfootFrame]:
        #    obj.addToView(app.getDRCView())


generateFeetForDebris()

