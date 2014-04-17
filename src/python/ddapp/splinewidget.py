import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp import transformUtils
from ddapp import applogic as app
from ddapp import vtkAll as vtk
import numpy as np





splineWidget = None


def computeHandleParameterization():

    global splineWidget
    rep = splineWidget.GetRepresentation()

    handlePoints = []
    for i in xrange(rep.GetNumberOfHandles()):
        pt = [0.0, 0.0, 0.0]
        rep.GetHandlePosition(i, pt)
        handlePoints.append(pt)

    distanceBetweenPoints = np.sqrt(np.sum(np.diff(handlePoints, axis=0)**2, axis=1))
    totalLength = np.sum(distanceBetweenPoints)

    handleParameterization = [0.0]
    for dist in distanceBetweenPoints:
        handleParameterization.append((handleParameterization[-1] + dist))

    assert handleParameterization[-1] == totalLength
    handleParameterization = np.array(handleParameterization) / totalLength
    #print 'handle parameterization:', handleParameterization
    return handleParameterization


def updateSplineWidget(frame):


    #palmFrame = handFactory.getLoader(leftHandType).getPalmToWorldTransform()
    palmFrame = robotModel.getLinkFrame('l_hand_face')
    handPos = np.array(palmFrame.GetPosition())

    goalFrame = frame.transform
    goalPos = np.array(goalFrame.GetPosition())

    up = np.array([0.0, 0.0, 1.0])

    goalUp = [0.0, -1.0, 0.0]
    goalFrame.TransformVector(goalUp, goalUp)
    goalUp = np.array(goalUp)
    offset = 0.2

    global splineWidget
    rep = splineWidget.GetRepresentation()

    handlePoints = [
        handPos,
        handPos + offset*up,
        #handPos + (offset+0.001)*up,
        #handPos + (offset+0.002)*up,
        #goalPos + (offset+0.002)*goalUp,
        #goalPos + (offset+0.001)*goalUp,
        goalPos + offset*goalUp,
        goalPos,
    ]


    rep.SetNumberOfHandles(len(handlePoints))
    for i, pt in enumerate(handlePoints):
        rep.SetHandlePosition(i, pt)


    def interpolateSplineFrame(u):

        assert 0.0 <= u <= 1.0

        pt = [0.0, 0.0, 0.0]
        rep.GetParametricSpline().Evaluate([u,0.0,0.0], pt, range(9))

        handleParameterization = computeHandleParameterization()

        if u >= handleParameterization[-2]:
            #print 'using goal frame for u:', u
            t = transformUtils.copyFrame(goalFrame)
        #elif u <= handleParameterization[1]:
        #    print 'using palm frame for u:', u
        #    t = transformUtils.copyFrame(palmFrame)
        else:
            uu = (u - handleParameterization[0]) / (handleParameterization[-2] - handleParameterization[0])
            #print 'rescaling u:', u, '-->', uu
            t = transformUtils.frameInterpolate(palmFrame, goalFrame, uu)

        t.PostMultiply()
        t.Translate(np.array(pt) - np.array(t.GetPosition()))

        return t

    global splineInterp
    splineInterp = interpolateSplineFrame

    '''
    om.removeFromObjectModel(om.findObjectByName('sampled hands'))
    handFolder = om.getOrCreateContainer('sampled hands')
    numberOfHandSamples = 10

    for i in xrange(numberOfHandSamples):
        t = interpolateSplineFrame(i/float(numberOfHandSamples-1))
        handObj, f = placeHandModelWithTransform(t, view, parent=handFolder)
        handObj.setProperty('Alpha', 0.2)
    '''

    view.render()


def removeSplineWidget():

    global splineWidget
    if splineWidget:
        splineWidget.Off()
        splineWidget.SetInteractor(None)
        splineWidget = None


def createSplineWidget(view):

    global splineWidget
    if splineWidget is None:
        splineWidget = vtk.vtkSplineWidget2()
        splineWidget.SetInteractor(view.renderWindow().GetInteractor())
        splineWidget.On()
        rep = splineWidget.GetRepresentation()
        rep.SetNumberOfHandles(4)

    return splineWidget


def newSpline(handObj, view):

    removeSplineWidget()
    createSplineWidget(view)

    handFrame = handObj.getChildFrame()
    handFrame.connectFrameModified(updateSplineWidget)
    updateSplineWidget(handFrame)


def makeTransformInterpolator():

    assert splineWidget

    interp = vtk.vtkTransformInterpolator()
    interp.GetPositionInterpolator().SetInterpolatingSpline(vtk.vtkCardinalSpline())
    #interp.SetInterpolationTypeToLinear()

    rep = splineWidget.GetRepresentation()

    numberOfPoints = rep.GetNumberOfHandles()

    for i in xrange(numberOfPoints):
        pt = i/float(numberOfPoints-1)

        t = getSplineFrame(i)
        t.PreMultiply()
        t.RotateZ(i*5)
        t.RotateX(i*5)

        interp.AddTransform(pt, t)

    return interp


def interpolateReach():

    t = vtk.vtkTransform()
    handObj, handFrame = handFactory.placeHandModelWithTransform(t, view, side='left', name='grasp interpolation', parent='debug')
    handObj.setProperty('Alpha', 0.2)
    handFrame.setProperty('Visible', True)

    #global interp
    #interp = makeTransformInterpolator()

    sliderMax = 100.0

    def sliderChanged(sliderValue):
        sliderValue = sliderValue/float(sliderMax)
        t = splineInterp(sliderValue)
        handFrame.copyFrame(t)

        reachGoal = om.findObjectByName('reach goal left')
        if reachGoal:
            reachGoal.copyFrame(t)


    global slider
    slider = QtGui.QSlider(QtCore.Qt.Horizontal)
    slider.connect('valueChanged(int)', sliderChanged)
    slider.setMaximum(sliderMax)
    slider.show()
    slider.resize(500, 30)
    sliderChanged(sliderMax)


def getSplineFrame(handleId):

    assert splineWidget
    rep = splineWidget.GetRepresentation()

    goalPos = range(3)
    rep.GetHandlePosition(handleId, goalPos)

    palmFrame = robotModel.getLinkFrame('l_hand_face')
    palmFrame.PostMultiply()
    delta = np.array(goalPos) - np.array(palmFrame.GetPosition())
    palmFrame.Translate(delta)
    return palmFrame


def planToSplineHandle(handleId):

    if not splineWidget:
        return

    goalFrame = om.findObjectByName('reach goal left')
    if not goalFrame:
        return

    goalFrame.copyFrame(getSplineFrame(handleId))



def init(view_, handFactory_, robotModel_):

    global view
    view = view_

    global handFactory
    handFactory = handFactory_

    global robotModel
    robotModel = robotModel_

    #app.addToolbarMacro('interpolate reach', interpolateReach)
