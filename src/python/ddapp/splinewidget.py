import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp import transformUtils
from ddapp import applogic as app
from ddapp import vtkAll as vtk
import numpy as np



class SplineEndEffectorPlanner(object):

    def __init__(self, handFactory, robotModel, view):
        self.handFactory = handFactory
        self.robotModel = robotModel
        self.view = view
        self.side = None
        self.rotationStartIndex = 0
        self.rotationEndIndex = -2
        self.splineWidget = vtk.vtkSplineWidget2()
        self.splineWidget.SetInteractor(self.view.renderWindow().GetInteractor())
        self.observerTag = self.splineWidget.AddObserver('InteractionEvent', self.onWidgetInteractionEvent)


    def __del__(self):
        self.splineWidget.Off()
        self.splineWidget.SetInteractor(None)
        self.splineWidget = None

    def getSplineWidget(self):
        '''
        Returns the vtkSplineWidget2 object
        '''
        return self.splineWidget

    def getHandlePoints(self):
        '''
        Returns a list of xyz points that are the handle points.
        '''
        rep = self.splineWidget.GetRepresentation()

        handlePoints = []
        for i in xrange(rep.GetNumberOfHandles()):
            pt = [0.0, 0.0, 0.0]
            rep.GetHandlePosition(i, pt)
            handlePoints.append(pt)

        return handlePoints


    def setHandlePoints(self, handlePoints):
        '''
        Set the spline handle points, given a list of xyz points.
        '''
        rep = self.splineWidget.GetRepresentation()
        rep.SetNumberOfHandles(len(handlePoints))
        for i, pt in enumerate(handlePoints):
            rep.SetHandlePosition(i, pt)


    def computeHandleParameterization(self):
        '''
        Returns the list of handle parameterized coordinates.  A coordinate is
        a value between 0.0 and 1.0, with 0.0 being the start and 1.0 being the
        end.  Handles are parameterized by distance along the spline where distance
        is computed as the straight line distance between two handle points.
        '''
        handlePoints = self.getHandlePoints()

        distanceBetweenPoints = np.sqrt(np.sum(np.diff(handlePoints, axis=0)**2, axis=1))
        totalLength = np.sum(distanceBetweenPoints)

        handleParameterization = [0.0]
        for dist in distanceBetweenPoints:
            handleParameterization.append((handleParameterization[-1] + dist))

        assert handleParameterization[-1] == totalLength
        handleParameterization = np.array(handleParameterization) / totalLength
        #print 'handle parameterization:', handleParameterization
        return handleParameterization

    @staticmethod
    def getPalmFrame(robotModel, side):
        linkName = '%s_hand_face' % side[0]
        return robotModel.getLinkFrame(linkName)


    @staticmethod
    def getReachGoalFrame(side):
        return om.findObjectByName('%s_hand constraint frame' % side[0])


    def createSplineInterpolationMethod(self, palmFrame, goalFrame):

        handPos = np.array(palmFrame.GetPosition())
        goalPos = np.array(goalFrame.GetPosition())

        up = np.array([0.0, 0.0, 1.0])

        goalUp = [0.0, -1.0, 0.0]
        goalFrame.TransformVector(goalUp, goalUp)
        goalUp = np.array(goalUp)
        offset = 0.2

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

        def interpolateSplineFrame(u):

            assert 0.0 <= u <= 1.0

            pt = [0.0, 0.0, 0.0]
            self.splineWidget.GetRepresentation().GetParametricSpline().Evaluate([u,0.0,0.0], pt, range(9))

            handleParameterization = self.computeHandleParameterization()

            if u >= handleParameterization[self.rotationEndIndex]:
                uu = 1.0

            elif u <= handleParameterization[self.rotationStartIndex]:
                uu = 0.0
            else:
                uu = (u - handleParameterization[self.rotationStartIndex]) / (handleParameterization[self.rotationEndIndex] - handleParameterization[self.rotationStartIndex])

            #print 'rescaled u:', u, '-->', uu

            t = transformUtils.frameInterpolate(palmFrame, goalFrame, uu)
            t.PostMultiply()
            t.Translate(np.array(pt) - np.array(t.GetPosition()))

            return t

        return handlePoints, interpolateSplineFrame


    def updateSplineWidget(self, frame):

        palmFrame = self.getPalmFrame(self.robotModel, self.side)
        goalFrame = frame.transform

        handlePoints, self.splineInterp = self.createSplineInterpolationMethod(palmFrame, goalFrame)

        self.setHandlePoints(handlePoints)

        self.showHandSamples()

        self.view.render()


    def onWidgetInteractionEvent(self, o, e):
        folder = om.findObjectByName('sampled hands')
        if not folder:
            return

        handObjs = folder.children()
        numberOfSamples = len(handObjs)

        for i, handObj in enumerate(handObjs):
            t = self.splineInterp(i/float(numberOfSamples-1))
            handObj.children()[0].copyFrame(t)


    def showHandSamples(self, numberOfSamples=15):

        om.removeFromObjectModel(om.findObjectByName('sampled hands'))
        handFolder = om.getOrCreateContainer('sampled hands', parentObj=om.getOrCreateContainer('debug'))

        for i in xrange(numberOfSamples):
            t = self.splineInterp(i/float(numberOfSamples-1))
            handObj, f = self.handFactory.placeHandModelWithTransform(t, self.view, side=self.side, name='sample %d' % i, parent=handFolder)
            handObj.setProperty('Alpha', 0.3)

        handFolder.setProperty('Visible', False)


    def getSplineSegmentSamples(self):
        params = self.computeHandleParameterization()
        segments = zip(params, params[1:])
        times = [np.linspace(segment[0], segment[1], 6) for segment in segments]
        times = [[0.0, 0.25,  0.5], np.linspace(params[-2], params[-1], 6)]
        #times = np.linspace(params[-2], params[-1], 6)

        times = np.hstack(times)
        times = np.unique(times)
        return times


    def computeIkPostures(self, samples, constraintSet):

        poses = []
        infos = []
        for u in samples:
            t = self.splineInterp(u)
            print u, t.GetPosition()
            reachGoal = self.getReachGoalFrame(self.side)
            print 'copying frame...'
            reachGoal.copyFrame(t)
            endPose, info = constraintSet.runIk()
            poses.append(list(endPose))
            infos.append(info)

        return poses, np.array(infos)


    def makeSplineGraspConstraints(self, ikPlanner, positionTolerance=0.03, angleToleranceInDegrees=15):

        params = self.computeHandleParameterization()
        segments = zip(params, params[1:])
        #times = [np.linspace(segment[0], segment[1], 6) for segment in segments]
        #times = [[0.0, 0.3, 0.5], np.linspace(params[-2], params[-1], 6)]

        times = np.linspace(params[-2], params[-1], 6)
        times = np.hstack(times)
        times = np.unique(times)

        frames = []
        for t in times:
            frames.append(self.splineInterp(t))


        folder = om.getOrCreateContainer('constraint spline samples', parentObj=om.getOrCreateContainer('debug'))
        for f in frames:
            vis.showFrame(f, 'frame', scale=0.1)


        side = self.side
        graspToPalm = vtk.vtkTransform()
        graspToHand = ikPlanner.newGraspToHandFrame(side, graspToPalm)

        constraints = []

        for f, t in zip(frames[:-1], times[:-1]):
            graspToWorld = f
            p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)

            p.lowerBound = np.tile(-positionTolerance, 3)
            p.upperBound = np.tile(positionTolerance, 3)
            q.angleToleranceInDegrees = angleToleranceInDegrees

            if t >= params[-2]:
                q.angleToleranceInDegrees = 0
                p.lowerBound = np.tile(-0.0, 3)
                p.upperBound = np.tile(0.0, 3)
                constraints.append(q)

            p.tspan = [t, t]
            q.tspan = [t, t]
            constraints.append(p)


        return constraints


    def show(self):
        self.splineWidget.On()

    def hide(self):
        self.splineWidget.Off()

    def newSpline(self, handObj, side):
        self.side = side
        handFrame = handObj.getChildFrame()
        handFrame.connectFrameModified(self.updateSplineWidget)
        self.updateSplineWidget(handFrame)
        self.show()


    def interpolateReach(self):

        t = vtk.vtkTransform()
        handObj, handFrame = self.handFactory.placeHandModelWithTransform(t, self.view, side=self.side, name='grasp interpolation', parent='debug')
        handObj.setProperty('Alpha', 0.2)
        handFrame.setProperty('Visible', True)

        sliderMax = 100.0

        def sliderChanged(sliderValue):
            sliderValue = sliderValue/float(sliderMax)
            t = self.splineInterp(sliderValue)
            handFrame.copyFrame(t)

            reachGoal = self.getReachGoalFrame(self.side)
            if reachGoal:
                reachGoal.copyFrame(t)


        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider.connect('valueChanged(int)', sliderChanged)
        self.slider.setMaximum(sliderMax)
        self.slider.show()
        self.slider.resize(500, 30)
        sliderChanged(sliderMax)


def init(view, handFactory, robotModel):

    global planner
    planner = SplineEndEffectorPlanner(handFactory, robotModel, view)

    #app.addToolbarMacro('interpolate reach', planner.interpolateReach)
