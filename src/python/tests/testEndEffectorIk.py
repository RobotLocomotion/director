from director import robotsystem
from director.consoleapp import ConsoleApp
from director import transformUtils
from director import robotstate
from director import ikplanner
from director import visualization as vis
from director import objectmodel as om

import numpy as np
import time
import itertools



def onMatlabStartup(ikServer, startSuccess):
    assert startSuccess
    runTest()



def computeIk(goalFrame, constraints, ikParameters, seedPoseName, nominalPoseName):

    constraints[-2].referenceFrame = goalFrame
    constraints[-1].quaternion = goalFrame
    cs = ikplanner.ConstraintSet(robotSystem.ikPlanner, constraints, '', '')
    cs.seedPoseName = seedPoseName
    cs.nominalPoseName = nominalPoseName
    return cs.runIk()

def runTest():


    side = 'left'
    testTolerances = False
    renderAllSamples = True
    randomizeSamples = True
    samplesPerJoint = 10
    jointLimitPadding = np.radians(5)

    ikPlanner = robotSystem.ikPlanner
    jointController = robotSystem.robotStateJointController
    robotModel = robotSystem.robotStateModel

    if app.getTestingEnabled():
        samplesPerJoint = 2

    jointGroup = str('%s Arm' % side).title()
    jointNames = ikPlanner.getJointGroup(jointGroup)
    jointIndices = [robotstate.getDrakePoseJointNames().index(name) for name in jointNames]
    jointLimits = np.array([robotModel.model.getJointLimits(jointName) for jointName in jointNames])
    otherJoints = [name for name in robotstate.getDrakePoseJointNames() if name not in jointNames]
    jointSamples = []


    for name, limit in zip(jointNames, jointLimits):
        jointMin = limit[0] + jointLimitPadding
        jointMax = limit[1] - jointLimitPadding
        samples, spacing = np.linspace(jointMin, jointMax, samplesPerJoint, retstep=True)
        jointSamples.append(samples)
        print 'joint name:', name
        print 'joint range: [%.4f, %.4f]' % (limit[0], limit[1])
        print 'joint number of samples:', samplesPerJoint
        print 'joint sample spacing: %.4f' % spacing

    totalSamples = np.product([len(x) for x in jointSamples])
    print 'total number of samples:', totalSamples

    allSamples = list(itertools.product(*jointSamples))
    if randomizeSamples:
        np.random.shuffle(allSamples)

    linkName = ikPlanner.getHandLink(side)
    linkFrame = robotModel.getLinkFrame(linkName)

    constraints = []
    constraints.append(ikPlanner.createPostureConstraint('q_nom', otherJoints))
    constraints.extend(ikPlanner.createSixDofLinkConstraints(jointController.q, linkName))

    def setTolerance(distance, angleInDegrees):
        constraints[-1].angleToleranceInDegrees = angleInDegrees
        constraints[-2].upperBound = np.ones(3)*distance
        constraints[-2].lowerBound = np.ones(3)*-distance

    setTolerance(0.005, 0.5)

    ikParameters = ikplanner.IkParameters()
    ikParameters.setToDefaults()

    ikParameters.majorIterationsLimit = 10000
    ikParameters.majorOptimalityTolerance = 1e-4
    ikParameters.majorFeasibilityTolerance = 1e-6

    #seedPoseName = 'q_nom'
    #nominalPoseName = 'q_nom'
    seedPoseName = 'sample_pose'
    nominalPoseName = 'sample_pose'

    print
    print 'constraints:'
    print
    print constraints[-2]
    print
    print constraints[-1]
    print
    print ikParameters
    print
    print 'seed pose name:', seedPoseName
    print 'nominal pose name:', nominalPoseName
    print


    ikPlanner.addPose(jointController.q, 'sample_pose')
    endPose, info = computeIk(linkFrame, constraints, ikParameters, seedPoseName, nominalPoseName)

    assert info == 1
    assert np.allclose(endPose, jointController.q)


    q = jointController.q.copy()
    nom_sample = q[jointIndices].copy()

    sampleCount = 0
    totalSampleCount = 0
    badSampleCount = 0
    sampleL2NormAccum = 0.0
    startTime = time.time()


    for sample in allSamples:

        sampleCount += 1
        totalSampleCount += 1

        dist = np.linalg.norm(sample - nom_sample)
        sampleL2NormAccum += dist


        q[jointIndices] = sample
        jointController.setPose('sample_pose', q)
        ikPlanner.addPose(q, 'sample_pose')

        if renderAllSamples:
            view.forceRender()

        targetFrame = robotModel.getLinkFrame(linkName)
        #pos, quat = transformUtils.poseFromTransform(frame)

        endPose, info = computeIk(targetFrame, constraints, ikParameters, seedPoseName, nominalPoseName)

        if info >= 10:
            print
            print 'bad info:', info
            jointController.addPose('bad', endPose)
            print 'sample num:', totalSampleCount
            print 'sample:', sample
            print

            badSampleCount += 1
            errorRate = badSampleCount/float(totalSampleCount)
            print 'error rate: %.2f' % errorRate
            print 'avg pose l2 norm:', sampleL2NormAccum/totalSampleCount

            if testTolerances:
                succeeded = False
                for tol in [(0.01, 1), (0.01, 2), (0.02, 2), (0.02, 3), (0.03, 3), (0.03, 5), (0.04, 5), (0.05, 5), (0.1, 10), (0.2, 20)]:

                    print 'retry tolerance:', tol
                    setTolerance(tol[0], tol[1])
                    endPose, info = computeIk(frame, constraints, ikParameters, seedPoseName, nominalPoseName)
                    if info < 10:
                        succeeded = True
                        print 'Worked!'
                        break

                setTolerance(0.005, 0.5)

                if not succeeded:
                    print 'Giving up after retries.'
                    continue


        timeNow = time.time()
        elapsed = timeNow - startTime
        if elapsed > 1.0:
            view.forceRender()
            print '%d samples/sec' % (sampleCount / elapsed), '%d total samples' % totalSampleCount
            startTime = timeNow
            sampleCount = 0


    if app.getTestingEnabled():
        assert badSampleCount == 0
        app.quit()


app = ConsoleApp()
app.setupGlobals(globals())
view = app.createView()
view.show()

robotSystem = robotsystem.create(view)

#robotSystem.ikPlanner.planningMode = 'pydrake'

if robotSystem.ikPlanner.planningMode == 'matlabdrake':
    robotSystem.ikServer.connectStartupCompleted(onMatlabStartup)
    robotSystem.startIkServer()
    app.start(enableAutomaticQuit=False)
else:
    runTest()

