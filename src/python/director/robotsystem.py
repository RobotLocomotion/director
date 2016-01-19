from director import irisdriver
from director import applogic
from director import vtkAll as vtk
from director import callbacks
from director.fieldcontainer import FieldContainer
from director import ik
from director import ikplanner
from director import raycastdriver
from director import objectmodel as om
from director import perception
from director import segmentation
from director import segmentationroutines
from director import robotstate
from director import roboturdf
from director import footstepsdriver
from director import drcargs
from director import atlasdriver
from director import affordanceitems
from director import affordancemanager
from director import robotplanlistener
from director import handdriver
from director import planplayback
from director import playbackpanel
from director import teleoppanel
from director import viewbehaviors
from director import plannerPublisher


def create(view=None, globalsDict=None):
    s = RobotSystem()
    return s.init(view, globalsDict)


class RobotSystem(object):


    def __init__(self):
        pass


    def init(self, view=None, globalsDict=None):

        view = view or applogic.getCurrentRenderView()

        useRobotState = True
        usePerception = True
        useFootsteps = True
        useHands = True
        usePlanning = True
        useAtlasDriver = True
        useAtlasConvexHull = False
        useWidgets = False

        directorConfig = drcargs.getDirectorConfig()
        neckPitchJoint = 'neck_ay'

        colorMode = 'URDF Colors'
        if 'colorMode' in directorConfig:
            assert directorConfig['colorMode'] in ['URDF Colors', 'Solid Color', 'Textures']
            colorMode = directorConfig['colorMode']


        if useAtlasDriver:
            atlasDriver = atlasdriver.init(None)


        if useRobotState:
            robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, urdfFile=directorConfig['urdfConfig']['robotState'], parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True, colorMode=colorMode)
            robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
            roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
            robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')
            segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)


        if usePerception:
            multisenseDriver, mapServerSource = perception.init(view)

            def getNeckPitch():
                return robotStateJointController.q[robotstate.getDrakePoseJointNames().index( neckPitchJoint )]
            neckDriver = perception.NeckDriver(view, getNeckPitch)

            def getSpindleAngleFunction():
                if (robotStateJointController.lastRobotStateMessage):
                    if ('hokuyo_joint' in robotStateJointController.lastRobotStateMessage.joint_name):
                        index = robotStateJointController.lastRobotStateMessage.joint_name.index('hokuyo_joint')
                        return (float(robotStateJointController.lastRobotStateMessage.utime)/(1e6),
                            robotStateJointController.lastRobotStateMessage.joint_position[index])
                return (0, 0)
            spindleMonitor = perception.SpindleMonitor(getSpindleAngleFunction)
            robotStateModel.connectModelChanged(spindleMonitor.onRobotStateChanged)



        if useHands:
            rHandDriver = handdriver.RobotiqHandDriver(side='right')
            lHandDriver = handdriver.RobotiqHandDriver(side='left')


        if useFootsteps:
            footstepsDriver = footstepsdriver.FootstepsDriver(robotStateJointController)
            irisDriver = irisdriver.IRISDriver(robotStateJointController, footstepsDriver.params)
            raycastDriver = raycastdriver.RaycastDriver()

        if usePlanning:

            ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', urdfFile=directorConfig['urdfConfig']['ik'], parent=None)
            om.removeFromObjectModel(ikRobotModel)
            ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
            ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))


            if 'leftFootLink' in directorConfig:
                ikServer = ik.AsyncIKCommunicator(directorConfig['urdfConfig']['ik'], directorConfig['fixedPointFile'], directorConfig['leftFootLink'], directorConfig['rightFootLink'], directorConfig['pelvisLink'])
            else: # assume that robot has no feet e.g. fixed base arm
                ikServer = ik.AsyncIKCommunicator(directorConfig['urdfConfig']['ik'], directorConfig['fixedPointFile'], '', '', '')

            def startIkServer():
                ikServer.startServerAsync()
                ikServer.comm.writeCommandsToLogFile = True

            #startIkServer()

            playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', view, urdfFile=directorConfig['urdfConfig']['playback'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False, colorMode=colorMode)
            teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', view, urdfFile=directorConfig['urdfConfig']['teleop'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False, colorMode=colorMode)

            if useAtlasConvexHull:
                chullRobotModel, chullJointController = roboturdf.loadRobotModel('convex hull atlas', view, urdfFile=urdfConfig['chull'], parent='planning',
                    color=roboturdf.getRobotOrangeColor(), visible=False)
                playbackJointController.models.append(chullRobotModel)


            planPlayback = planplayback.PlanPlayback()

            handFactory = roboturdf.HandFactory(robotStateModel)
            handModels = []

            for side in ['left', 'right']:
                if side in handFactory.defaultHandTypes:
                    handModels.append(handFactory.getLoader(side))

            ikPlanner = ikplanner.IKPlanner(ikServer, ikRobotModel, ikJointController, handModels)

            manipPlanner = robotplanlistener.ManipulationPlanDriver(ikPlanner)

            affordanceManager = affordancemanager.AffordanceObjectModelManager(view)
            affordanceitems.MeshAffordanceItem.getMeshManager().collection.sendEchoRequest()
            affordanceManager.collection.sendEchoRequest()
            segmentation.affordanceManager = affordanceManager

            plannerPub = plannerPublisher.PlannerPublisher(ikPlanner,affordanceManager,ikRobotModel)
            ikPlanner.setPublisher(plannerPub)

            # This joint angle is mapped to the Multisense panel
            neckPitchJoint = ikPlanner.neckPitchJoint

        applogic.resetCamera(viewDirection=[-1,0,0], view=view)




        if useWidgets:

            playbackPanel = playbackpanel.PlaybackPanel(planPlayback, playbackRobotModel, playbackJointController,
                                              robotStateModel, robotStateJointController, manipPlanner)

            teleopPanel = teleoppanel.TeleopPanel(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                             ikPlanner, manipPlanner, playbackPanel.setPlan, playbackPanel.hidePlan)

            footstepsDriver.walkingPlanCallback = playbackPanel.setPlan
            manipPlanner.connectPlanReceived(playbackPanel.setPlan)

        robotSystemArgs = dict(locals())
        for arg in ['globalsDict', 'self']:
            del robotSystemArgs[arg]

        if globalsDict is not None:
            globalsDict.update(robotSystemArgs)

        robotSystem = FieldContainer(**robotSystemArgs)
        viewbehaviors.ViewBehaviors.addRobotBehaviors(robotSystem)
        return robotSystem


