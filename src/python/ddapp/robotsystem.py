from ddapp import applogic
from ddapp import vtkAll as vtk
from ddapp import callbacks
from ddapp.fieldcontainer import FieldContainer
from ddapp import ik
from ddapp import ikplanner
from ddapp import irisdriver
from ddapp import raycastdriver
from ddapp import objectmodel as om
from ddapp import perception
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import robotstate
from ddapp import roboturdf
from ddapp import footstepsdriver
from ddapp import drcargs
from ddapp import atlasdriver
from ddapp import affordanceitems
from ddapp import affordancemanager
from ddapp import robotplanlistener
from ddapp import handdriver
from ddapp import planplayback
from ddapp import playbackpanel
from ddapp import teleoppanel
from ddapp import viewbehaviors
from ddapp import plannerPublisher

import os
import json


def create(view=None, globalsDict=None):
    s = RobotSystem()
    return s.init(view, globalsDict)

class RobotSystem(object):


    def __init__(self):
        pass


    @staticmethod
    def getDirectorConfig():

        with open(drcargs.args().directorConfigFile) as directorConfigFile:
            directorConfig = json.load(directorConfigFile)
            directorConfigDirectory = os.path.dirname(os.path.abspath(directorConfigFile.name))
            directorConfig['fixedPointFile'] = os.path.join(directorConfigDirectory, directorConfig['fixedPointFile'])
            urdfConfig = directorConfig['urdfConfig']
            for key, urdf in list(urdfConfig.items()):
                urdfConfig[key] = os.path.join(directorConfigDirectory, urdf)
        return directorConfig


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

        directorConfig = self.getDirectorConfig()


        if useAtlasDriver:
            atlasDriver = atlasdriver.init(None)


        if useRobotState:
            robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, urdfFile=directorConfig['urdfConfig']['robotState'], parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
            robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
            roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
            robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')
            segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)


        if usePerception:
            multisenseDriver, mapServerSource = perception.init(view)

            def getNeckPitch():
                return robotStateJointController.q[robotstate.getDrakePoseJointNames().index('neck_ay')]
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
                ikServer = ik.AsyncIKCommunicator(directorConfig['urdfConfig']['ik'], directorConfig['fixedPointFile'], directorConfig['leftFootLink'], directorConfig['rightFootLink'])
            else: # assume that robot has no feet e.g. fixed base arm
	        ikServer = ik.AsyncIKCommunicator(directorConfig['urdfConfig']['ik'], directorConfig['fixedPointFile'], "","")

            def startIkServer():
                ikServer.startServerAsync()
                ikServer.comm.writeCommandsToLogFile = True

            #startIkServer()

            playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', view, urdfFile=directorConfig['urdfConfig']['playback'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False)
            teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', view, urdfFile=directorConfig['urdfConfig']['teleop'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False)

            if useAtlasConvexHull:
                chullRobotModel, chullJointController = roboturdf.loadRobotModel('convex hull atlas', view, urdfFile=urdfConfig['chull'], parent='planning',
                    color=roboturdf.getRobotOrangeColor(), visible=False)
                playbackJointController.models.append(chullRobotModel)


            planPlayback = planplayback.PlanPlayback()

            if (roboturdf.numberOfHands == 1):
                handFactory = roboturdf.HandFactory(robotStateModel)
                handModels = [handFactory.getLoader(side) for side in ['left']]
            elif (roboturdf.numberOfHands == 2):
                handFactory = roboturdf.HandFactory(robotStateModel)
                handModels = [handFactory.getLoader(side) for side in ['left', 'right']]
            else:
                handFactory = None
                handModels = []


            ikPlanner = ikplanner.IKPlanner(ikServer, ikRobotModel, ikJointController, handModels)

            manipPlanner = robotplanlistener.ManipulationPlanDriver(ikPlanner)

            affordanceManager = affordancemanager.AffordanceObjectModelManager(view)
            affordanceitems.MeshAffordanceItem.getMeshManager().collection.sendEchoRequest()
            affordanceManager.collection.sendEchoRequest()
            segmentation.affordanceManager = affordanceManager

            plannerPub = plannerPublisher.PlannerPublisher(ikPlanner,affordanceManager)
            ikPlanner.setPublisher(plannerPub)

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


