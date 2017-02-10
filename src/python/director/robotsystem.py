from director.componentgraph import ComponentFactory
from director.fieldcontainer import FieldContainer


class RobotSystemFactory(object):

    def getComponents(self):

        components = {
            'DirectorConfig' : [],
            'RobotState' : ['DirectorConfig'],
            'Segmentation' : [],
            'SegmentationRobotState' : ['Segmentation', 'RobotState'],
            'SegmentationAffordances' : ['Segmentation', 'Affordances'],
            'PerceptionDrivers' : ['RobotState', 'Planning'],
            'HandDrivers' : [],
            'Footsteps' : ['RobotState'],
            'RaycastDriver' : ['Footsteps'],
            'IRISDriver' : ['RobotState', 'Footsteps'],
            'AtlasDriver' : [],
            'Planning' : ['RobotState'],
            'Playback' : ['Planning'],
            'Teleop' : ['Planning', 'Playback', 'Affordances'],
            'ConvexHullModel' : ['Playback'],
            'FootstepsPlayback' : ['Footsteps', 'Playback'],
            'Affordances' : [],
            'PlannerPublisher' : ['Planning', 'Affordances'],
            'ViewBehaviors' : ['Footsteps', 'PerceptionDrivers', 'Planning', 'Affordances'],
            'RobotLinkSelector' : ['ViewBehaviors']}

        disabledComponents = [
            'ConvexHullModel',
            'RobotLinkSelector']

        return components, disabledComponents

    def initDirectorConfig(self, robotSystem):

        from director import drcargs

        directorConfig = drcargs.getDirectorConfig()

        if 'colorMode' not in directorConfig:
            defaultColorMode = 'URDF Colors'
            directorConfig['colorMode'] = defaultColorMode

        assert directorConfig['colorMode'] in ['URDF Colors', 'Solid Color', 'Textures']

        return FieldContainer(directorConfig=directorConfig)

    def initRobotState(self, robotSystem):

        from director import roboturdf

        robotStateModel, robotStateJointController = roboturdf.loadRobotModel(
            'robot state model',
            robotSystem.view,
            urdfFile=robotSystem.directorConfig['urdfConfig']['robotState'],
            color=roboturdf.getRobotGrayColor(),
            colorMode=robotSystem.directorConfig['colorMode'],
            parent='sensors',
            visible=True)

        robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
        roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
        robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')

        return FieldContainer(robotStateModel=robotStateModel,
                                robotStateJointController=robotStateJointController)

    def initSegmentation(self, robotSystem):
        from director import segmentation

    def initSegmentationRobotState(self, robotSystem):

        from director import segmentationroutines
        segmentationroutines.SegmentationContext.initWithRobot(robotSystem.robotStateModel)

    def initPerceptionDrivers(self, robotSystem):

        from director import perception
        from director import robotstate

        multisenseDriver, mapServerSource = perception.init(robotSystem.view)

        useNeckDriver = hasattr(robotSystem.ikPlanner, 'neckPitchJoint')
        if useNeckDriver:
            neckPitchJoint = robotSystem.ikPlanner.neckPitchJoint
            neckPitchIndex = robotstate.getDrakePoseJointNames().index(neckPitchJoint)

            def getNeckPitch():
                return robotSystem.robotStateJointController.q[neckPitchIndex]

            neckDriver = perception.NeckDriver(robotSystem.view, getNeckPitch)
        else:
            neckDriver = None


        spindleJoint = 'hokuyo_joint'

        def getSpindleAngleFunction():
            msg = robotSystem.robotStateJointController.lastRobotStateMessage
            if msg and spindleJoint in msg.joint_name:
                index = msg.joint_name.index(spindleJoint)
                return (float(msg.utime)/(1e6), msg.joint_position[index])
            else:
                return (0, 0)

        spindleMonitor = perception.SpindleMonitor(getSpindleAngleFunction)
        robotSystem.robotStateModel.connectModelChanged(spindleMonitor.onRobotStateChanged)

        return FieldContainer(multisenseDriver=multisenseDriver,
                                mapServerSource=mapServerSource,
                                neckDriver=neckDriver,
                                spindleMonitor=spindleMonitor)

    def initHandDrivers(self, robotSystem):

        from director import handdriver

        rHandDriver = handdriver.RobotiqHandDriver(side='right')
        lHandDriver = handdriver.RobotiqHandDriver(side='left')
        return FieldContainer(rHandDriver=rHandDriver, lHandDriver=lHandDriver)

    def initFootsteps(self, robotSystem):

        from director import footstepsdriver
        footstepsDriver = footstepsdriver.FootstepsDriver(robotSystem.robotStateJointController)
        return FieldContainer(footstepsDriver=footstepsDriver)

    def initRaycastDriver(self, robotSystem):

        from director import raycastdriver
        raycastDriver = raycastdriver.RaycastDriver()
        return FieldContainer(raycastDriver=raycastDriver)

    def initIRISDriver(self, robotSystem):

        from director import irisdriver

        irisDriver = irisdriver.IRISDriver(robotSystem.robotStateJointController, robotSystem.footstepsDriver.params)
        return FieldContainer(irisDriver=irisDriver)

    def initAtlasDriver(self, robotSystem):

        from director import atlasdriver

        atlasDriver = atlasdriver.init(None)
        return FieldContainer(atlasDriver=atlasDriver)

    def initPlanning(self, robotSystem):

        from director import objectmodel as om
        from director import planningutils
        from director import roboturdf
        from director import ikplanner


        directorConfig = robotSystem.directorConfig

        ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', urdfFile=directorConfig['urdfConfig']['ik'], parent=None)
        om.removeFromObjectModel(ikRobotModel)
        ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
        ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))

        handFactory = roboturdf.HandFactory(robotSystem.robotStateModel)
        handModels = []

        for side in ['left', 'right']:
            if side in handFactory.defaultHandTypes:
                handModels.append(handFactory.getLoader(side))

        ikPlanner = ikplanner.IKPlanner(ikRobotModel, ikJointController, handModels)

        planningUtils = planningutils.PlanningUtils(robotSystem.robotStateModel, robotSystem.robotStateJointController)

        return FieldContainer(
            ikRobotModel=ikRobotModel,
            ikJointController=ikJointController,
            handFactory=handFactory,
            handModels=handModels,
            ikPlanner=ikPlanner,
            planningUtils=planningUtils
            )

    def initConvexHullModel(self, robotSystem):

        from director import roboturdf

        directorConfig = robotSystem.directorConfig
        chullRobotModel, chullJointController = roboturdf.loadRobotModel('convex hull model', view, urdfFile=directorConfig['urdfConfig']['chull'], parent='planning', color=roboturdf.getRobotOrangeColor(), colorMode=directorConfig['colorMode'], visible=False)

        robotSystem.playbackJointController.models.append(chullRobotModel)

        return FieldContainer(
            chullRobotModel=chullRobotModel,
            chullJointController=chullJointController
            )

    def initPlayback(self, robotSystem):

        from director import roboturdf
        from director import planplayback
        from director import playbackpanel
        from director import robotplanlistener

        directorConfig = robotSystem.directorConfig

        manipPlanner = robotplanlistener.ManipulationPlanDriver(robotSystem.ikPlanner)

        playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', robotSystem.view, urdfFile=directorConfig['urdfConfig']['playback'], parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False, colorMode=directorConfig['colorMode'])

        planPlayback = planplayback.PlanPlayback()

        playbackPanel = playbackpanel.PlaybackPanel(planPlayback, playbackRobotModel, playbackJointController,
                                          robotSystem.robotStateModel, robotSystem.robotStateJointController, manipPlanner)

        manipPlanner.connectPlanReceived(playbackPanel.setPlan)


        return FieldContainer(
            playbackRobotModel=playbackRobotModel,
            playbackJointController=playbackJointController,
            planPlayback=planPlayback,
            manipPlanner=manipPlanner,
            playbackPanel=playbackPanel
            )

    def initTeleop(self, robotSystem):

        from director import roboturdf
        from director import teleoppanel

        directorConfig = robotSystem.directorConfig

        teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', robotSystem.view, urdfFile=directorConfig['urdfConfig']['teleop'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False, colorMode=directorConfig['colorMode'])


        teleopPanel = teleoppanel.TeleopPanel(robotSystem.robotStateModel, robotSystem.robotStateJointController, teleopRobotModel, teleopJointController,
                          robotSystem.ikPlanner, robotSystem.manipPlanner, robotSystem.affordanceManager, robotSystem.playbackPanel.setPlan, robotSystem.playbackPanel.hidePlan, robotSystem.planningUtils)

        return FieldContainer(
            teleopRobotModel=teleopRobotModel,
            teleopJointController=teleopJointController,
            teleopPanel=teleopPanel,
            )

    def initFootstepsPlayback(self, robotSystem):
        robotSystem.footstepsDriver.walkingPlanCallback = robotSystem.playbackPanel.setPlan

    def initAffordances(self, robotSystem):

        from director import affordancemanager
        from director import affordanceitems

        affordanceManager = affordancemanager.AffordanceObjectModelManager(robotSystem.view)
        affordanceitems.MeshAffordanceItem.getMeshManager()

        if affordancemanager.lcmobjectcollection.USE_LCM:
            affordanceitems.MeshAffordanceItem.getMeshManager().collection.sendEchoRequest()
            affordanceManager.collection.sendEchoRequest()

        return FieldContainer(
            affordanceManager=affordanceManager,
            )

    def initSegmentationAffordances(self, robotSystem):

        from director import segmentation
        segmentation.affordanceManager = robotSystem.affordanceManager

    def initPlannerPublisher(self, robotSystem):

        from director import plannerPublisher
        from director import pydrakeik
        from director import matlabik

        dummyPlannerPub = plannerPublisher.DummyPlannerPublisher(robotSystem.ikPlanner, robotSystem.affordanceManager)
        pyDrakePlannerPub = pydrakeik.PyDrakePlannerPublisher(robotSystem.ikPlanner, robotSystem.affordanceManager)
        exoticaPlannerPub = plannerPublisher.ExoticaPlannerPublisher(robotSystem.ikPlanner, robotSystem.affordanceManager)
        matlabPlannerPub = plannerPublisher.MatlabDrakePlannerPublisher(robotSystem.ikPlanner, robotSystem.affordanceManager)

        robotSystem.ikPlanner.addPublisher('dummy', dummyPlannerPub)
        robotSystem.ikPlanner.addPublisher('pydrake', pyDrakePlannerPub)
        robotSystem.ikPlanner.addPublisher('matlabdrake', matlabPlannerPub)
        robotSystem.ikPlanner.addPublisher('exotica', exoticaPlannerPub)

        directorConfig = robotSystem.directorConfig
        if 'planningMode' in directorConfig:
            robotSystem.ikPlanner.planningMode = directorConfig['planningMode']
        else:
            robotSystem.ikPlanner.planningMode = 'matlabdrake'


        linkNameArgs = ['','','']
        if 'leftFootLink' in directorConfig:
            linkNameArgs = [directorConfig['leftFootLink'], directorConfig['rightFootLink'], directorConfig['pelvisLink']]

        matlabIkServer = matlabik.AsyncIKCommunicator(directorConfig['urdfConfig']['ik'], directorConfig['fixedPointFile'], *linkNameArgs)

        def startIkServer():
            matlabIkServer.startServerAsync()
            matlabIkServer.comm.writeCommandsToLogFile = True

        matlabIkServer.handModels = robotSystem.ikPlanner.handModels
        matlabPlannerPub.ikServer = matlabIkServer

        robotSystem.ikPlanner.ikServer = matlabIkServer

        return FieldContainer(
            ikServer=matlabIkServer,
            startIkServer=startIkServer
            )

    def initRobotLinkSelector(self, robotSystem):

        from director import robotlinkselector
        robotLinkSelector = robotlinkselector.RobotLinkSelector()
        robotSystem.viewBehaviors.addHandler(
            robotSystem.viewBehaviors.LEFT_DOUBLE_CLICK_EVENT,
            robotLinkSelector.onLeftDoubleClick)
        return FieldContainer(robotLinkSelector=robotLinkSelector)

    def initViewBehaviors(self, robotSystem):

        from director import applogic
        from director import robotviewbehaviors

        viewBehaviors = robotviewbehaviors.RobotViewBehaviors(robotSystem.view, robotSystem)
        applogic.resetCamera(viewDirection=[-1,0,0], view=robotSystem.view)
        return FieldContainer(viewBehaviors=viewBehaviors)


def create(view=None, globalsDict=None, options=None, planningOnly=False):
    '''
    Convenience function for initializing a robotSystem
    with the default options and populating a globals()
    dictionary with all the constructed objects.
    '''

    from director import applogic

    view = view or applogic.getCurrentRenderView()

    factory = ComponentFactory()
    factory.register(RobotSystemFactory)
    options = options or factory.getDefaultOptions()

    if planningOnly:
        options = factory.getDisabledOptions()
        factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)

    robotSystem = factory.construct(options, view=view)

    if globalsDict is not None:
        globalsDict.update(dict(robotSystem))

    return robotSystem
