from director.componentgraph import ComponentFactory


class RobotSystemFactory(ComponentFactory):

    def initDefaultOptions(self, options):
        '''
        Components are enabled by default.  This function
        determines which components should be disabled.
        '''
        options.useConvexHullModel = False

    def addComponents(self, componentGraph):

        addComponent = componentGraph.addComponent

        addComponent('DirectorConfig', [])
        addComponent('RobotState', ['DirectorConfig'])
        addComponent('Segmentation', [])
        addComponent('SegmentationRobotState', ['Segmentation', 'RobotState'])
        addComponent('SegmentationAffordances', ['Segmentation', 'Affordances'])
        addComponent('PerceptionDrivers', ['RobotState', 'Planning'])
        addComponent('HandDrivers', [])
        addComponent('Footsteps', ['RobotState'])
        addComponent('RaycastDriver', ['Footsteps'])
        addComponent('IRISDriver', ['RobotState', 'Footsteps'])
        addComponent('AtlasDriver', [])
        addComponent('Planning', ['RobotState'])
        addComponent('Playback', ['Planning'])
        addComponent('Teleop', ['Planning', 'Playback', 'Affordances'])
        addComponent('ConvexHullModel', ['Playback'])
        addComponent('FootstepsPlayback', ['Footsteps', 'Playback'])
        addComponent('Affordances', [])
        addComponent('PlannerPublisher', ['Planning', 'Affordances'])
        addComponent('ViewBehaviors', ['Footsteps', 'PerceptionDrivers', 'Planning'])

    def initDirectorConfig(self, robotSystem):

        from director import drcargs

        directorConfig = drcargs.getDirectorConfig()

        if 'colorMode' not in directorConfig:
            defaultColorMode = 'URDF Colors'
            directorConfig['colorMode'] = defaultColorMode

        assert directorConfig['colorMode'] in ['URDF Colors', 'Solid Color', 'Textures']

        robotSystem._add_fields(directorConfig=directorConfig)

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

        robotSystem._add_fields(robotStateModel=robotStateModel,
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

        robotSystem._add_fields(multisenseDriver=multisenseDriver,
                                mapServerSource=mapServerSource,
                                neckDriver=neckDriver,
                                spindleMonitor=spindleMonitor)

    def initHandDrivers(self, robotSystem):

        from director import handdriver

        rHandDriver = handdriver.RobotiqHandDriver(side='right')
        lHandDriver = handdriver.RobotiqHandDriver(side='left')
        robotSystem._add_fields(rHandDriver=rHandDriver, lHandDriver=lHandDriver)

    def initFootsteps(self, robotSystem):

        from director import footstepsdriver
        footstepsDriver = footstepsdriver.FootstepsDriver(robotSystem.robotStateJointController)
        robotSystem._add_fields(footstepsDriver=footstepsDriver)

    def initRaycastDriver(self, robotSystem):

        from director import raycastdriver
        raycastDriver = raycastdriver.RaycastDriver()
        robotSystem._add_fields(raycastDriver=raycastDriver)

    def initIRISDriver(self, robotSystem):

        from director import irisdriver

        irisDriver = irisdriver.IRISDriver(robotSystem.robotStateJointController, robotSystem.footstepsDriver.params)
        robotSystem._add_fields(irisDriver=irisDriver)

    def initAtlasDriver(self, robotSystem):

        from director import atlasdriver

        atlasDriver = atlasdriver.init(None)
        robotSystem._add_fields(atlasDriver=atlasDriver)

    def initPlanning(self, robotSystem):

        from director import objectmodel as om
        from director import planningutils
        from director import roboturdf
        from director import ikplanner
        from director import ik


        directorConfig = robotSystem.directorConfig

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

        handFactory = roboturdf.HandFactory(robotSystem.robotStateModel)
        handModels = []

        for side in ['left', 'right']:
            if side in handFactory.defaultHandTypes:
                handModels.append(handFactory.getLoader(side))

        ikPlanner = ikplanner.IKPlanner(ikServer, ikRobotModel, ikJointController, handModels)

        planningUtils = planningutils.PlanningUtils(robotSystem.robotStateModel, robotSystem.robotStateJointController)

        robotSystem._add_fields(
            ikRobotModel=ikRobotModel,
            ikJointController=ikJointController,
            ikServer=ikServer,
            startIkServer=startIkServer,
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

        robotSystem._add_fields(
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


        robotSystem._add_fields(
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

        robotSystem._add_fields(
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
        affordanceitems.MeshAffordanceItem.getMeshManager().initLCM()
        affordanceitems.MeshAffordanceItem.getMeshManager().collection.sendEchoRequest()
        affordanceManager.collection.sendEchoRequest()

        robotSystem._add_fields(
            affordanceManager=affordanceManager,
            )

    def initSegmentationAffordances(self, robotSystem):

        from director import segmentation
        segmentation.affordanceManager = robotSystem.affordanceManager

    def initPlannerPublisher(self, robotSystem):

        from director import plannerPublisher

        plannerPub = plannerPublisher.PlannerPublisher(robotSystem.ikPlanner, robotSystem.affordanceManager, robotSystem.ikRobotModel)
        robotSystem.ikPlanner.setPublisher(plannerPub)

    def initViewBehaviors(self, robotSystem):

        from director import applogic
        from director import robotviewbehaviors

        viewBehaviors = robotviewbehaviors.RobotViewBehaviors(robotSystem.view, robotSystem)
        applogic.resetCamera(viewDirection=[-1,0,0], view=robotSystem.view)
        robotSystem._add_fields(viewBehaviors=viewBehaviors)



def create(view=None, globalsDict=None, options=None):
    '''
    Convenience function for initializing a robotSystem
    with the default options and populating a globals()
    dictionary with all the constructed objects.
    '''

    from director import applogic

    view = view or applogic.getCurrentRenderView()

    factory = RobotSystemFactory()
    options = options or factory.getDefaultOptions()
    robotSystem = factory.construct(options, view=view)

    if globalsDict is not None:
        globalsDict.update(dict(robotSystem))

    return robotSystem
