from director.consoleapp import ConsoleApp
from director import roboturdf
from director import jointcontrol
from director import planplayback
from director import playbackpanel
from director import robotplanlistener
from director import robotviewbehaviors
from director import pointcloudlcm
from director import cameraview
from director import lcmUtils
from PythonQt import QtCore, QtGui
import drc as lcmdrc
import bot_core as lcmbotcore


# create the application
app = ConsoleApp()
view = app.createView()

# load robot state model
robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot model', view, colorMode='Textures')

# listen for robot state updates
robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')

# reload urdf from model publisher lcm
roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])

# load  playback model
playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback robot model', view, color=roboturdf.getRobotOrangeColor(), visible=False)

# initialize the playback panel
planPlayback = planplayback.PlanPlayback()
manipPlanner = robotplanlistener.ManipulationPlanDriver(ikPlanner=None)
playbackPanel = playbackpanel.PlaybackPanel(planPlayback, playbackRobotModel, playbackJointController, robotStateModel, robotStateJointController, manipPlanner)
manipPlanner.connectPlanReceived(playbackPanel.setPlan)

# initialize pointcloud lcm
pointcloudlcm.init(view)

# initialize camera view
cameraChannel = 'MULTISENSE_CAMERA_LEFT'
imageManager = cameraview.ImageManager()
imageManager.queue.addCameraStream(cameraChannel)
imageManager.addImage(cameraChannel)
cameraView = cameraview.CameraImageView(imageManager, cameraChannel, view=app.createView(useGrid=False))

# show widgets with a grid layout
w = QtGui.QWidget()
l = QtGui.QGridLayout(w)
l.addWidget(view, 0, 0)
l.addWidget(cameraView.view, 0, 1)
l.addWidget(playbackPanel.widget, 1, 0, 1, 2) # row, column, row span, column span
l.setContentsMargins(0, 0, 0, 0)
w.resize(1024, 600)
w.show()

# add lcm logplayer keyboard shortcuts
logCommander = robotviewbehaviors.KeyPressLogCommander(view)

# reset camera position when first robot state message is received
lcmUtils.captureMessageCallback('EST_ROBOT_STATE', lcmbotcore.robot_state_t, lambda x: view.resetCamera())

# start the application
app.start()
