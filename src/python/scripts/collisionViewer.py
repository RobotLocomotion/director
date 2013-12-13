import os
import sys
import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.applogic as app
from ddapp import jointcontrol
from ddapp import lcmUtils
from ddapp import robotstate
from ddapp.timercallback import TimerCallback
import functools
import drc as lcmdrc



class PlanChecker(object):

    def __init__(self, model, jointController, view):
        self.model = model
        self.jointController = jointController
        self.view = view

        self.colorNoHighlight = QtGui.QColor(190, 190, 190)
        self.colorHighlight = QtCore.Qt.red

        self.timer = TimerCallback()
        self.timer.callback = self.update
        self.timer.targetFps = 60
        #self.timer.start()

        self.collisionStateIds = []

        self.lastRobotStateMessage = None
        self.lastPlanMessage = None
        self.lastPlanCheckMessage = None

        lcmUtils.addSubscriber('EST_ROBOT_STATE', lcmdrc.robot_state_t, self.onRobotState)
        lcmUtils.addSubscriber('ROBOT_COLLISIONS', lcmdrc.robot_collision_array_t, self.onPlanCheck)
        lcmUtils.addSubscriber('CANDIDATE_MANIP_PLAN', lcmdrc.robot_plan_w_keyframes_t, self.onManipPlan)

        w = QtGui.QWidget()
        l = QtGui.QHBoxLayout(w)
        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.clearButton = QtGui.QPushButton('clear')
        self.zeroButton = QtGui.QPushButton('zero')

        l.addWidget(self.clearButton)
        l.addWidget(self.zeroButton)
        l.addWidget(self.slider)

        self.slider.connect(self.slider, 'valueChanged(int)', self.onSlider)
        self.slider.connect(self.zeroButton, 'clicked()', self.onZeroButtonClicked)
        self.slider.connect(self.clearButton, 'clicked()', self.onClearButtonClicked)

        ww = QtGui.QWidget()
        ll = QtGui.QVBoxLayout(ww)
        ll.addWidget(self.view)
        ll.addWidget(w)
        ll.setMargin(0)
        ww.show()
        ww.resize(800, 600)
        self.widget = ww


    def gotoZeroPose(self):
        self.jointController.setZeroPose()
        self.view.render()

    def onZeroButtonClicked(self):
        self.onPlanCheck(self.lastPlanCheckMessage)
        self.zeroCamera()

    def onRobotState(self, message):
        self.lastRobotStateMessage = message

    def unhighlight(self):
        self.model.setColor(self.colorNoHighlight)

    def highlightLinks(self, names):
        self.unhighlight()
        for name in names:
            self.model.setLinkColor(name, self.colorHighlight)

    def highlightCollisionState(self, state):
        self.highlightLinks(state.links_in_collision)

    def onSlider(self):

        value = self.slider.value / 100.0

        if not self.lastPlanMessage:
            return

        numberOfStates = self.lastPlanMessage.num_states
        if not numberOfStates:
            return

        stateIndex = int(numberOfStates*value)

        state = self.lastPlanMessage.plan[stateIndex]
        self.showRobotState(state)
        self.highlightCollisionLinksForState(stateIndex)


    def zeroCamera(self):
        camera = view.camera()
        camera.SetPosition(1,0,0)
        camera.SetViewUp(0,0,1)
        view.resetCamera()


    def onClearButtonClicked(self):
        self.unhighlight()
        self.gotoZeroPose()
        self.zeroCamera()


    def onPlanCheck(self, message):

        self.unhighlight()
        self.gotoZeroPose()
        self.collisionStateIds = []
        if not message:
            return

        linkNamesInCollision = []
        for state in message.collision_states:
            linkNamesInCollision += state.links_in_collision
            self.collisionStateIds.append(state.plan_idx)

        linkNamesInCollision = set(linkNamesInCollision)

        self.highlightLinks(linkNamesInCollision)
        self.lastPlanCheckMessage = message


    def highlightCollisionLinksForState(self, stateId):

        self.unhighlight()

        linkNamesInCollision = []
        for state in self.lastPlanCheckMessage.collision_states:
            if state.plan_idx == stateId:
                linkNamesInCollision += state.links_in_collision

        self.highlightLinks(linkNamesInCollision)


    def onManipPlan(self, message):

        self.lastPlanMessage = message


    def showRobotState(self, msg):

        jointMap = {}
        for name, position in zip(msg.joint_name, msg.joint_position):
            jointMap[name] = position

        jointPositions = []
        for name in robotstate.getRobotStateJointNames():
            jointPositions.append(jointMap[name])

        robotState = [0.0] * 7
        robotState += jointPositions

        self.model.setEstRobotState(robotState)
        self.view.render()


    def update(self):
        self.updateRobotState()
        self.view.render()


    def updateRobotState(self):

        msg = self.lastRobotStateMessage
        if not msg:
            return

        self.showRobotState(msg)



def waitForRobotXML():
    print 'waiting for robot model...'
    msg = lcmUtils.captureMessage('ROBOT_MODEL', lcmdrc.robot_urdf_t)
    return app.loadRobotModelFromString(msg.urdf_xml_string)


def startup():

    app.setupPackagePaths()

    global view
    view = PythonQt.dd.ddQVTKWidgetView()

    modelFromFile = False
    defaultModelFile = 'model_LI_RR.urdf'

    if modelFromFile:
        mitRobotDir = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot')
        urdfFile = os.path.join(mitRobotDir, defaultModelFile)
        model = app.loadRobotModelFromFile(urdfFile)
    else:
        model = waitForRobotXML()

    model.addToRenderer(view.renderer())
    jointController = jointcontrol.JointController([model])
    jointController.setZeroPose()


    checker = PlanChecker(model, jointController, view)
    checker.zeroCamera()
    app.toggleCameraTerrainMode(view)

    QtCore.QCoreApplication.instance().exec_()


startup()
