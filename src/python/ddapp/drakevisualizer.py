from ddapp import lcmUtils
from ddapp import jointcontrol
from ddapp import roboturdf

import ddapp.objectmodel as om
import ddapp.applogic as app

import drake as lcmdrake

from PythonQt import QtGui


class DrakeVisualizer(object):

    def __init__(self, view):
        lcmUtils.addSubscriber('DRAKE_VIEWER_COMMAND', lcmdrake.lcmt_viewer_command, self.onViewerCommand)
        lcmUtils.addSubscriber('DRAKE_VIEWER_STATE', lcmdrake.lcmt_robot_state, self.onViewerState)

        self.view = view
        self.models = []
        self.jointControllers = []
        self.filenames = []

    def onViewerCommand(self, msg):
        if msg.command_type == lcmdrake.lcmt_viewer_command.LOAD_URDF:
            msg.command_type = msg.STATUS
            lcmUtils.publish('DRAKE_VIEWER_STATUS', msg)
            urdfFile = msg.command_data
            for model in self.models:
                if model.model.filename() == urdfFile:
                    return
            self.loadURDF(urdfFile)

    def addRobotModelItem(self, model):
        obj = roboturdf.RobotModelItem(model)
        om.addToObjectModel(obj, om.getOrCreateContainer('Drake Viewer Models'))
        obj.setProperty('Color', QtGui.QColor(255, 180, 0))
        obj.addToView(self.view)
        return obj

    def loadURDF(self, filename):
        model = roboturdf.loadRobotModelFromFile(filename)
        jointController = jointcontrol.JointController([model])
        jointController.setZeroPose()
        obj = self.addRobotModelItem(model)
        self.models.append(obj)
        self.jointControllers.append(jointController)

    def onViewerState(self, msg):

          if not self.models:
              return

          if self.models[0] not in om.getObjects():
              self.models[0] = self.addRobotModelItem(self.models[0].model)

          assert msg.num_robots == 1
          pose = msg.joint_position
          self.jointControllers[0].setPose('drake_viewer_pose', pose)
