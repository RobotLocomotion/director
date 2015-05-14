import os
import vtkAll as vtk

class JointPropagator(object):

    def __init__(self, parentRobotModel, childRobotModel, jointsToPropagate):
        self.parentRobotModel = parentRobotModel
        self.childRobotModel = childRobotModel

        # Figure out which joint indices we're going to
        # be propagating
        all_joint_names = parentRobotModel.model.getJointNames()
        if childRobotModel.model.getJointNames() != all_joint_names:
            raise Exception('parent and child robot model joints don\'t match!')
        all_joint_names = [str(joint) for joint in all_joint_names]
        self.jointIndices = []
        for joint in jointsToPropagate:
            if joint in all_joint_names:
                self.jointIndices.append(all_joint_names.index(joint))
            #else:
            #    raise Exception('Couldn\'t find dof ' + joint)
        self.jointsToPropagate = jointsToPropagate

    def doPropagation(self):
        current_positions = [i for i in self.childRobotModel.model.getJointPositions()]
        parent_positions = self.parentRobotModel.model.getJointPositions()
        for jointi in self.jointIndices:
            current_positions[jointi] = parent_positions[jointi]
        self.childRobotModel.model.setJointPositions(current_positions)
