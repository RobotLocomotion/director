import os
import vtkAll as vtk
from ddapp import botpy
import math
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp.utime import getUtime

import drc as lcmdrc


waitingForPlan = False


class WidgetCallback(TimerCallback):

    def __init__(self, frame):
        TimerCallback.__init__(self)

        self.transform = frame.transform
        self.lastMTime = self.transform.GetMTime()
        self.pending = False

    def tick(self):

        mtime = self.transform.GetMTime()

        if mtime == self.lastMTime and not self.pending:
            return

        self.lastMTime = mtime

        if not waitingForPlan:
            self.pending = False
            sendWalkingGoal()
        else:
            self.pending = True


def loadFootMeshes():
    meshDir = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot/meshes')
    meshes = []
    for foot in ['l', 'r']:
        d = DebugData()
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_talus.stl' % foot), computeNormals=True))
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_foot.stl' % foot), computeNormals=True))
        meshes.append(d.getPolyData())
    return meshes


def getLeftFootMesh():
    return getFootMeshes()[0]

def getRightFootMesh():
    return getFootMeshes()[1]

def getLeftFootColor():
    return [1.0, 1.0, 0.0]

def getRightFootColor():
    return [0.33, 1.0, 0.0]

_footMeshes = None

def getFootMeshes():
    global _footMeshes
    if not _footMeshes:
        _footMeshes = loadFootMeshes()
    return _footMeshes



def getFootstepsFolder():
    obj = om.findObjectByName('footstep plan')
    if obj is None:
        obj = om.getOrCreateContainer('footstep plan')
        #om.collapse(obj)
    return obj


def clearFootstepPlan():
    folder = getFootstepsFolder()
    om.removeFromObjectModel(folder)


def showFoot(transform, name, parentObj):


    obj = showPolyData(feet[0], 'fs0 mesh')
    obj.actor.SetUserTransform(t)


def onCandidateFootstepPlan(msg):


    clearFootstepPlan()

    planFolder = getFootstepsFolder()

    for i, footstep in enumerate(msg.footstep_goals[2:]):
        trans = footstep.pos.translation
        trans = [trans.x, trans.y, trans.z]
        quat = footstep.pos.rotation
        quat = [quat.w, quat.x, quat.y, quat.z]
        footstepTransform = transformUtils.transformFromPose(trans, quat)

        if footstep.is_right_foot:
            mesh = getRightFootMesh()
            color = getRightFootColor()
        else:
            mesh = getLeftFootMesh()
            color = getLeftFootColor()

        obj = vis.showPolyData(mesh, 'step %d' % i, color=color, alpha=1.0, parent=planFolder)
        frameObj = vis.showFrame(footstepTransform, 'frame', parent=obj, scale=0.3, visible=False)
        obj.actor.SetUserTransform(footstepTransform)

    global waitingForPlan
    waitingForPlan = False


def positionMessageFromFrame(transform):

    pos, wxyz = transformUtils.poseFromTransform(transform)

    trans = lcmdrc.vector_3d_t()
    trans.x, trans.y, trans.z = pos

    quat = lcmdrc.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = lcmdrc.position_3d_t()
    pose.translation = trans
    pose.rotation = quat
    return pose



def createWalkingGoal(transform):

    frameObj = vis.showFrame(transform, 'walking goal')
    callback = WidgetCallback(frameObj)
    callback.start()
    frameObj.callback = callback
    sendWalkingGoal()


def sendWalkingGoal():

    goalObj = om.findObjectByName('walking goal')
    assert goalObj

    msg = lcmUtils.loadMessage('walking_goal_t.pkl')
    msg.utime = getUtime()
    msg.goal_pos = positionMessageFromFrame(goalObj.transform)
    msg.max_num_steps = 100
    msg.follow_spline = 1
    msg.allow_optimization = False

    msg.nom_forward_step = 0.20

    global waitingForPlan
    waitingForPlan = True

    lcmUtils.publish('WALKING_GOAL', msg)
    return msg


def init():

    lcmUtils.addSubscriber('CANDIDATE_BDI_FOOTSTEP_PLAN', lcmdrc.footstep_plan_t, onCandidateFootstepPlan)


