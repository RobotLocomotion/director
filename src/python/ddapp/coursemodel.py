
#import ddapp
from ddapp import cameraview
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik
from ddapp.ikparameters import IkParameters
from ddapp.ikplanner import ConstraintSet
from ddapp import polarisplatformplanner
from ddapp import robotstate
from ddapp import segmentation
from ddapp import sitstandplanner
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp import planplayback
from ddapp import lcmUtils
from ddapp import affordancepanel
from ddapp.uuidutil import newUUID

import os
import functools
import numpy as np
import scipy.io
import vtkAll as vtk
import bot_core as lcmbotcore
from ddapp.tasks.taskuserpanel import TaskUserPanel
import ddapp.tasks.robottasks as rt
from ddapp import filterUtils
from ddapp import ioUtils
import ddapp
from numpy import array

class CourseModel(object):

    def __init__(self):
        
        pose = transformUtils.poseFromTransform(vtk.vtkTransform())


        self.pointcloud  = ioUtils.readPolyData(ddapp.getDRCBaseDir() + '/software/models/rehearsal_pointcloud.vtp')
        self.pointcloudPD = vis.showPolyData(self.pointcloud, 'coursemodel', parent=None)
        segmentation.makeMovable(self.pointcloudPD, transformUtils.transformFromPose(array([0, 0, 0]), array([ 1.0,  0.        ,  0.        , 0.0])))

        self.originFrame = self.pointcloudPD.getChildFrame()

        t = transformUtils.transformFromPose(array([-4.39364111, -0.51507392, -0.73125563]), array([ 0.93821625,  0.        ,  0.        , -0.34604951]))
        self.valveWalkFrame  = vis.updateFrame(t, 'ValveWalk', scale=0.2,visible=True, parent=self.pointcloudPD)

        t = transformUtils.transformFromPose(array([-3.31840048,  0.36408685, -0.67413123]), array([ 0.93449475,  0.        ,  0.        , -0.35597691]))
        self.drillPreWalkFrame = vis.updateFrame(t, 'DrillPreWalk', scale=0.2,visible=True, parent=self.pointcloudPD)

        t = transformUtils.transformFromPose(array([-2.24553758, -0.52990939, -0.73255338]), array([ 0.93697004,  0.        ,  0.        , -0.34940972]))
        self.drillWalkFrame  = vis.updateFrame(t, 'DrillWalk', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([-2.51306835, -0.92994004, -0.74173541 ]), array([-0.40456572,  0.        ,  0.        ,  0.91450893]))
        self.drillWallWalkFarthestSafeFrame  = vis.updateFrame(t, 'DrillWallWalkFarthestSafe', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([-2.5314524 , -0.27401861, -0.71302976]), array([ 0.98691519,  0.        ,  0.        , -0.16124022]))
        self.drillWallWalkBackFrame  = vis.updateFrame(t, 'DrillWallWalkBack', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([-1.16122318,  0.04723203, -0.67493468]), array([ 0.93163145,  0.        ,  0.        , -0.36340451]))
        self.surprisePreWalkFrame  = vis.updateFrame(t, 'SurprisePreWalk', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([-0.5176186 , -1.00151554, -0.70650799]), array([ 0.84226497,  0.        ,  0.        , -0.53906374]))
        self.surpriseWalkFrame  = vis.updateFrame(t, 'SurpriseWalk', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([-0.69100097, -0.43713269, -0.68495922]), array([ 0.98625075,  0.        ,  0.        , -0.16525575]))
        self.surpriseWalkBackFrame  = vis.updateFrame(t, 'SurpriseWalkBack', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([ 0.65827322, -0.08028796, -0.77370834]), array([ 0.94399977,  0.        ,  0.        , -0.3299461 ]))
        self.terrainPreWalkFrame = vis.updateFrame(t, 'TerrainPreWalk', scale=0.2,visible=True, parent=self.pointcloudPD)
        
        t = transformUtils.transformFromPose(array([ 5.47126425, -0.09790393, -0.70504679]), array([ 1.,  0.,  0.,  0.]))
        self.stairsPreWalkFrame = vis.updateFrame(t, 'StairsPreWalk', scale=0.2,visible=True, parent=self.pointcloudPD)

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.originFrame)
        self.frameSync.addFrame(self.pointcloudPD.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.valveWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.drillPreWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.drillWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.drillWallWalkFarthestSafeFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.drillWallWalkBackFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.surprisePreWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.surpriseWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.surpriseWalkBackFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.terrainPreWalkFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.stairsPreWalkFrame, ignoreIncoming=True)