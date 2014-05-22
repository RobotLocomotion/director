from __future__ import division

import numpy as np

import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp import transformUtils
from ddapp.debugVis import DebugData
from PythonQt import QtGui

class TerrainRegionItem(vis.PolyDataItem):
    def __init__(self, name, view, seed_pose, terrain_segmentation):

        vis.PolyDataItem.__init__(self, name, None, view)

        self.frameObj = vis.FrameItem('region seed', seed_pose, view)
        om.addToObjectModel(self.frameObj, om.getOrCreateContainer('IRIS region seeds'))
        self.frameObj.setProperty('Scale', 0.25)
        self.frameObj.setProperty('Visible', True)
        self.frameObj.setProperty('Edit', True)
        self.frameObj.connectFrameModified(self.onFrameModified)
        self.terrain_segmentation = terrain_segmentation
        self.safe_region = None
        self.addProperty('Visible', True)
        self.addProperty('Enabled for Walking', True)
        self.addProperty('Edit', True)
        self.addProperty('Color', QtGui.QColor(200,200,20))
        self.addProperty('Alpha', 1.0)
        self.onFrameModified(self.frameObj)
        self.setProperty('Color', QtGui.QColor(200,200,20))

    def onFrameModified(self, frame):
        pos, wxyz = transformUtils.poseFromTransform(frame.transform)
        rpy = transformUtils.rollPitchYawFromTransform(frame.transform)
        pose = np.hstack((pos, rpy))
        self.safe_region = self.terrain_segmentation.findSafeRegion(pose, iter_limit=2)
        debug = DebugData()
        xy_verts = self.safe_region.xy_polytope()
        for j in range(xy_verts.shape[1]):
            z = pos[2]
            p1 = np.hstack((xy_verts[:,j], z))
            if j < xy_verts.shape[1] - 1:
                p2 = np.hstack((xy_verts[:,j+1], z))
            else:
                p2 = np.hstack((xy_verts[:,0], z))
            debug.addLine(p1, p2, color=[.8,.8,.2])
        debug.addSphere(pos, color=[.8, .8, .2])
        self.setPolyData(debug.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Enabled for Walking':
            if self.getProperty('Enabled for Walking'):
                self.setProperty('Alpha', 1.0)
            else:
                self.setProperty('Alpha', 0.25)
        elif propertyName in ['Visible', 'Edit']:
            if self.getProperty('Visible') and self.getProperty('Edit'):
                self.frameObj.setProperty('Visible', True)
                self.frameObj.setProperty('Edit', True)
            else:
                self.frameObj.setProperty('Visible', False)
                self.frameObj.setProperty('Edit', False)


    def onRemoveFromObjectModel(self):
        om.removeFromObjectModel(self.frameObj)
        vis.PolyDataItem.onRemoveFromObjectModel(self)
