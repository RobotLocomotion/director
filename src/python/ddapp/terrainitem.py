from __future__ import division

import numpy as np

import ddapp.objectmodel as om
import ddapp.visualization as vis
from scipy.spatial.qhull import QhullError
from ddapp import transformUtils
from ddapp.debugVis import DebugData
# from ddapp import vtkAll as vtk
from ddapp.pointpicker import PlacerWidget
from PythonQt import QtGui

class TerrainRegionItem(vis.PolyDataItem):
    def __init__(self, name, view, seed_pose, terrain_segmentation):

        vis.PolyDataItem.__init__(self, name, None, view)
        self.transform = seed_pose
        d = DebugData()
        d.addSphere((0,0,0), radius=0.02)
        self.seedObj = vis.showPolyData(d.getPolyData(), 'region seed', color=[.8,.8,.1], parent=om.getOrCreateContainer('IRIS region seeds'))
        self.seedObj.actor.SetUserTransform(self.transform)
        self.frameObj = vis.showFrame(self.transform, 'region seed frame',
                                      scale=0.2,
                                      visible=False,
                                      parent=self.seedObj)
        self.frameObj.connectFrameModified(self.onFrameModified)
        terrain = om.findObjectByName('HEIGHT_MAP_SCENE')
        if terrain:
            self.placer = PlacerWidget(view, self.seedObj, terrain)
            self.placer.start()
        else:
            self.frameObj.setProperty('Edit', True)
            self.frameObj.setProperty('Visible', True)

        self.terrain_segmentation = terrain_segmentation
        self.safe_region = None
        self.addProperty('Visible', True)
        self.addProperty('Enabled for Walking', True)
        self.addProperty('Color', QtGui.QColor(200,200,20))
        self.addProperty('Alpha', 1.0)
        self.onFrameModified(self.frameObj)
        self.setProperty('Color', QtGui.QColor(200,200,20))

    def onFrameModified(self, frame):
        pos, wxyz = transformUtils.poseFromTransform(frame.transform)
        rpy = transformUtils.rollPitchYawFromTransform(frame.transform)
        pose = np.hstack((pos, rpy))
        debug = DebugData()
        safe_region = self.terrain_segmentation.findSafeRegion(pose, iter_limit=2)
        try:
            xy_verts = safe_region.xy_polytope()
            for j in range(xy_verts.shape[1]):
                z = pos[2]
                p1 = np.hstack((xy_verts[:,j], z))
                if j < xy_verts.shape[1] - 1:
                    p2 = np.hstack((xy_verts[:,j+1], z))
                else:
                    p2 = np.hstack((xy_verts[:,0], z))
                debug.addLine(p1, p2, color=[.8,.8,.2], radius=0.01)
            self.setPolyData(debug.getPolyData())
            self.safe_region = safe_region
        except QhullError:
            print "Could not generate convex hull (polytope is likely unbounded)."

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Enabled for Walking':
            if self.getProperty('Enabled for Walking'):
                self.setProperty('Alpha', 1.0)
            else:
                self.setProperty('Alpha', 0.25)
        if propertyName == 'Visible':
            self.seedObj.setProperty('Visible', self.getProperty('Visible'))

    def onRemoveFromObjectModel(self):
        om.removeFromObjectModel(self.frameObj)
        om.removeFromObjectModel(self.seedObj)
        vis.PolyDataItem.onRemoveFromObjectModel(self)
