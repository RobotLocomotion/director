from __future__ import division

import numpy as np

import ddapp.objectmodel as om
import ddapp.visualization as vis
import ddapp.vtkNumpy as vnp
from scipy.spatial.qhull import QhullError
from ddapp import filterUtils
from ddapp import segmentation
from ddapp.debugVis import DebugData
from ddapp.pointpicker import PlacerWidget
from PythonQt import QtGui

class TerrainRegionItem(vis.PolyDataItem):
    def __init__(self, uid, view, seed_pose, irisDriver, existing_region=None):

        d = DebugData()
        self.uid = uid
        vis.PolyDataItem.__init__(self, "IRIS region {:d}".format(uid), d.getPolyData(), view)
        self.transform = seed_pose
        d.addSphere((0,0,0), radius=0.02)
        self.seedObj = vis.showPolyData(d.getPolyData(), 'region seed', parent=om.getOrCreateContainer('IRIS region seeds'))
        self.seedObj.actor.SetUserTransform(self.transform)
        self.frameObj = vis.showFrame(self.transform, 'region seed frame',
                                      scale=0.2,
                                      visible=False,
                                      parent=self.seedObj)
        self.frameObj.setProperty('Edit', True)

        self.frameObj.widget.HandleRotationEnabledOff()

        terrain = om.findObjectByName('HEIGHT_MAP_SCENE')
        if terrain:
            rep = self.frameObj.widget.GetRepresentation()
            rep.SetTranslateAxisEnabled(2, False)
            rep.SetRotateAxisEnabled(0, False)
            rep.SetRotateAxisEnabled(1, False)

            pos = np.array(self.frameObj.transform.GetPosition())
            polyData = filterUtils.removeNonFinitePoints(terrain.polyData)
            if polyData.GetNumberOfPoints():
                polyData = segmentation.labelDistanceToLine(polyData, pos, pos+[0,0,1])
                polyData = segmentation.thresholdPoints(polyData, 'distance_to_line', [0.0, 0.1])
                if polyData.GetNumberOfPoints():
                    pos[2] = np.nanmax(vnp.getNumpyFromVtk(polyData, 'Points')[:,2])
                    self.frameObj.transform.Translate(pos - np.array(self.frameObj.transform.GetPosition()))

            self.placer = PlacerWidget(view, self.seedObj, terrain)
            self.placer.start()
        else:
            self.frameObj.setProperty('Edit', True)
            self.frameObj.setProperty('Visible', True)


        self.driver = irisDriver
        self.safe_region = None
        self.addProperty('Visible', True)
        self.addProperty('Enabled for Walking', True)
        self.addProperty('Alpha', 1.0)
        self.addProperty('Color', QtGui.QColor(200,200,20))

        self.frameObj.connectFrameModified(self.onFrameModified)
        if existing_region is None:
            self.onFrameModified(self.frameObj)
        else:
            self.setRegion(existing_region)

        self.setProperty('Alpha', 0.5)
        self.setProperty('Color', QtGui.QColor(220,220,220))

    def setRegion(self, safe_region):
        debug = DebugData()
        pos = safe_region.point
        try:
            xy_verts = safe_region.xy_polytope()
            if xy_verts.shape[1] == 0:
                raise QhullError("No points returned")
            xyz_verts = np.vstack((xy_verts, pos[2] + 0.02 + np.zeros((1, xy_verts.shape[1]))))
            xyz_verts = np.hstack((xyz_verts, np.vstack((xy_verts, pos[2] + 0.015 + np.zeros((1, xy_verts.shape[1]))))))
            # print xyz_verts.shape
            polyData = vnp.getVtkPolyDataFromNumpyPoints(xyz_verts.T.copy())
            vol_mesh = filterUtils.computeDelaunay3D(polyData)
            for j in range(xy_verts.shape[1]):
                z = pos[2] + 0.005
                p1 = np.hstack((xy_verts[:,j], z))
                if j < xy_verts.shape[1] - 1:
                    p2 = np.hstack((xy_verts[:,j+1], z))
                else:
                    p2 = np.hstack((xy_verts[:,0], z))
                debug.addLine(p1, p2, color=[.7,.7,.7], radius=0.003)
            debug.addPolyData(vol_mesh)
            # self.setPolyData(vol_mesh)
            self.setPolyData(debug.getPolyData())
            self.safe_region = safe_region
        except QhullError:
            print "Could not generate convex hull (polytope is likely unbounded)."


    def onFrameModified(self, frame):
        self.driver.requestIRISRegion(frame.transform, self.uid)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Enabled for Walking':
            if self.getProperty('Enabled for Walking'):
                self.setProperty('Alpha', 0.2)
            else:
                self.setProperty('Alpha', 0.05)
        if propertyName == 'Visible':
            self.seedObj.setProperty('Visible', self.getProperty('Visible'))

    def onRemoveFromObjectModel(self):
        om.removeFromObjectModel(self.frameObj)
        om.removeFromObjectModel(self.seedObj)
        self.driver.regions.pop(self.uid)
        vis.PolyDataItem.onRemoveFromObjectModel(self)
