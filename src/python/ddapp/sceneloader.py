import os
import copy
import math
import numpy as np

from director import transformUtils
from director import segmentation
from director import affordanceupdater
from director import affordanceitems
from numpy import array
from director.uuidutil import newUUID
import ioUtils
from director.thirdparty import pysdf

class SceneLoader(object):
    def __init__(self):
        self.affordanceManager = segmentation.affordanceManager
    
    def loadSDF(self, filename):
        sdf = pysdf.SDF(file=filename)
        for model in sdf.world.models:
            for link in model.links:
              if hasattr(link, 'submodels'):
                if len(link.submodels)>0:
                  print model.name+' - This is an articulated object - SKIPPING!'
                  break
              for col in link.collisions:   
                t1=transformUtils.getTransformFromNumpy(model.pose)
                t2=transformUtils.getTransformFromNumpy(link.pose)
                t3=transformUtils.getTransformFromNumpy(col.pose)
                t=t1
                t.PreMultiply()
                t.Concatenate(t2)  
                t.PreMultiply()
                t.Concatenate(t3)
                p = transformUtils.poseFromTransform(t)
                name = model.name;
                if len(link.name)>0 and link.name != model.name:
                    name+='-'+link.name
                if len(col.name)>0 and len(link.collisions)>1:
                    name+='-'+col.name
                if col.geometry_type=='mesh':
                    print 'Mesh geometry is unsupported - SKIPPING!'
                if col.geometry_type=='image':
                    print 'image geometry is unsupported - SKIPPING!'
                if col.geometry_type=='height_map':
                    print 'Height map geometry is unsupported - SKIPPING!'
                if col.geometry_type=='plane':
                    print 'Plane geometry is unsupported - SKIPPING!'
                if col.geometry_type=='sphere':
                    print 'Sphere geometry is unsupported - SKIPPING!'
                if col.geometry_type=='box':
                    desc = dict(classname='BoxAffordanceItem', Name=name, uuid=newUUID(), pose=p, Color=[0.8, 0.8, 0.8], Dimensions=map(float, col.geometry_data['size'].split(' ')))
                    self.affordanceManager.newAffordanceFromDescription(desc)
                if col.geometry_type=='cylinder':
                    desc = dict(classname='CylinderAffordanceItem', Name=name, uuid=newUUID(), pose=p, Color=[0.8, 0.8, 0.8], Radius=float(col.geometry_data['radius']), Length = float(col.geometry_data['length']))
                    self.affordanceManager.newAffordanceFromDescription(desc)            

