import os
import copy
import math
import numpy as np

from ddapp import transformUtils
from ddapp import segmentation
from ddapp import affordanceupdater
from ddapp import affordanceitems
from numpy import array
from ddapp.uuidutil import newUUID
import ioUtils
from ddapp.thirdparty import pysdf

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
                    
    def generateSDFfromAffordances(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/directorAffordances.sdf'
        sdfFile = open(filename, 'w')
        am = segmentation.affordanceManager
        affordances = am.getAffordances()
        xmlStr = '<sdf version=\'1.4\'>\n'\
                 '\t<world name=\'directorAffordances\'>\n'
        for aff in affordances:
            if aff.getDescription()['classname'] in ['BoxAffordanceItem', 'CylinderAffordanceItem']:
                name = aff.getDescription()['Name']
                xmlStr += '\t\t<model name=\'{0:s}\'>\n'\
                          '\t\t\t<pose>0 0 0 0 0 0</pose>\n'\
                          '\t\t\t<link name=\'{0:s}\'>\n'.format(name)
                pose = np.append(aff.getDescription()['pose'][0], transformUtils.quaternionToRollPitchYaw(aff.getDescription()['pose'][1]))
                xmlStr += '\t\t\t\t<pose>{:s}</pose>\n'.format(' '.join(map(str, pose)))
                collisions = aff.getDescription()['Collision Enabled']
                color = aff.getDescription()['Color']
                alpha = aff.getDescription()['Alpha']
                if aff.getDescription()['classname'] == 'BoxAffordanceItem':
                    geometry = 'box'
                    dimensions = aff.getDescription()['Dimensions']
                    if collisions:
                        xmlStr += '\t\t\t\t<collision name=\'collision\'>\n'\
                                  '\t\t\t\t\t<geometry>\n'\
                                  '\t\t\t\t\t\t<{0:s}>\n'\
                                  '\t\t\t\t\t\t\t<size>{1:s}</size>\n'\
                                  '\t\t\t\t\t\t</{0:s}>\n'\
                                  '\t\t\t\t\t</geometry>\n'\
                                  '\t\t\t\t</collision>\n'.format(geometry, ' '.join(map(str, dimensions)))
                    xmlStr += '\t\t\t\t<visual name=\'visual\'>\n'\
                              '\t\t\t\t\t<geometry>\n'\
                              '\t\t\t\t\t\t<{0:s}>\n'\
                              '\t\t\t\t\t\t\t<size>{1:s}</size>\n'\
                              '\t\t\t\t\t\t</{0:s}>\n'\
                              '\t\t\t\t\t</geometry>\n'\
                              '\t\t\t\t\t<material>\n'\
                              '\t\t\t\t\t\t<diffuse>{2:s} {3:.1f}</diffuse>\n'\
                              '\t\t\t\t\t\t<ambient>0 0 0 1</ambient>\n'\
                              '\t\t\t\t\t</material>\n'\
                              '\t\t\t\t</visual>\n'.format(geometry, ' '.join(map(str, dimensions)), ' '.join(map(str, color)), alpha)                    
                elif aff.getDescription()['classname'] == 'CylinderAffordanceItem':
                    geometry = 'cylinder'
                    radius = aff.getDescription()['Radius']
                    length = aff.getDescription()['Length']
                    if collisions:
                        xmlStr += '\t\t\t\t<collision name=\'collision\'>\n'\
                                  '\t\t\t\t\t<geometry>\n'\
                                  '\t\t\t\t\t\t<{0:s}>\n'\
                                  '\t\t\t\t\t\t\t<radius>{1:f}</radius>\n'\
                                  '\t\t\t\t\t\t\t<length>{2:f}</length>\n'\
                                  '\t\t\t\t\t\t</{0:s}>\n'\
                                  '\t\t\t\t\t</geometry>\n'\
                                  '\t\t\t\t</collision>\n'.format(geometry, radius, length)
                    xmlStr += '\t\t\t\t<visual name=\'visual\'>\n'\
                              '\t\t\t\t\t<geometry>\n'\
                              '\t\t\t\t\t\t<{0:s}>\n'\
                              '\t\t\t\t\t\t\t<radius>{1:f}</radius>\n'\
                              '\t\t\t\t\t\t\t<length>{2:f}</length>\n'\
                              '\t\t\t\t\t\t</{0:s}>\n'\
                              '\t\t\t\t\t</geometry>\n'\
                              '\t\t\t\t\t<material>\n'\
                              '\t\t\t\t\t\t<diffuse>{3:s} {4:.1f}</diffuse>\n'\
                              '\t\t\t\t\t\t<ambient>0 0 0 1</ambient>\n'\
                              '\t\t\t\t\t</material>\n'\
                              '\t\t\t\t</visual>\n'.format(geometry, radius, length, ' '.join(map(str, color)), alpha) 
                
                xmlStr += '\t\t\t</link>\n'
                xmlStr += '\t\t</model>\n'                
            else:
                print '{:s} is unsupported skipping {:s} affordance!'.format(aff.getDescription()['classname'], name)
        xmlStr += '\t</world>\n'\
                 '</sdf>'
        sdfFile.write(xmlStr.expandtabs(2))
        sdfFile.close()
                

