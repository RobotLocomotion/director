import os
import copy
import math
import numpy as np
import xml.etree.ElementTree as etree

from director import transformUtils
from director import segmentation
from director import affordanceupdater
from director import affordanceitems
from numpy import array
from director.uuidutil import newUUID
from . import ioUtils
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
                  print(model.name+' - This is an articulated object - SKIPPING!')
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
                name = model.name
                color = col.color[0:3] if col.color is not None else [0.8,0.8,0.8]
                if len(link.name)>0 and link.name != model.name:
                    name+='-'+link.name
                if len(col.name)>0 and len(link.collisions)>1:
                    name+='-'+col.name
                if col.geometry_type=='mesh':
                    desc = dict(classname='MeshAffordanceItem', Name=name, uuid=newUUID(), pose=p, Color=color, Filename=col.geometry_data['uri'], Scale=map(float, col.geometry_data['scale'].split(' ')))
                    self.affordanceManager.newAffordanceFromDescription(desc)
                if col.geometry_type=='image':
                    print('image geometry is unsupported - SKIPPING!')
                if col.geometry_type=='height_map':
                    print('Height map geometry is unsupported - SKIPPING!')
                if col.geometry_type=='plane':
                    print('Plane geometry is unsupported - SKIPPING!')
                if col.geometry_type=='sphere':
                    print('Sphere geometry is unsupported - SKIPPING!')
                if col.geometry_type=='box':
                    desc = dict(classname='BoxAffordanceItem', Name=name, uuid=newUUID(), pose=p, Color=color, Dimensions=list(map(float, col.geometry_data['size'].split(' '))))
                    self.affordanceManager.newAffordanceFromDescription(desc)
                if col.geometry_type=='cylinder':
                    desc = dict(classname='CylinderAffordanceItem', Name=name, uuid=newUUID(), pose=p, Color=color, Radius=float(col.geometry_data['radius']), Length = float(col.geometry_data['length']))
                    self.affordanceManager.newAffordanceFromDescription(desc)
    
    def generateBoxLinkNode(self, aff):
        link = etree.Element('link', {'name':aff.getDescription()['Name']})
        link.append(etree.Element('pose'))
        pose = np.append(aff.getDescription()['pose'][0], transformUtils.quaternionToRollPitchYaw(aff.getDescription()['pose'][1]))
        link.find('pose').text = ' '.join(map(str, pose))
        dimensions = aff.getDescription()['Dimensions']
        
        if aff.getDescription()['Collision Enabled']:
            link.append(etree.Element('collision', {'name': 'collision'}))
            link.find('collision').append(etree.Element('geometry'))
            link.find('collision/geometry').append(etree.Element('box'))
            link.find('collision/geometry/box').append(etree.Element('size'))
            link.find('collision/geometry/box/size').text = ' '.join(map(str, dimensions))
        
        link.append(etree.Element('visual', {'name': 'visual'}))
        link.find('visual').append(etree.Element('geometry'))
        link.find('visual/geometry').append(etree.Element('box'))
        link.find('visual/geometry/box').append(etree.Element('size'))
        link.find('visual/geometry/box/size').text = ' '.join(map(str, dimensions))
        link.find('visual').append(etree.Element('material'))
        link.find('visual/material').append(etree.Element('diffuse'))
        link.find('visual/material/diffuse').text = '{:s} {:.1f}'.format(' '.join(map(str, aff.getDescription()['Color'])), aff.getDescription()['Alpha'])
        link.find('visual/material').append(etree.Element('ambient'))
        link.find('visual/material/ambient').text = '0 0 0 1'
        
        return link
    
    def generateCylinderLinkNode(self, aff):
        link = etree.Element('link', {'name':aff.getDescription()['Name']})
        link.append(etree.Element('pose'))
        pose = np.append(aff.getDescription()['pose'][0], transformUtils.quaternionToRollPitchYaw(aff.getDescription()['pose'][1]))
        link.find('pose').text = ' '.join(map(str, pose))
        
        if aff.getDescription()['Collision Enabled']:
            link.append(etree.Element('collision', {'name': 'collision'}))
            link.find('collision').append(etree.Element('geometry'))
            link.find('collision/geometry').append(etree.Element('cylinder'))
            link.find('collision/geometry/cylinder').append(etree.Element('radius'))
            link.find('collision/geometry/cylinder/radius').text = '{:.4f}'.format(aff.getDescription()['Radius'])
            link.find('collision/geometry/cylinder').append(etree.Element('length'))
            link.find('collision/geometry/cylinder/length').text = '{:.4f}'.format(aff.getDescription()['Length'])
        
        link.append(etree.Element('visual', {'name': 'visual'}))
        link.find('visual').append(etree.Element('geometry'))
        link.find('visual/geometry').append(etree.Element('cylinder'))
        link.find('visual/geometry/cylinder').append(etree.Element('radius'))
        link.find('visual/geometry/cylinder/radius').text = '{:.4f}'.format(aff.getDescription()['Radius'])
        link.find('visual/geometry/cylinder').append(etree.Element('length'))
        link.find('visual/geometry/cylinder/length').text = '{:.4f}'.format(aff.getDescription()['Length'])
        link.find('visual').append(etree.Element('material'))
        link.find('visual/material').append(etree.Element('diffuse'))
        link.find('visual/material/diffuse').text = '{:s} {:.1f}'.format(' '.join(map(str, aff.getDescription()['Color'])), aff.getDescription()['Alpha'])
        link.find('visual/material').append(etree.Element('ambient'))
        link.find('visual/material/ambient').text = '0 0 0 1'
        
        return link
        
                    
    def generateSDFfromAffordances(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/directorAffordances.sdf'
        sdfFile = open(filename, 'w')
        am = segmentation.affordanceManager
        affordances = am.getAffordances()
        
        root = etree.Element('sdf', {'version': '1.4'})
        tree = etree.ElementTree(root)
        world = etree.Element('world', {'name': 'directorAffordances'})
        root.append(world)
        
        for aff in affordances:
            if aff.getDescription()['classname'] in ['BoxAffordanceItem', 'CylinderAffordanceItem']:
                
                model = etree.Element('model', {'name': aff.getDescription()['Name']})
                world.append(model)
                
                model.append(etree.Element('pose'))
                model.find('pose').text = '0 0 0 0 0 0'
                
                if aff.getDescription()['classname'] == 'BoxAffordanceItem':
                    model.append(self.generateBoxLinkNode(aff))
                elif aff.getDescription()['classname'] == 'CylinderAffordanceItem':
                    model.append(self.generateCylinderLinkNode(aff))
            else:
                print('{:s} is unsupported skipping {:s} affordance!'.format(aff.getDescription()['classname'], aff.getDescription()['Name']))
        
        tree.write(sdfFile)
        sdfFile.close()
                

