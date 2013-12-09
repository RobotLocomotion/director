import vtkAll as vtk
import transformUtils
import drc as lcmdrc
import numpy as np
import time

from ddapp import lcmUtils

def createCylinderAffordance(params):


    aff = lcmdrc.affordance_t()

    length = params['length']
    radius = params['radius']
    origin = params['origin']
    axis = params['axis']

    orientation = transformUtils.orientationFromNormal(axis)

    aff.utime = 0
    aff.otdf_type = params.get('otdf_type') or 'cylinder'
    aff.friendly_name = params.get('friendly_name') or 'cylinder'
    aff.uid = params.get('uid') or 0
    aff.map_id = 0
    aff.aff_store_control = lcmdrc.affordance_t.NEW
    aff.origin_xyz = origin
    aff.origin_rpy = orientation

    aff.nparams = 3
    aff.param_names = ['length', 'mass', 'radius']
    aff.params = [length, 1.0, radius]

    aff.nstates = 0

    aff.bounding_xyz = [0,0,0]
    aff.bounding_rpy = [0,0,0]
    aff.bounding_lwh = [radius*2, radius*2, length]

    aff.modelfile = ''

    return aff


def createFrameAffordance(params):

    aff = lcmdrc.affordance_t()

    origin = params['origin']
    xaxis = params['xaxis']
    yaxis = params['yaxis']
    zaxis = params['zaxis']


    orientation = transformUtils.orientationFromAxes(xaxis, yaxis, zaxis)

    aff.utime = 0
    aff.otdf_type = params.get('otdf_type') or 'box'
    aff.friendly_name = params.get('friendly_name') or 'box'
    aff.uid = params.get('uid') or 0
    aff.map_id = 0
    aff.aff_store_control = lcmdrc.affordance_t.NEW
    aff.origin_xyz = origin
    aff.origin_rpy = orientation

    aff.nparams = 0
    aff.param_names = []
    aff.params = []

    skipParams = ['xwidth', 'ywidth', 'zwidth',
                  'xaxis', 'yaxis', 'zaxis', 'origin',
                  'friendly_name', 'otdf_type', 'uid']

    for k, v in params.iteritems():
        if k in skipParams:
            continue
        if isinstance(v, float):
            aff.param_names.append(k)
            aff.params.append(v)

    aff.nparams = len(aff.params)

    aff.nstates = 0

    aff.bounding_xyz = [0,0,0]
    aff.bounding_rpy = [0,0,0]
    aff.bounding_lwh = [0,0,0]

    aff.modelfile = ''

    return aff


def createBoxAffordance(params):


    aff = lcmdrc.affordance_t()

    origin = params['origin']

    xwidth = params['xwidth']
    ywidth = params['ywidth']
    zwidth = params['zwidth']

    xaxis = params['xaxis']
    yaxis = params['yaxis']
    zaxis = params['zaxis']

    orientation = transformUtils.orientationFromAxes(xaxis, yaxis, zaxis)

    aff.utime = 0
    aff.otdf_type = params.get('otdf_type') or 'box'
    aff.friendly_name = params.get('friendly_name') or 'box'
    aff.uid = params.get('uid') or 0
    aff.map_id = 0
    aff.aff_store_control = lcmdrc.affordance_t.NEW
    aff.origin_xyz = origin
    aff.origin_rpy = orientation

    aff.nparams = 4
    aff.param_names = ['mass', 'lX', 'lY', 'lZ']
    aff.params = [1.0, xwidth, ywidth, zwidth]

    aff.nstates = 0

    aff.bounding_xyz = [0,0,0]
    aff.bounding_rpy = [0,0,0]
    aff.bounding_lwh = [0, 0, 0]

    aff.modelfile = ''

    return aff


def createValveAffordance(params):

    aff = createCylinderAffordance(params)
    aff.otdf_type = 'steering_cyl'
    aff.friendly_name = 'valve'
    return aff


def createAffordancePlus(affordance):

    affPlus = lcmdrc.affordance_plus_t()
    affPlus.npoints = 0
    affPlus.ntriangles = 0
    affPlus.aff = affordance
    return affPlus


def publishWorldAffordance():

    params = dict(origin=[0,0,0], xwidth=0.1, ywidth=0.1, zwidth=0.1, xaxis=[1,0,0], yaxis=[0,1,0], zaxis=[0,0,1], friendly_name='world_aff')
    aff = createBoxAffordance(params)
    existingAffordance = findExistingAffordance(aff)
    if not existingAffordance:
        publishAffordance(aff)
        time.sleep(1.5) # allow time for drc viewer to receive updated affordance info
        while not existingAffordance:
            existingAffordance = findExistingAffordance(aff)

    return '%s_%d' % (existingAffordance.otdf_type, existingAffordance.uid)


def getAffordances():

    msg = lcmUtils.captureMessage('AFFORDANCE_COLLECTION', lcmdrc.affordance_collection_t)
    return msg


def findExistingAffordance(aff):
    affs = getAffordances()
    for storeAff in affs.affs:
        if storeAff.uid == aff.uid or storeAff.friendly_name == aff.friendly_name:
            return storeAff


def publishAffordance(aff):

    if aff.uid:
        #print 'publishing update affordance with uid:', aff.uid
        aff.aff_store_control = lcmdrc.affordance_t.UPDATE
        channelName =  'AFFORDANCE_TRACK'
        lcmUtils.publish(channelName, aff)

    else:
        #print 'publishing new affordance with uid:', aff.uid
        affPlus = createAffordancePlus(aff)
        channelName = 'AFFORDANCE_FIT'
        lcmUtils.publish(channelName, affPlus)


def deleteAffordance(msg):
    msg.aff_store_control = lcmdrc.affordance_t.DELETE
    affPlus = createAffordancePlus(msg)
    channelName =  'AFFORDANCE_FIT'
    lcmUtils.publish(channelName, affPlus)
