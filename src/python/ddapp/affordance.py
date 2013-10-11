import vtkAll as vtk
import transformUtils
import drc as lcmdrc
import numpy as np


def createCylinderAffordance(params):


    aff = lcmdrc.affordance_t()

    length = params['length']
    radius = params['radius']
    origin = params['origin']
    axis = params['axis']

    orientation = transformUtils.orientationFromNormal(axis)

    aff.utime = 0
    aff.otdf_type = 'cylinder'
    aff.friendly_name = 'cylinder'
    aff.uid = 0
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



def createBoxAffordance(params):


    aff = lcmdrc.affordance_t()

    origin = params['origin']

    xwidth = params['xwidth']
    ywidth = params['ywidth']
    zwidth = params['zwidth']

    xaxis = params['xaxis']
    yaxis = params['yaxis']
    zaxis = params['zaxis']


    # swap z and y before publishing
    zaxis, yaxis = yaxis, zaxis
    zwidth, ywidth = ywidth, zwidth


    if True: # this is for cinderblockstep.otdf
        xaxis, yaxis = yaxis, xaxis
        xwidth, ywidth = ywidth, xwidth

        zaxis = -zaxis
        xaxis = -xaxis
        yaxis = np.cross(zaxis, xaxis)

        origin -= zaxis*0.5*zwidth


    orientation = transformUtils.orientationFromAxes(xaxis, yaxis, zaxis)

    aff.utime = 0
    aff.otdf_type = 'box'
    aff.friendly_name = 'box'
    aff.uid = 0
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


def getAffordances():

    msg = lcmUtils.captureMessage('AFFORDANCE_COLLECTION', lcmdrc.affordance_collection_t)
    return msg


def findExistingAffordance(aff):

    affs = getAffordances()

    # lookup by friendly_name
    for storeAff in affs.affs:
        if storeAff.friendly_name == aff.friendly_name:
            return storeAff.uid

    # lookup by otdf_type
    for storeAff in affs.affs:
        if storeAff.otdf_type == aff.otdf_type:
            return storeAff.uid


def publishAffordance(aff):


    existingUid = findExistingAffordance(aff)

    if existingUid:
        aff.uid = existingUid
        aff.aff_store_control = lcmdrc.affordance_t.UPDATE
        channelName =  'AFFORDANCE_TRACK'
        lc.publish(channelName, aff.encode())

    else:
        affPlus = createAffordancePlus(aff)
        channelName = 'AFFORDANCE_FIT'
        lc.publish(channelName, affPlus.encode())
