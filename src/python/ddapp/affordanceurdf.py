from ddapp import transformUtils
from ddapp import affordanceitems
from urdf_parser_py import urdf


def geometryFromAffordance(aff):

    if isinstance(aff, affordanceitems.SphereAffordanceItem):
        radius = aff.getProperty('Radius')
        return urdf.Sphere(radius=radius)

    if isinstance(aff, affordanceitems.BoxAffordanceItem):
        dimensions = aff.getProperty('Dimensions')
        return urdf.Box(size=dimensions)

    if isinstance(aff, affordanceitems.CylinderAffordanceItem):
        return urdf.Cylinder(length=aff.getProperty('Length'), radius=aff.getProperty('Radius'))

    if isinstance(aff, affordanceitems.CapsuleAffordanceItem):
        return urdf.Cylinder(length=aff.getProperty('Length'), radius=aff.getProperty('Radius'))

    if isinstance(aff, affordanceitems.CapsuleRingAffordanceItem):
        raise Exception('not supported yet')

    if isinstance(aff, affordanceitems.MeshAffordanceItem):
        filename = aff.getProperty('Filename')
        filename = affordanceitems.MeshAffordanceItem.getMeshManager().getFilesystemFilename(filename)
        return urdf.Mesh(filename=filename, scale=[1.0, 1.0, 1.0])


def stringWithAffordanceId(inputStr, aff):
    return inputStr % aff.getProperty('uuid')


def colorFromAffordance(aff):
    color = aff.getProperty('Color')
    return urdf.Color(color[0], color[1], color[2] ,1)


def materialFromAffordance(aff):
    color = colorFromAffordance(aff)
    return urdf.Material(name=stringWithAffordanceId('material_%s', aff), color=color, texture=None)


def poseFromAffordance(aff):

    t = aff.getChildFrame().transform
    position, quat = transformUtils.poseFromTransform(t)
    rpy = transformUtils.rollPitchYawFromTransform(t)
    return urdf.Pose(position, rpy)


def linkFromAffordance(aff):

    geometry = geometryFromAffordance(aff)
    material = materialFromAffordance(aff)
    pose = poseFromAffordance(aff)

    visual = urdf.Visual(geometry=geometry, material=material, origin=pose)
    collision =  urdf.Collision(geometry=geometry, origin=pose)
    return urdf.Link(name=stringWithAffordanceId('link_%s', aff), visual=visual, collision=collision)


def urdfStringFromAffordances(affordanceList):
    r = urdf.Robot(name='affordance_environment')
    for aff in affordanceList:
        r.add_link(linkFromAffordance(aff))
    return r.to_xml_string()
