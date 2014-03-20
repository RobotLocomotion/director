from ddapp import robotstate
import ddapp.vtkAll as vtk
from ddapp.transformUtils import poseFromTransform
import numpy as np
import math

def _max_length(strings):
    if not strings: return 0
    return max(len(s) for s in strings)


def _fields_repr(self, indent=4):
    indent_str = ' '*indent
    field_names = list(self._fields)
    field_names.sort()
    fill_length = _max_length(field_names) + 1

    s = type(self).__name__ + "(\n"
    for field in field_names:
        value = getattr(self, field)
        value_repr = _repr(value, indent + 4)
        s += "%s%s%s= %s,\n" % (indent_str, field, ' '*(fill_length-len(field)), value_repr)
    s += "%s)" % indent_str
    return s


def _dict_repr(self, indent=4):
    indent_str = ' '*indent
    field_names = self.keys()
    for n in field_names:
        assert isinstance(n, str)
    field_names.sort()
    fill_length = _max_length(field_names) + 3

    s = "{\n"
    for field in field_names:
        value = self[field]
        value_repr = _repr(value, indent + 4)
        s += "%s'%s'%s: %s,\n" % (indent_str, field, ' '*(fill_length-len(field)-2), value_repr)
    s += "%s}" % indent_str
    return s


def _list_repr(self, indent=4):
    indent_str = ' '*indent
    return "[\n" + indent_str + (",\n" + indent_str).join(
              [_repr(item, indent + 4) for item in self]) + "\n" + indent_str + "]"


def _transform_repr(mat, indent=4):
    mat = np.array([[mat.GetMatrix().GetElement(r, c) for c in xrange(4)] for r in xrange(4)])
    indent_str = ' '*indent
    return '\n' + indent_str + repr(mat)


def _repr(self, indent=4):
    if isinstance(self, FieldContainer):
        return _fields_repr(self, indent)
    if isinstance(self, vtk.vtkTransform):
        return _transform_repr(self, indent)
    if isinstance(self, dict):
        return _dict_repr(self, indent)
    if isinstance(self, list) and len(self) and not isinstance(self[0], (int, float)):
        return _list_repr(self, indent)
    else:
        return repr(self)


class FieldContainer(object):

    __repr__ = _repr

    def _add_fields(self, **fields):
        if not hasattr(self, '_fields'):
            object.__setattr__(self, '_fields', fields.keys())
        else:
            object.__setattr__(self, '_fields', list(set(self._fields + fields.keys())))
        for name, value in fields.iteritems():
            object.__setattr__(self, name, value)

    def _set_fields(self, **fields):
        if not hasattr(self, '_fields'):
            self._add_fields(**fields)
        else:
            for name, value in fields.iteritems():
                self.__setattr__(name, value)

    def __setattr__(self, name, value):
        if hasattr(self, name):
            object.__setattr__(self, name, value)
        else:
            raise AttributeError("'%s' object has no attribute '%s'" % (type(self).__name__, name))

    def __delattr__(self, name):
        if hasattr(self, name):
            del self._fields[self._fields.index(name)]
            object.__delattr__(self, name)
        else:
            raise AttributeError("'%s' object has no attribute '%s'" % (type(self).__name__, name))


class ConstraintBase(FieldContainer):

    __isfrozen = False

    def __init__(self, **kwargs):
        self._add_fields(
          tspan    = [-np.inf, np.inf],
          robotArg = 'r',
          joints   = []
          )

        self._set_fields(**kwargs)

    def getCommands(self, commands=None, constraintNames=None, suffix=''):
        commands = [] if commands is None else commands
        constraintNames = [] if constraintNames is None else constraintNames
        self._getCommands(commands, constraintNames, suffix)
        return commands, constraintNames

    def printCommands(self):
        for command in self.getCommands()[0]:
            print command

    @staticmethod
    def toRowVectorString(vec):
        ''' Returns elements separated by "," '''
        return '[%s]' % ', '.join([repr(x) for x in vec])

    @staticmethod
    def toColumnVectorString(vec):
        ''' Returns elements separated by ";" '''
        return '[%s]' % '; '.join([repr(x) for x in vec])

    @staticmethod
    def toMatrixString(mat):
        if isinstance(mat, vtk.vtkTransform):
            mat = np.array([[mat.GetMatrix().GetElement(r, c) for c in xrange(4)] for r in xrange(4)])
        assert len(mat.shape) == 2
        return '[%s]' % '; '.join([', '.join([repr(x) for x in row]) for row in mat])

    @staticmethod
    def toPositionQuaternionString(pose):
        if isinstance(pose, vtk.vtkTransform):
            pos, quat = poseFromTransform(pose)
            pose = np.hstack((pos, quat))
        assert pose.shape == (7,)
        return ConstraintBase.toColumnVectorString(pose)

    def getTSpanString(self):
        return self.toRowVectorString(self.tspan)

    def getJointsString(self):
        return '[%s]' % '; '.join(['joints.%s' % jointName for jointName in self.joints])

    def addRightLegJoints(self):
        self.joints += robotstate.matchJoints('r_leg_.*')

    def addLeftLegJoints(self):
        self.joints += robotstate.matchJoints('l_leg_.*')

    def addLeftArmJoints(self):
        self.joints += robotstate.matchJoints('l_arm_.*')

    def addRightArmJoints(self):
        self.joints += robotstate.matchJoints('r_arm_.*')


class PostureConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            postureName      = 'q_zero',
            jointsLowerBound = [],
            jointsUpperBound = [],
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert len(self.jointsLowerBound) == len(self.jointsUpperBound) == len(self.joints)

        varName='posture_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          postureName=self.postureName,
                          jointsVar='joint_inds',
                          lowerLimit='joints_lower_limit',
                          upperLimit='joints_upper_limit',
                          lowerBound=self.toColumnVectorString(self.jointsLowerBound),
                          upperBound=self.toColumnVectorString(self.jointsUpperBound),
                          jointInds=self.getJointsString())

        commands.append(
            '{varName} = PostureConstraint({robotArg}, {tspan});\n'
            '{jointsVar} = {jointInds};\n'
            '{lowerLimit} = {postureName}({jointsVar}) + {lowerBound};\n'
            '{upperLimit} = {postureName}({jointsVar}) + {upperBound};\n'
            '{varName} = {varName}.setJointLimits({jointsVar}, {lowerLimit}, {upperLimit});'
            ''.format(**formatArgs))


class PositionConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            linkName = '',
            referenceFrame = vtk.vtkTransform(),
            pointInLink    = np.zeros(3),
            positionTarget = np.zeros(3),
            positionOffset = np.zeros(3),
            lowerBound     = np.zeros(3),
            upperBound     = np.zeros(3),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'position_constraint%s' % suffix
        constraintNames.append(varName)

        positionTarget = self.positionTarget
        if isinstance(positionTarget, vtk.vtkTransform):
            positionTarget = positionTarget.GetPosition()

        positionOffset = self.positionOffset
        if isinstance(positionOffset, vtk.vtkTransform):
            positionOffset = positionOffset.GetPosition()


        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          pointInLink=self.toColumnVectorString(self.pointInLink),
                          refFrame=self.toMatrixString(self.referenceFrame),
                          positionTarget=self.toColumnVectorString(positionTarget + positionOffset),
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound))


        commands.append(
            'point_in_link_frame = {pointInLink};\n'
            'ref_frame = {refFrame};\n'
            'lower_bounds = {positionTarget} + {lowerBound};\n'
            'upper_bounds = {positionTarget} + {upperBound};\n'
            '{varName} = WorldPositionInFrameConstraint({robotArg}, {linkName}, '
            'point_in_link_frame, ref_frame, lower_bounds, upper_bounds, {tspan});'
            ''.format(**formatArgs))


class RelativePositionConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            linkName = '',
            linkNameTarget = '',
            referenceFrame = vtk.vtkTransform(),
            pointInLink    = np.zeros(3),
            positionTarget = np.zeros(3),
            positionOffset = np.zeros(3),
            lowerBound     = np.zeros(3),
            upperBound     = np.zeros(3),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'relative_position_constraint%s' % suffix
        constraintNames.append(varName)

        positionTarget = self.positionTarget
        if isinstance(positionTarget, vtk.vtkTransform):
            positionTarget = positionTarget.GetPosition()

        positionOffset = self.positionOffset
        if isinstance(positionOffset, vtk.vtkTransform):
            positionOffset = positionOffset.GetPosition()


        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkNameA=self.linkName,
                          linkNameB=self.linkNameTarget,
                          pointInLink=self.toColumnVectorString(self.pointInLink),
                          refFrame=self.toPositionQuaternionString(self.referenceFrame),
                          positionTarget=self.toColumnVectorString(positionTarget + positionOffset),
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound))


        commands.append(
            'point_in_link_frame = {pointInLink};\n'
            'ref_frame = {refFrame};\n'
            'lower_bounds = {positionTarget} + {lowerBound};\n'
            'upper_bounds = {positionTarget} + {upperBound};\n'
            '{varName} = RelativePositionConstraint({robotArg}, point_in_link_frame, lower_bounds, upper_bounds, '
            '{linkNameA}, {linkNameB}, ref_frame, {tspan});'
            ''.format(**formatArgs))


class QuatConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName                = '',
            angleToleranceInDegrees = 0.0,
            quaternion              = np.array([1.0, 0.0, 0.0, 0.0]),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'quat_constraint%s' % suffix
        constraintNames.append(varName)

        quat = self.quaternion
        if isinstance(quat, vtk.vtkTransform):
            _, quat = poseFromTransform(quat)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          quat=self.toColumnVectorString(quat),
                          #tolerance=repr(math.sin(math.radians(self.angleToleranceInDegrees))**2))
                          tolerance=repr(math.radians(self.angleToleranceInDegrees)))

        commands.append(
            '{varName} = WorldQuatConstraint({robotArg}, {linkName}, '
            '{quat}, {tolerance}, {tspan});'
            ''.format(**formatArgs))


class WorldGazeOrientConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName                = '',
            quaternion              = np.array([1.0, 0.0, 0.0, 0.0]),
            axis                    = np.array([1.0, 0.0, 0.0]),
            coneThreshold           = 0.0,
            threshold               = 0.0,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'gaze_orient_constraint%s' % suffix
        constraintNames.append(varName)

        quat = self.quaternion
        if isinstance(quat, vtk.vtkTransform):
            _, quat = poseFromTransform(quat)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          quat=self.toColumnVectorString(quat),
                          axis=self.toColumnVectorString(self.axis),
                          coneThreshold=repr(self.coneThreshold),
                          threshold=repr(self.threshold))

        commands.append(
            '{varName} = WorldGazeOrientConstraint({robotArg}, {linkName}, {axis}, '
            '{quat}, {coneThreshold}, {threshold}, {tspan});'
            ''.format(**formatArgs))


class QuasiStaticConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            leftFootEnabled  = True,
            rightFootEnabled = True,
            shrinkFactor     = 0.5,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):


        if not (self.leftFootEnabled or self.rightFootEnabled):
            return

        varName = 'qsc_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          shrinkFactor=repr(self.shrinkFactor))

        commands.append(
            '{varName} = QuasiStaticConstraint({robotArg}, {tspan}, 1);\n'
            '{varName} = {varName}.setShrinkFactor({shrinkFactor});\n'
            '{varName}.setActive(true);'
            ''.format(**formatArgs))

        if self.leftFootEnabled:
            commands.append('{varName} = {varName}.addContact(l_foot, l_foot_pts);'.format(**formatArgs))
        if self.rightFootEnabled:
            commands.append('{varName} = {varName}.addContact(r_foot, r_foot_pts);'.format(**formatArgs))


