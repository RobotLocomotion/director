from ddapp import robotstate
import ddapp.vtkAll as vtk
from ddapp.transformUtils import poseFromTransform
from ddapp.fieldcontainer import FieldContainer
import numpy as np
import math



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

    @staticmethod
    def toPosition(pos):
        if isinstance(pos, vtk.vtkTransform):
            pos = np.array(pos.GetPosition())
        assert pos.shape == (3,)
        return pos

    @staticmethod
    def toQuaternion(quat):
        if isinstance(quat, vtk.vtkTransform):
            _, quat = poseFromTransform(quat)
        assert quat.shape == (4,)
        return quat

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
            bodyNameA = '',
            bodyNameB = '',
            pointInBodyA    = np.zeros(3),
            frameInBodyB = vtk.vtkTransform(),
            positionTarget = np.zeros(3),
            positionOffset = np.zeros(3),
            lowerBound     = np.zeros(3),
            upperBound     = np.zeros(3),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.bodyNameA
        assert self.bodyNameB

        varName = 'relative_position_constraint%s' % suffix
        constraintNames.append(varName)

        pointInBodyA = self.toPosition(self.pointInBodyA)
        positionTarget = self.toPosition(self.positionTarget)
        positionOffset = self.toPosition(self.positionOffset)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          bodyNameA=self.bodyNameA,
                          bodyNameB=self.bodyNameB,
                          pointInBodyA=self.toColumnVectorString(pointInBodyA),
                          frameInBodyB=self.toPositionQuaternionString(self.frameInBodyB),
                          positionTarget=self.toColumnVectorString(positionTarget + positionOffset),
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound))


        commands.append(
            'point_in_body_a = {pointInBodyA};\n'
            'frame_in_body_b = {frameInBodyB};\n'
            'lower_bounds = {positionTarget} + {lowerBound};\n'
            'upper_bounds = {positionTarget} + {upperBound};\n'
            '{varName} = RelativePositionConstraint({robotArg}, point_in_body_a, lower_bounds, '
            'upper_bounds, {bodyNameA}, {bodyNameB}, frame_in_body_b, {tspan});'
            ''.format(**formatArgs))


class PointToPointDistanceConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            bodyNameA = '',
            bodyNameB = '',
            pointInBodyA    = np.zeros(3),
            pointInBodyB    = np.zeros(3),
            lowerBound     = np.zeros(1),
            upperBound     = np.zeros(1),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.bodyNameA
        assert self.bodyNameB

        varName = 'point_to_point_distance_constraint%s' % suffix
        constraintNames.append(varName)

        pointInBodyA = self.toPosition(self.pointInBodyA)
        pointInBodyB = self.toPosition(self.pointInBodyB)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          bodyNameA=self.bodyNameA,
                          bodyNameB=self.bodyNameB,
                          pointInBodyA=self.toColumnVectorString(pointInBodyA),
                          pointInBodyB=self.toColumnVectorString(pointInBodyB),
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound))

        commands.append(
            '{varName} = Point2PointDistanceConstraint({robotArg}, {bodyNameA}, {bodyNameB}, {pointInBodyA}, {pointInBodyB}, '
            '{lowerBound}, {upperBound}, {tspan});'
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


class EulerConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName                = '',
            orientation              = np.array([0.0, 0.0, 0.0]),
            lowerBound              = np.array([0.0, 0.0, 0.0]),
            upperBound              = np.array([0.0, 0.0, 0.0]),
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'euler_constraint%s' % suffix
        constraintNames.append(varName)

        orientation = self.orientation
        if isinstance(orientation, vtk.vtkTransform):
            orientation = np.radians(np.array(orientation.GetOrientation()))

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          orientation=self.toColumnVectorString(orientation),
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound))

        commands.append(
            '{varName} = WorldEulerConstraint({robotArg}, {linkName}, '
            '{orientation} + {lowerBound}, {orientation} + {upperBound}, {tspan});'
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


class WorldGazeDirConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName                = '',
            bodyAxis                = np.array([1.0, 0.0, 0.0]),
            targetFrame             = vtk.vtkTransform,
            targetAxis              = np.array([1.0, 0.0, 0.0]),
            coneThreshold           = 0.0,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'gaze_dir_constraint%s' % suffix
        constraintNames.append(varName)

        worldAxis = np.array(self.targetAxis)
        self.targetFrame.TransformVector(worldAxis, worldAxis)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          bodyAxis=self.toColumnVectorString(self.bodyAxis),
                          worldAxis=self.toColumnVectorString(worldAxis),
                          coneThreshold=repr(self.coneThreshold))

        commands.append(
            '{varName} = WorldGazeDirConstraint({robotArg}, {linkName}, {bodyAxis}, '
            '{worldAxis}, {coneThreshold}, {tspan});'
            ''.format(**formatArgs))


class WorldGazeTargetConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            linkName                = '',
            axis                    = np.array([1.0, 0.0, 0.0]),
            worldPoint              = np.array([1.0, 0.0, 0.0]),
            bodyPoint               = np.array([1.0, 0.0, 0.0]),
            coneThreshold           = 0.0,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName

        varName = 'gaze_target_constraint%s' % suffix
        constraintNames.append(varName)

        worldPoint = self.worldPoint
        if isinstance(worldPoint, vtk.vtkTransform):
            worldPoint = worldPoint.GetPosition()

        bodyPoint = self.bodyPoint
        if isinstance(bodyPoint, vtk.vtkTransform):
            bodyPoint = bodyPoint.GetPosition()

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          axis=self.toColumnVectorString(self.axis),
                          worldPoint=self.toColumnVectorString(worldPoint),
                          bodyPoint=self.toColumnVectorString(bodyPoint),
                          coneThreshold=repr(self.coneThreshold))

        commands.append(
            '{varName} = WorldGazeTargetConstraint({robotArg}, {linkName}, {axis}, '
            '{worldPoint}, {bodyPoint}, {coneThreshld}, {tspan});'
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
            '{varName} = {varName}.setActive(true);'
            ''.format(**formatArgs))

        if self.leftFootEnabled:
            commands.append('{varName} = {varName}.addContact(l_foot, l_foot_pts);'.format(**formatArgs))
        if self.rightFootEnabled:
            commands.append('{varName} = {varName}.addContact(r_foot, r_foot_pts);'.format(**formatArgs))


class ContactConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            lowerBound  = 0.01,
            upperBound = 1e6,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):


        if not (self.leftFootEnabled or self.rightFootEnabled):
            return

        varName = 'contact_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          lowerBound=repr(self.lowerBound),
                          upperBound=repr(self.upperBound))

        commands.append(
            '{varName} = AllBodiesClosestDistanceConstraint({robotArg}, {lowerBound}, {upperBound}, {tspan});\n'
            ''.format(**formatArgs))

