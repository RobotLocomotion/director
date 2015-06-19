from ddapp import robotstate
import ddapp.vtkAll as vtk
from ddapp.transformUtils import poseFromTransform
from ddapp.fieldcontainer import FieldContainer
import numpy as np
import math



class ConstraintBase(FieldContainer):

    __isfrozen = False
    robotArg = 'r'

    def __init__(self, **kwargs):
        self._add_fields(
          enabled  = True,
          tspan    = [-np.inf, np.inf],
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
        return self.toRowVectorString([self.tspan[0], self.tspan[-1]])

    def getJointsString(self, joints):
        return '[%s]' % '; '.join(['joints.%s' % jointName for jointName in joints])


class PostureConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            postureName      = 'q_zero',
            joints           = [],
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
                          jointInds=self.getJointsString(self.joints))

        commands.append(
            '{varName} = PostureConstraint({robotArg}, {tspan});\n'
            '{jointsVar} = {jointInds};\n'
            '{lowerLimit} = {postureName}({jointsVar}) + {lowerBound};\n'
            '{upperLimit} = {postureName}({jointsVar}) + {upperBound};\n'
            '{varName} = {varName}.setJointLimits({jointsVar}, {lowerLimit}, {upperLimit});'
            ''.format(**formatArgs))


class FixedLinkFromRobotPoseConstraint (ConstraintBase):
    def __init__(self, **kwargs):

        self._add_fields(
            poseName = '',
            linkName    = '',
            lowerBound  = np.zeros(3),
            upperBound  = np.zeros(3),
            angleToleranceInDegrees = 0.0,
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        assert self.linkName
        assert self.poseName
        positionVarName = 'position_constraint%s' % suffix
        quaternionVarName = 'quaternion_constraint%s' % suffix
        constraintNames.append(positionVarName)
        constraintNames.append(quaternionVarName)

        formatArgs = dict(positionVarName=positionVarName,
                          quaternionVarName=quaternionVarName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          poseName=self.poseName,
                          lowerBound=self.toColumnVectorString(self.lowerBound),
                          upperBound=self.toColumnVectorString(self.upperBound),
                          tolerance=repr(math.radians(self.angleToleranceInDegrees)))

        commands.append(
            'point_in_link_frame = [0; 0; 0];\n'
            'kinsol = {robotArg}.doKinematics({poseName});\n'
            'xyz_quat = {robotArg}.forwardKin(kinsol, links.{linkName}, point_in_link_frame, 2);\n'
            'lower_bounds = xyz_quat(1:3) + {lowerBound};\n'
            'upper_bounds = xyz_quat(1:3) + {upperBound};\n'
            '{positionVarName} = WorldPositionConstraint({robotArg}, links.{linkName}, '
            'point_in_link_frame, lower_bounds, upper_bounds, {tspan});'
            '{quaternionVarName} = WorldQuatConstraint({robotArg}, links.{linkName}, '
            'xyz_quat(4:7), {tolerance}, {tspan});'
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
            '{varName} = WorldPositionInFrameConstraint({robotArg}, links.{linkName}, '
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
            'upper_bounds, links.{bodyNameA}, links.{bodyNameB}, frame_in_body_b, {tspan});'
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
            '{varName} = Point2PointDistanceConstraint({robotArg}, links.{bodyNameA}, links.{bodyNameB}, {pointInBodyA}, {pointInBodyB}, '
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
            '{varName} = WorldQuatConstraint({robotArg}, links.{linkName}, '
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
            '{varName} = WorldEulerConstraint({robotArg}, links.{linkName}, '
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
            '{varName} = WorldGazeOrientConstraint({robotArg}, links.{linkName}, {axis}, '
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

        worldAxis = list(self.targetAxis)
        #print 'in:', worldAxis
        self.targetFrame.TransformVector(worldAxis, worldAxis)

        #print 'out:', worldAxis

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName,
                          bodyAxis=self.toColumnVectorString(self.bodyAxis),
                          worldAxis=self.toColumnVectorString(worldAxis),
                          coneThreshold=repr(self.coneThreshold))

        commands.append(
            '{varName} = WorldGazeDirConstraint({robotArg}, links.{linkName}, {bodyAxis}, '
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
            '{varName} = WorldGazeTargetConstraint({robotArg}, links.{linkName}, {axis}, '
            '{worldPoint}, {bodyPoint}, {coneThreshold}, {tspan});'
            ''.format(**formatArgs))


class QuasiStaticConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            leftFootEnabled  = True,
            rightFootEnabled = True,
            pelvisEnabled    = False,
            shrinkFactor     = None,
            leftFootLinkName = "",
            rightFootLinkName = "",
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):


        if not (self.leftFootEnabled or self.rightFootEnabled or self.pelvisEnabled):
            return

        varName = 'qsc_constraint%s' % suffix
        constraintNames.append(varName)

        if self.shrinkFactor is None:
            shrinkFactor = 'default_shrink_factor'
        else:
            shrinkFactor = repr(self.shrinkFactor)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          shrinkFactor=shrinkFactor,
                          leftFootLinkName = self.leftFootLinkName,
                          rightFootLinkName = self.rightFootLinkName)

        commands.append(
            '{varName} = QuasiStaticConstraint({robotArg}, {tspan}, 1);\n'
            '{varName} = {varName}.setShrinkFactor({shrinkFactor});\n'
            '{varName} = {varName}.setActive(true);'
            ''.format(**formatArgs))

        if self.leftFootEnabled:
            commands.append('{varName} = {varName}.addContact(links.{leftFootLinkName}, l_foot_pts);'.format(**formatArgs))
        if self.rightFootEnabled:
            commands.append('{varName} = {varName}.addContact(links.{rightFootLinkName}, r_foot_pts);'.format(**formatArgs))
        if self.pelvisEnabled:
            commands.append('{varName} = {varName}.addContact(links.pelvis, pelvis_pts);'.format(**formatArgs))


class WorldFixedBodyPoseConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName = '',
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        varName = 'fixed_body_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName)

        commands.append(
            '{varName} = WorldFixedBodyPoseConstraint({robotArg}, links.{linkName}, {tspan});\n'
            ''.format(**formatArgs))


class WorldFixedOrientConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            linkName = '',
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        varName = 'fixed_orientation_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          linkName=self.linkName)

        commands.append(
            '{varName} = WorldFixedOrientConstraint({robotArg}, links.{linkName}, {tspan});\n'
            ''.format(**formatArgs))


class MinDistanceConstraint(ConstraintBase):


    def __init__(self, **kwargs):

        self._add_fields(
            minDistance = 0.05
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        varName = 'contact_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          tspan=self.getTSpanString(),
                          minDistance=repr(self.minDistance))

        commands.append(
            '{varName} = MinDistanceConstraint({robotArg}, {minDistance}, {tspan});\n'
            ''.format(**formatArgs))


class ExcludeCollisionGroupConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            excludedGroupName = ''
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        formatArgs = dict(name=self.excludedGroupName,
                          tspan=self.getTSpanString()
                          )

        commands.append(
            'excluded_collision_groups = struct(\'name\',\'{name}\',\'tspan\',{tspan});\n'
            ''.format(**formatArgs))


class ActiveEndEffectorConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            endEffectorName = '',
            endEffectorPoint = np.zeros(3)
            )

        ConstraintBase.__init__(self, **kwargs)

    def _getCommands(self, commands, constraintNames, suffix):

        commands.append("end_effector_name = '%s';" % self.endEffectorName)
        commands.append("end_effector_pt = %s;" % self.toColumnVectorString(self.endEffectorPoint))


class GravityCompensationTorqueConstraint(ConstraintBase):

    def __init__(self, **kwargs):

        self._add_fields(
            joints           = [],
            torquesLowerBound = [],
            torquesUpperBound = [],
            )

        ConstraintBase.__init__(self, **kwargs)


    def _getCommands(self, commands, constraintNames, suffix):

        varName = 'gravity_compensation_torque_constraint%s' % suffix
        constraintNames.append(varName)

        formatArgs = dict(varName=varName,
                          robotArg=self.robotArg,
                          jointInds=self.getJointsString(self.joints),
                          lowerBound=self.toColumnVectorString(self.torquesLowerBound),
                          upperBound=self.toColumnVectorString(self.torquesUpperBound),
                          tspan=self.getTSpanString())

        commands.append(
            '{varName} = GravityCompensationTorqueConstraint({robotArg}, {jointInds}, {lowerBound}, {upperBound}, {tspan});\n'
            ''.format(**formatArgs))
