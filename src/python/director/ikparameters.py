from copy import deepcopy
from director.fieldcontainer import FieldContainer

class IkParameters(FieldContainer):

    def __init__(self, **kwargs):
        self._add_fields(
            usePointwise = None,
            useCollision = None,
            majorIterationsLimit = None,
            majorOptimalityTolerance = None,
            majorFeasibilityTolerance = None,
            maxDegreesPerSecond = None,
            maxBaseMetersPerSecond = None,
            maxBaseRPYDegreesPerSecond = None,
            maxBackDegreesPerSecond = None,
            accelerationParam = None,
            accelerationFraction = None,
            maxPlanDuration = None,
            fixInitialState = None,
            numberOfAddedKnots = None,
            numberOfInterpolatedCollisionChecks = None,
            collisionMinDistance = None,
            rrtMaxEdgeLength = None,
            rrtGoalBias = None,
            rrtHand = None,
            rrtMaxNumVertices = None,
            rrtNSmoothingPasses = None,
            maxBodyTranslationSpeed = None,
            maxBodyRotationSpeed = None,
            rescaleBodyNames = None,
            rescaleBodyPts = None,
            quasiStaticShrinkFactor = None
        )

        FieldContainer.__init__(self, **kwargs)

    def setToDefaults(self):
        self.usePointwise = True
        self.useCollision = 'none'
        self.majorIterationsLimit = 500
        self.majorOptimalityTolerance = 1e-4
        self.majorFeasibilityTolerance = 1e-6
        self.maxDegreesPerSecond = 30.0
        self.maxBackDegreesPerSecond = 10.0
        self.maxBaseMetersPerSecond = 0.05
        self.maxBaseRPYDegreesPerSecond = 2
        self.maxBodyTranslationSpeed = 0.50
        self.maxBodyRotationSpeed = 10
        self.rescaleBodyNames = []
        self.rescaleBodyPts = []
        self.accelerationParam = 2
        self.accelerationFraction = 0.3
        self.maxPlanDuration = 30.0
        self.fixInitialState = True
        self.numberOfAddedKnots = 0
        self.numberOfInterpolatedCollisionChecks = 2
        self.collisionMinDistance = 0.03
        self.rrtMaxEdgeLength = 0.05
        self.rrtGoalBias = 1.0
        self.rrtHand = 'left'
        self.rrtMaxNumVertices = 5000
        self.rrtNSmoothingPasses = 10
        self.quasiStaticShrinkFactor = 0.2

    def fillInWith(self, other):
        for field, value in self:
            if value is None:
                setattr(self, field, deepcopy(getattr(other, field)))
