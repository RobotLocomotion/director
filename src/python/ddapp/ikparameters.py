from ddapp.fieldcontainer import FieldContainer

class IkParameters(FieldContainer):

    def __init__(self, **kwargs):
        self._add_fields(
            usePointwise = None,
            useCollision = None,
            majorIterationsLimit = None,
            majorOptimalityTolerance = None,
            majorFeasibilityTolerance = None,
            maxDegreesPerSecond = 30.0,
            maxBaseMetersPerSecond = 0.05,
            maxBaseRPYDegreesPerSecond = 2,
            accelerationParam = 2,
            accelerationFraction = 0.3,
            maxPlanDuration = 30.0
        )

    def setToDefaults(self):
        self.usePointwise = True
        self.useCollision = False
        self.majorIterationsLimit = 500
        self.majorOptimalityTolerance = 1e-4
        self.majorFeasibilityTolerance = 1e-6
        self.maxDegreesPerSecond = 30.0
        self.maxBaseMetersPerSecond = 0.05
        self.maxBaseRPYDegreesPerSecond = 2
        self.accelerationParam = 2;
        self.accelerationFraction = 0.3;
        self.maxPlanDuration = 30.0

    def fillInWith(self, other):
        for field, value in self:
            if value is None:
                setattr(self, field, getattr(other, field))
