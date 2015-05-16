from ddapp.fieldcontainer import FieldContainer

class IkParameters(FieldContainer):

    def __init__(self, **kwargs):
        self._add_fields(
            usePointwise = None,
            useCollision = None
        )

    def setToDefaults(self):
        self.usePointwise = True
        self.useCollision = False

    def fillInWith(self, other):
        for field, value in self:
            if value is None:
                setattr(self, field, getattr(other, field))
