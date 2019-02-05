from director.fieldcontainer import FieldContainer


class Flags(FieldContainer):
    '''An enum like class that acts as a simple container for flags.
    This enum members are strings.

    Usage:

    colors = Flags('RED', 'GREEN', 'BLUE')
    assert colors.RED == 'RED'
    '''
    def __init__(self, *strs):
        kwargs = {s:s for s in strs}
        assert len(kwargs) == len(strs)
        FieldContainer.__init__(self, **kwargs)

    def __setattr__(self, name, value):
        raise RuntimeError('Flags object is read only')

    def __delattr__(self, name):
        raise RuntimeError('Flags object is read only')
