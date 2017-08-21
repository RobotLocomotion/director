import vtk
import numpy as np

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
    field_names = list(self.keys())
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
    mat = np.array([[mat.GetMatrix().GetElement(r, c) for c in range(4)] for r in range(4)])
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

    def __init__(self, **kwargs):
        self._set_fields(**kwargs)

    def __iter__(self):
        for name in self._fields:
            yield name, getattr(self, name)

    def _add_fields(self, **fields):
        if not hasattr(self, '_fields'):
            object.__setattr__(self, '_fields', list(fields.keys()))
        else:
            object.__setattr__(self, '_fields', list(set(self._fields + list(fields.keys()))))
        for name, value in list(fields.items()):
            object.__setattr__(self, name, value)

    def _set_fields(self, **fields):
        if not hasattr(self, '_fields'):
            self._add_fields(**fields)
        else:
            for name, value in list(fields.items()):
                self.__setattr__(name, value)

    def __getitem__(self, name):
        return getattr(self, name)

    def __setitem__(self, name, value):
        setattr(self, name, value)

    def __len__(self):
        return len(self._fields)

    def __contains__(self, name):
        return name in self._fields

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
