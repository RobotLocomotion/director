import base64
import json
import numpy as np

'''
Implementation taken from: http://stackoverflow.com/a/24375113
'''

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        """
        if input object is a ndarray it will be converted into a dict holding dtype, shape and the data base64 encoded
        """
        if isinstance(obj, np.ndarray):
            if np.prod(obj.shape) <= 16 and obj.dtype == np.float64:
                return dict(__ndarray__=obj.tolist())
            else:
                data_b64 = str(base64.b64encode(obj.data))
                return dict(__ndarray__=data_b64,
                            dtype=str(obj.dtype),
                            shape=obj.shape)
        return json.JSONEncoder.default(self, obj)


class NumpyConvertEncoder(json.JSONEncoder):
    def default(self, obj):
        """
        if input object is a ndarray it will be converted to a Python list with ndarray.tolist
        """
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def NumpyDecoder(dct):
    """
    Decodes a previously encoded numpy ndarray
    with proper shape and dtype
    :param dct: (dict) json encoded ndarray
    :return: (ndarray) if input was an encoded ndarray
    """
    if isinstance(dct, dict) and '__ndarray__' in dct:

        if 'dtype' in dct:
            data = base64.b64decode(dct['__ndarray__'])
            return np.frombuffer(data, dct['dtype']).reshape(dct['shape'])
        else:
            return np.array(dct['__ndarray__'])
    return dct


def encode(dataObj):
    return json.dumps(dataObj, cls=NumpyEncoder)

def decode(dataStream):
    return json.loads(dataStream, object_hook=NumpyDecoder)


