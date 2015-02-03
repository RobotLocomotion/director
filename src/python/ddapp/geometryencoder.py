from ddapp import vtkAll as vtk
from ddapp import vtkNumpy as vnp
from ddapp.shallowCopy import shallowCopy
import numpy as np

def encodePolyData(polyData):
    '''Given a vtkPolyData, returns a numpy int8 array that contains
    the serialization of the data.  This array can be passed to the
    decodePolyData function to construct a new vtkPolyData object from
    the serialized data.'''

    if not hasattr(vtk, 'vtkCommunicator'):
        w = vtk.vtkPolyDataWriter()
        w.WriteToOutputStringOn()
        w.SetInput(polyData)
        w.Write()
        return np.frombuffer(w.GetOutputStdString(), dtype=np.int8)

    charArray = vtk.vtkCharArray()
    vtk.vtkCommunicator.MarshalDataObject(polyData, charArray)
    numpyArray = vnp.numpy_support.vtk_to_numpy(charArray)
    assert numpyArray.dtype == np.int8
    return numpyArray

def decodePolyData(data):
    '''Given a numpy int8 array, deserializes the data to construct a new
    vtkPolyData object and returns the result.'''

    if not hasattr(vtk, 'vtkCommunicator'):
        r = vtk.vtkPolyDataReader()
        r.ReadFromInputStringOn()
        r.SetInputString(str(data.data))
        r.Update()
        return shallowCopy(r.GetOutput())

    charArray = vnp.getVtkFromNumpy(data)
    assert isinstance(charArray, vtk.vtkCharArray)
    polyData = vtk.vtkPolyData()
    vtk.vtkCommunicator.UnMarshalDataObject(charArray, polyData)
    return polyData
