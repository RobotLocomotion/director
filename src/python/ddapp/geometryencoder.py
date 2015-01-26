import base64
from ddapp import vtkAll as vtk
from ddapp.shallowCopy import shallowCopy

def encodePolyData(polyData):
    w = vtk.vtkPolyDataWriter()
    w.WriteToOutputStringOn()
    w.SetInput(polyData)
    w.Write()
    return base64.b64encode(w.GetOutputStdString())

def decodePolyData(data):
    r = vtk.vtkPolyDataReader()
    r.ReadFromInputStringOn()
    r.SetInputString(base64.b64decode(data))
    r.Update()
    return shallowCopy(r.GetOutput())
