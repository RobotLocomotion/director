import os
import vtkAll as vtk

def readPolyData(filename):

    ext = os.path.splitext(filename)[1]

    if ext == '.vtp':
        reader = vtk.vtkXMLPolyDataReader()
        reader.SetFileName(filename)
        reader.Update()
        return reader.GetOutput()

    raise Exception('Unknown file extension in readPolyData: %s' % filename)


def writePolyData(polyData, filename):

    ext = os.path.splitext(filename)[1]

    if ext == '.vtp':
        writer = vtk.vtkXMLPolyDataWriter()
        writer.SetFileName(filename)
        writer.SetInputData(polyData)
        writer.Update()
    else:
        raise Exception('Unknown file extension in writePolyData: %s' % filename)
