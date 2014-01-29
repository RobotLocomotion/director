import os
import vtkAll as vtk
from shallowCopy import shallowCopy

def readPolyData(filename):

    ext = os.path.splitext(filename)[1]

    readers = {
            '.vtp' : vtk.vtkXMLPolyDataReader,
            '.vtk' : vtk.vtkPolyDataReader,
            '.ply' : vtk.vtkPLYReader,
            '.pcd' : vtk.vtkPCDReader,
            '.obj' : vtk.vtkOBJReader,
            '.stl' : vtk.vtkSTLReader,
              }

    if ext not in readers:
        raise Exception('Unknown file extension in readPolyData: %s' % filename)

    reader = readers[ext]()
    reader.SetFileName(filename)
    reader.Update()
    return shallowCopy(reader.GetOutput())


def writePolyData(polyData, filename):

    ext = os.path.splitext(filename)[1]

    writers = {
            '.vtp' : vtk.vtkXMLPolyDataWriter,
            '.ply' : vtk.vtkPLYWriter,
            '.stl' : vtk.vtkSTLWriter,
              }

    if ext not in writers:
        raise Exception('Unknown file extension in writePolyData: %s' % filename)

    writer = writers[ext]()
    writer.SetFileName(filename)
    writer.SetInput(polyData)
    writer.Update()
