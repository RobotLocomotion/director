import vtkAll as vtk
from vtkNumpy import addNumpyToVtk
from shallowCopy import shallowCopy
import numpy as np

class DebugData(object):

    def __init__(self):
        self.append = vtk.vtkAppendPolyData()


    def write(self, filename):
        writer = vtk.vtkXMLPolyDataWriter()
        writer.SetInputConnection(self.append.GetOutputPort())
        writer.SetFileName(filename)
        writer.Update()


    def addPolyData(self, polyData, color=[1,1,1], extraLabels=None):
        '''
        Add a vtkPolyData to the debug data.  A color can be provided.
        If the extraLabels argument is used, it should be a list of tuples,
        each tuple is (labelName, labelValue) where labelName is a string and
        labelValue is an int or float.  An array with labelName will be filled
        with labelValue and added to the poly data.
        '''
        polyData = shallowCopy(polyData)

        if color is not None:
            colorArray = np.empty((polyData.GetNumberOfPoints(), 3), dtype=np.uint8)
            colorArray[:,:] = np.array(color)*255
            addNumpyToVtk(polyData, colorArray, 'RGB255')

        if extraLabels is not None:
            for labelName, labelValue in extraLabels:
                extraArray = np.empty((polyData.GetNumberOfPoints(), 1), dtype=type(labelValue))
                extraArray[:] = labelValue
                addNumpyToVtk(polyData, extraArray, labelName)

        self.append.AddInput(polyData)


    def addLine(self, p1, p2, radius=0.0, color=[1,1,1]):

        line = vtk.vtkLineSource()
        line.SetPoint1(p1)
        line.SetPoint2(p2)
        line.Update()

        if radius == 0.0:
            self.addPolyData(line.GetOutput(), color)
        else:
            tube = vtk.vtkTubeFilter()
            tube.SetRadius(radius)
            tube.SetNumberOfSides(24)
            tube.CappingOn()
            tube.SetInputConnection(line.GetOutputPort())
            tube.Update()
            self.addPolyData(tube.GetOutput(), color)

    def addFrame(self, frame, scale, tubeRadius=0.0):

        origin = np.array([0.0, 0.0, 0.0])
        axes = [[scale, 0.0, 0.0], [0.0, scale, 0.0], [0.0, 0.0, scale]]
        colors = [[1,0,0], [0,1,0], [0,0,1]]
        frame.TransformPoint(origin, origin)
        for axis, color in zip(axes, colors):
            frame.TransformVector(axis, axis)
            self.addLine(origin, origin+axis, radius=tubeRadius, color=color)

    def addCircle(self, origin, normal, radius, color=[1,1,1]):

        cone = vtk.vtkConeSource()
        cone.SetRadius(radius)
        cone.SetCenter(origin)
        cone.SetDirection(normal)
        cone.SetHeight(0)
        cone.SetResolution(32)
        edges = vtk.vtkExtractEdges()
        edges.AddInputConnection(cone.GetOutputPort())
        edges.Update()
        self.addPolyData(edges.GetOutput(), color)

    def addSphere(self, center, radius=0.05, color=[1,1,1]):

        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(center)
        sphere.SetThetaResolution(24)
        sphere.SetPhiResolution(24)
        sphere.SetRadius(radius)
        sphere.Update()
        self.addPolyData(sphere.GetOutput(), color)


    def getPolyData(self):

        if self.append.GetNumberOfInputConnections(0):
            self.append.Update()
        return shallowCopy(self.append.GetOutput())
