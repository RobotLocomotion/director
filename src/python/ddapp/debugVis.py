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
        self.addCone(origin, normal, radius, height=0, color=color, fill=False)

    def addCone(self, origin, normal, radius, height, color=[1,1,1], fill=True):
        cone = vtk.vtkConeSource()
        cone.SetRadius(radius)
        cone.SetCenter(origin)
        cone.SetDirection(normal)
        cone.SetHeight(height)
        cone.SetResolution(32)
        if fill:
            cone.Update()
            self.addPolyData(cone.GetOutput(), color)
        else:
            edges = vtk.vtkExtractEdges()
            edges.AddInputConnection(cone.GetOutputPort())
            edges.Update()
            self.addPolyData(edges.GetOutput(), color)

    def addArrow(self, start, end, headRadius=0.05, tubeRadius=0.01, color=[1,1,1], startHead=False, endHead=True):
        normal = np.array(end) - np.array(start)
        normal = normal / np.linalg.norm(normal)
        if startHead:
            start = np.array(start) + headRadius * normal
        if endHead:
            end = np.array(end) - headRadius * normal
        self.addLine(start, end, radius=tubeRadius, color=color)
        if startHead:
            self.addCone(origin=start, normal=-normal, radius=headRadius,
                         height=headRadius, color=color, fill=True)
        if endHead:
            self.addCone(origin=end, normal=normal, radius=headRadius,
                         height=headRadius, color=color, fill=True)

    def addSphere(self, center, radius=0.05, color=[1,1,1], resolution=24):

        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(center)
        sphere.SetThetaResolution(resolution)
        sphere.SetPhiResolution(resolution)
        sphere.SetRadius(radius)
        sphere.Update()
        self.addPolyData(sphere.GetOutput(), color)

    def addCube(self, dimensions, center, color=[1,1,1], subdivisions=0):

        bmin = np.array(center) - np.array(dimensions)/2.0
        bmax = np.array(center) + np.array(dimensions)/2.0
        cube = vtk.vtkTessellatedBoxSource()
        cube.SetBounds(bmin[0], bmax[0], bmin[1], bmax[1], bmin[2], bmax[2])
        cube.SetLevel(subdivisions)
        cube.Update()
        self.addPolyData(cube.GetOutput(), color)

    def addCylinder(self, center, axis, length, radius, color=[1,1,1]):
        axis = np.array(axis)
        axis /= np.linalg.norm(axis)
        center = np.array(center)
        self.addLine(center - 0.5*length*axis, center + 0.5*length*axis, radius=radius, color=color)

    def addCapsule(self, center, axis, length, radius, color=[1,1,1]):
        axis /= np.linalg.norm(axis)
        center = np.array(center)
        self.addCylinder(center=center, axis=axis, radius=radius, length=length)
        self.addSphere(center=center-0.5*length*axis, radius=radius)
        self.addSphere(center=center+0.5*length*axis, radius=radius)

    def addTorus(self, radius, thickness, resolution=30):

        q = vtk.vtkSuperquadricSource()
        q.SetToroidal(1)
        q.SetSize(radius)
        q.SetThetaResolution(resolution)
        # thickness doesnt seem to match to Eucliean units. 0 is none. 1 is full. .1 is a good valve
        q.SetThickness(thickness)
        q.Update()

        # rotate Torus so that the hole axis (internally y), is set to be z, which we use for valves
        transform = vtk.vtkTransform()
        transform.RotateWXYZ(90,1,0,0)
        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        transformFilter.SetInputConnection(q.GetOutputPort())
        transformFilter.Update()
        self.addPolyData(transformFilter.GetOutput())

    def getPolyData(self):

        if self.append.GetNumberOfInputConnections(0):
            self.append.Update()
        return shallowCopy(self.append.GetOutput())
