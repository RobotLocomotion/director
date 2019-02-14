import director.vtkAll as vtk
from director import vtkNumpy as vnp
from director.shallowCopy import shallowCopy
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
            vnp.addNumpyToVtk(polyData, colorArray, 'RGB255')

        if extraLabels is not None:
            for labelName, labelValue in extraLabels:
                extraArray = np.empty((polyData.GetNumberOfPoints(), 1), dtype=type(labelValue))
                extraArray[:] = labelValue
                vnp.addNumpyToVtk(polyData, extraArray, labelName)

        self.append.AddInputData(polyData)

    def addLine(self, p1, p2, radius=0.0, color=[1,1,1]):
        line = vtk.vtkLineSource()
        line.SetPoint1(p1)
        line.SetPoint2(p2)
        line.Update()
        polyData = line.GetOutput()
        if radius > 0.0:
            polyData = applyTubeFilter(polyData, radius)
        self.addPolyData(polyData, color)

    def addPolyLine(self, points, isClosed=False, radius=0.0, color=[1,1,1]):
        pts = vnp.getVtkPointsFromNumpy(np.array(points, dtype=np.float64))
        polyLine = vtk.vtkPolyLineSource()
        polyLine.SetPoints(pts)
        polyLine.SetClosed(isClosed)
        polyLine.Update()
        polyData = polyLine.GetOutput()
        if radius > 0:
            polyData = applyTubeFilter(polyData, radius)
        self.addPolyData(polyData, color)

    def addFrame(self, frame, scale, tubeRadius=0.0):
        axes = vtk.vtkAxes()
        axes.ComputeNormalsOff()
        axes.SetScaleFactor(scale)
        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(frame)
        transformFilter.SetInputConnection(axes.GetOutputPort())
        transformFilter.Update()
        polyData = transformFilter.GetOutput()
        colors = np.array(
            [[255, 0, 0], [255, 0, 0],
            [0, 255, 0], [0, 255, 0],
            [0, 0, 255], [0, 0, 255]], dtype=np.uint8)
        vnp.addNumpyToVtk(polyData, colors, 'RGB255')
        if tubeRadius:
            polyData = applyTubeFilter(polyData, tubeRadius)
        self.addPolyData(polyData, color=None)

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

    def addArrow(self, start, end, headRadius=0.05, headLength=None, tubeRadius=0.01, color=[1,1,1], startHead=False, endHead=True):
        if headLength is None:
            headLength = headRadius
        normal = np.array(end) - np.array(start)
        normal = normal / np.linalg.norm(normal)
        if startHead:
            start = np.array(start) + 0.5 * headLength * normal
        if endHead:
            end = np.array(end) - 0.5 * headLength * normal
        self.addLine(start, end, radius=tubeRadius, color=color)
        if startHead:
            self.addCone(origin=start, normal=-normal, radius=headRadius,
                         height=headLength, color=color, fill=True)
        if endHead:
            self.addCone(origin=end, normal=normal, radius=headRadius,
                         height=headLength, color=color, fill=True)

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
        cube.QuadsOn()
        cube.Update()
        self.addPolyData(cube.GetOutput(), color)

    def addPlane(self, origin, normal, width, height, resolution=1, color=[1,1,1]):
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-width/2.0, -height/2.0, 0.0)
        plane.SetPoint1(width/2.0, -height/2.0, 0.0)
        plane.SetPoint2(-width/2.0, height/2.0, 0.0)
        plane.SetCenter(origin)
        plane.SetNormal(normal)
        plane.SetResolution(resolution, resolution)
        plane.Update()
        self.addPolyData(plane.GetOutput(), color)

    def addCylinder(self, center, axis, length, radius, color=[1,1,1]):
        axis = np.asarray(axis) / np.linalg.norm(axis)
        center = np.array(center)
        self.addLine(center - 0.5*length*axis, center + 0.5*length*axis, radius=radius, color=color)

    def addCapsule(self, center, axis, length, radius, color=[1,1,1]):
        axis = np.asarray(axis) / np.linalg.norm(axis)
        center = np.array(center)
        self.addCylinder(center=center, axis=axis, radius=radius, length=length, color=color)
        self.addSphere(center=center-0.5*length*axis, radius=radius, color=color)
        self.addSphere(center=center+0.5*length*axis, radius=radius, color=color)

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

    def addEllipsoid(self, center, radii, resolution=24, color=[1,1,1]):
        """
        Add an ellipsoid centered at [center] with x, y, and z principal axis radii given by
        radii = [x_scale, y_scale, z_scale]
        """
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter([0,0,0])
        sphere.SetThetaResolution(resolution)
        sphere.SetPhiResolution(resolution)
        sphere.SetRadius(1.0)
        sphere.Update()

        transform = vtk.vtkTransform()
        transform.Translate(center)
        transform.Scale(radii)

        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        transformFilter.SetInputConnection(sphere.GetOutputPort())
        transformFilter.Update()
        self.addPolyData(transformFilter.GetOutput(), color)

    def addPolygon(self, points, color=[1,1,1]):
        points = vnp.getVtkPointsFromNumpy(np.array(points, dtype=np.float64))
        polygon = vtk.vtkPolygon()
        polygon.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())

        for i in range(points.GetNumberOfPoints()):
            polygon.GetPointIds().SetId(i, i)

        polyData = vtk.vtkPolyData()
        polyData.SetPoints(points)
        polyData.Allocate(1, 1)
        polyData.InsertNextCell(polygon.GetCellType(), polygon.GetPointIds())
        self.addPolyData(polyData, color)

    def getPolyData(self):
        if self.append.GetNumberOfInputConnections(0):
            self.append.Update()
        return shallowCopy(self.append.GetOutput())


def applyTubeFilter(polyData, radius, numberOfSides=24):
    tube = vtk.vtkTubeFilter()
    tube.SetRadius(radius)
    tube.SetNumberOfSides(numberOfSides)
    tube.CappingOn()
    tube.SetInputData(polyData)
    tube.Update()
    return tube.GetOutput()
