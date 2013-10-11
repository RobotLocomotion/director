import vtkAll as vtk
import vtkDRCFiltersPython as drcFilters


def getTransformFromAxes(xaxis, yaxis, zaxis):

    t = vtk.vtkTransform()
    m = vtk.vtkMatrix4x4()

    axes = [xaxis, yaxis, zaxis]
    for r in xrange(3):
        for c in xrange(3):
            # transpose on assignment
            m.SetElement(r, c, axes[c][r])

    t.SetMatrix(m)
    return t


def orientationFromNormal(normal):
    '''
    Creates a frame where the Z axis points in the direction of the given normal.
    '''

    zaxis = normal
    xaxis = [0,0,0]
    yaxis = [0,0,0]

    vtk.vtkMath.Perpendiculars(zaxis, xaxis, yaxis, 0)

    return orientationFromAxes(xaxis, yaxis, zaxis)


def orientationFromAxes(xaxis, yaxis, zaxis):
    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    rpy = [0.0, 0.0, 0.0]
    drcFilters.vtkMultisenseSource.GetBotRollPitchYaw(t, rpy)
    return rpy
