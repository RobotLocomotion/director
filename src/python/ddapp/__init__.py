import os
import sys


def findFileInPaths(filename, searchPaths):
    for path in searchPaths:
        if os.path.isfile(os.path.join(path, filename)):
            return path


def _updateSysPath():

    libraries = [
                'vtkPCLFiltersPython.so',
                'vtkCommonPython.so',
                'vtk/__init__.py'
                ]

    buildDir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

    searchPaths = []
    searchPaths.append(os.path.join(buildDir, 'PointCloudLibraryPlugin/lib'))
    searchPaths.append(os.path.join(buildDir, 'vtk/Wrapping/Python'))
    searchPaths.append(os.path.join(buildDir, 'vtk/bin'))

    for library in libraries:
        path = findFileInPaths(library, searchPaths)
        if path and path not in sys.path:
            sys.path.insert(0, path)
            continue


_updateSysPath()
