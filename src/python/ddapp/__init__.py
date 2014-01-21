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

    baseDir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../externals/pod-build/src'))

    searchPaths = []
    searchPaths.append(os.path.join(baseDir, 'PointCloudLibraryPlugin-build/lib'))
    searchPaths.append(os.path.join(baseDir, 'vtk-build/Wrapping/Python'))
    searchPaths.append(os.path.join(baseDir, 'vtk-build/bin'))

    for library in libraries:
        path = findFileInPaths(library, searchPaths)
        if path and path not in sys.path:
            sys.path.insert(0, path)
            continue


_updateSysPath()
