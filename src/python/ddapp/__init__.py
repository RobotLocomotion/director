import os
import sys


def _updateSysPath():

    libraryName = 'vtkPCLFiltersPython.so'

    searchPaths = []

    searchPaths.append('/source/paraview/PCLPlugin/build/lib')
    searchPaths.append(os.path.expanduser('~/source/paraview/PCLPlugin/build/lib'))

    for pathname in searchPaths:
        filename = os.path.join(pathname, libraryName)
        if os.path.isfile(filename):
            if pathname not in sys.path:
                sys.path.append(pathname)
            break


_updateSysPath()
