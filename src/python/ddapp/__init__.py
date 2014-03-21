import os
import sys


def _initCoverage():
    if  'COVERAGE_PROCESS_START' in os.environ:
        try:
            import coverage
            coverage.process_startup()
        except ImportError:
            pass


def getDRCBaseDir():
    return os.environ['DRC_BASE']


def findFileInPaths(filename, searchPaths):
    for path in searchPaths:
        if os.path.isfile(os.path.join(path, filename)):
            return path


def _locateExternals():
    baseDir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..'))
    for path in ['', '..']:
        externals = os.path.join(baseDir, path, 'externals/pod-build/src')
        if os.path.isdir(externals):
            return externals
    raise Exception('Failed to locate externals dir.')


def _updateSysPath():

    libraries = [
                'vtkPCLFiltersPython.so',
                'vtkCommonPython.so',
                'vtk/__init__.py'
                ]

    baseDir = _locateExternals()

    searchPaths = [
      os.path.join(baseDir, 'PointCloudLibraryPlugin-build/lib'),
      os.path.join(baseDir, 'vtk-build/Wrapping/Python'),
      os.path.join(baseDir, 'vtk-build/bin')]

    for library in libraries:
        path = findFileInPaths(library, searchPaths)
        if path and path not in sys.path:
            sys.path.insert(0, path)
            continue


_updateSysPath()
_initCoverage()
