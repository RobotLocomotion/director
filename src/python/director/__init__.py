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

def getDRCBaseIsSet():
    return 'DRC_BASE' in os.environ

def updateSysPath(path):
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
        return True
    return False


_initCoverage()

# this is for mac homebrew users
#updateSysPath('/usr/local/opt/vtk7/lib/python3.6/site-packages')
