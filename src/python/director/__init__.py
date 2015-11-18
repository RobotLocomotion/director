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


def updateSysPath(path):
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
        return True
    return False


_initCoverage()

# this is for mac homebrew users
updateSysPath('/usr/local/opt/vtk5/lib/python2.7/site-packages')

# this inserts the build dirs in the path before anything else
# this ensures modules in the build dirs will take precedence over
# system versions (homebrew)
for relDir in ['../../dist-packages', '../../site-packages']:
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), relDir)))
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), relDir)))
