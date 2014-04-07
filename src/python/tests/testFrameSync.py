import sys
import PythonQt
from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback
import ddapp.objectmodel as om
import ddapp.visualization as vis
from ddapp import applogic
from ddapp import vtkAll as vtk

import weakref



def main():


    om.init(QtGui.QTreeWidget(), PythonQt.dd.ddPropertiesPanel())

    # create a frame
    t1 = vtk.vtkTransform()
    f1 = vis.FrameItem('frame 1', t1, view=None)
    om.addToObjectModel(f1)

    # test object model lookup
    assert om.findObjectByName('frame 1')
    assert om.findObjectByName('frame 2') is None


    # test reference cleanup
    f1Ref = weakref.ref(f1)
    assert f1Ref() is f1

    om.removeFromObjectModel(f1)
    del f1
    assert f1Ref() is None

    # add frame again
    f1 = vis.FrameItem('frame 1', t1, view=None)
    om.addToObjectModel(f1)

    # create second frame
    t2 = vtk.vtkTransform()
    f2 = vis.FrameItem('frame 2', t2, view=None)
    om.addToObjectModel(f2)

    # test transform reference is input transform
    assert f2.transform is t2

    # test frame sync
    frameSync = vis.FrameSync()

    frameSync.addFrame(f1)
    frameSync.addFrame(f2)

    t1.Translate(10,0,0)
    t1.Modified()

    assert t2.GetPosition() == (10.0, 0.0, 0.0)


    # test frame sync cleanup
    f1Ref = weakref.ref(f1)
    assert f1Ref() is f1


    assert len(frameSync.frames) == 2

    om.removeFromObjectModel(f1)
    del f1
    assert f1Ref() is None

    assert len(frameSync.frames) == 2

    t2.Translate(10,0,0)
    t2.Modified()

    assert t2.GetPosition() == (20.0, 0.0, 0.0)

    assert len(frameSync.frames) == 1


    # add frame again
    f1 = vis.FrameItem('frame 1', t1, view=None)
    om.addToObjectModel(f1)
    frameSync.addFrame(f1)


    t1.Translate(0,5,0)
    t1.Modified()

    assert t1.GetPosition() == (10.0, 5.0, 0.0)
    assert t2.GetPosition() == (20.0, 5.0, 0.0)


    frameSync.removeFrame(f1)


    t1.Translate(0,5,0)
    t1.Modified()

    assert t1.GetPosition() == (10.0, 10.0, 0.0)
    assert t2.GetPosition() == (20.0, 5.0, 0.0)

    # this has to be wrapped in a function, otherwise the exception
    # handling holds a reference to the FrameSync object which breaks
    # the delete test at the end
    def testException(fs):
        try:
            fs.removeFrame('test')
        except KeyError:
            pass
        else:
            assert False

    testException(frameSync)


    # test cleanup
    f1Ref = weakref.ref(f1)
    om.removeFromObjectModel(f1)
    del f1
    assert f1Ref() is None

    t1Ref = weakref.ref(t1)
    del t1
    assert t1Ref() is None


    # add frame again
    t1 = vtk.vtkTransform()
    f1 = vis.FrameItem('frame 1', t1, view=None)
    om.addToObjectModel(f1)

    frameSync.addFrame(f1)

    t1.Translate(0,0,10)
    t1.Modified()
    assert t2.GetPosition() == (20.0, 5.0, 10.0)


    # verify FrameSync object can be deleted
    frameSyncRef = weakref.ref(frameSync)
    del frameSync
    assert frameSyncRef() is None

    # verify frames are no longer synced after FrameSync is deleted
    t1.Translate(0,0,10)
    t1.Modified()
    assert t2.GetPosition() == (20.0, 5.0, 10.0)


    sys.exit(0)

if __name__ == '__main__':
    main()
