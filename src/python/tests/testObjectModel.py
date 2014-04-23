import PythonQt
from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback
import ddapp.objectmodel as om

def startApplication(enableQuitTimer=False):
    appInstance = QtGui.QApplication.instance()
    if enableQuitTimer:
        quitTimer = TimerCallback()
        quitTimer.callback = appInstance.quit
        quitTimer.singleShot(0.1)
    appInstance.exec_()


def main():

    objectTree = QtGui.QTreeWidget()
    propertiesPanel = PythonQt.dd.ddPropertiesPanel()

    om.init(objectTree, propertiesPanel)

    o = om.ObjectModelItem('test item')
    o.addProperty('foo', 1)

    om.addToObjectModel(o)

    objectTree.show()
    propertiesPanel.show()

    startApplication(enableQuitTimer=False)


if __name__ == '__main__':
    main()
