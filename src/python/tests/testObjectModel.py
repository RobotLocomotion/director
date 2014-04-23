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

    p = om.ObjectModelItem('test parent item')

    p.addProperty('foo', 1)

    om.addToObjectModel(p)

    c = om.ObjectModelItem('test child item')

    om.addToObjectModel(c, p)

    assert om.findObjectByName('test parent item') == p
    assert om.findObjectByName('test child item') == c

    assert p.children()[0] == c

    assert c.children() == []

    objectTree.show()
    propertiesPanel.show()

    startApplication(enableQuitTimer=True)


if __name__ == '__main__':
    main()
