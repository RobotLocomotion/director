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
    p.addProperty('Visible', True)
    om.addToObjectModel(p)
    c = om.ObjectModelItem('test child item')
    om.addToObjectModel(c, p)

    assert om.findObjectByName('test parent item') == p
    assert om.findObjectByName('test child item') == c
    assert p.children()[0] == c
    assert c.children() == []

    objectTree.show()
    propertiesPanel.show()

    objectTree2 = QtGui.QTreeWidget()
    propertiesPanel2 = PythonQt.dd.ddPropertiesPanel()

    tree = om.ObjectModelTree()
    tree.init(objectTree2, propertiesPanel2)

    p2 = om.ObjectModelItem('test parent item 2')
    p2.addProperty('foo2', 1)
    p2.addProperty('Visible', True)
    tree.addToObjectModel(p2)
    c2 = om.ObjectModelItem('test child item 2')
    tree.addToObjectModel(c2, p2)

    assert tree.findObjectByName('test parent item 2') == p2
    assert tree.findObjectByName('test child item 2') == c2
    assert p2.children()[0] == c2
    assert c2.children() == []

    objectTree2.show()
    propertiesPanel2.show()

    startApplication(enableQuitTimer=True)


if __name__ == '__main__':
    main()
