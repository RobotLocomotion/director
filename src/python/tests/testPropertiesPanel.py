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

    obj = om.ObjectModelItem('test')
    obj.addProperty('double', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=100, singleStep=0.5, hidden=False))
    obj.addProperty('double list', [1.0, 2.0, 3.0], attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=100, singleStep=0.5, hidden=False))

    obj.addProperty('int', 1, attributes=om.PropertyAttributes(decimals=0, minimum=100, maximum=1, singleStep=0.5, hidden=False))
    obj.addProperty('int list', [1, 2, 3], attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=100, singleStep=0.5, hidden=False))

    obj.addProperty('bool', True, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=1, singleStep=1, hidden=False))

    obj.addProperty('str', 'value', attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False))

    obj.addProperty('str list', ['value 1', 'value 2', 'value 3'], attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False))

    obj.addProperty('color', QtGui.QColor(255, 200, 0), attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False))

    panel = PythonQt.dd.ddPropertiesPanel()
    panel.setBrowserModeToWidget()
    om.addPropertiesToPanel(obj, panel)
    panel.show()

    startApplication(enableQuitTimer=True)


if __name__ == '__main__':
    main()
