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
    obj.addProperty('double', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=100, singleStep=0.5))
    obj.addProperty('double list', [1.0, 2.0, 3.0], attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=100, singleStep=0.5))

    obj.addProperty('int', 1, attributes=om.PropertyAttributes(minimum=0, maximum=100, singleStep=5))
    obj.addProperty('int list', [1, 2, 3], attributes=om.PropertyAttributes(minimum=0, maximum=100, singleStep=1))

    obj.addProperty('bool', True)

    obj.addProperty('str', 'value')

    obj.addProperty('str list', 0, attributes=om.PropertyAttributes(enumNames=['value 1', 'value 2', 'value 3']))

    obj.addProperty('color', [1.0, 0.5, 0.0])

    panel = PythonQt.dd.ddPropertiesPanel()
    panel.setBrowserModeToWidget()
    om.PropertyPanelHelper.addPropertiesToPanel(obj.properties, panel)
    panel.show()

    startApplication(enableQuitTimer=True)


if __name__ == '__main__':
    main()
