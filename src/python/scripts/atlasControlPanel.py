from ddapp import atlasdriver
from ddapp import consoleapp
from PythonQt import QtCore, QtGui
from collections import namedtuple

atlasDriver = atlasdriver.init()


w = QtGui.QWidget()
l = QtGui.QVBoxLayout(w)

Button = namedtuple('Button', ['name', 'callback']);
buttons = [Button('Freeze', atlasDriver.sendFreezeCommand),
           Button('Stop', atlasDriver.sendStopCommand),
           Button('Reactive Recovery', atlasDriver.sendRecoveryTriggerOn)]

for button in buttons:
    qb = QtGui.QPushButton(button.name)
    qb.connect('clicked()', button.callback)
    qb.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    l.addWidget(qb)

w.setWindowTitle('Atlas Control Panel')
w.show()
w.resize(500,600)

consoleapp.ConsoleApp.start()
