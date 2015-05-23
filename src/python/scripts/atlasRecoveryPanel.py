from ddapp import atlasdriver
from ddapp import consoleapp
from PythonQt import QtCore, QtGui
from collections import namedtuple

atlasDriver = atlasdriver.init()


w = QtGui.QWidget()
l = QtGui.QVBoxLayout(w)

Button = namedtuple('Button', ['name', 'callback', 'color']);
buttons = [
           Button('Reactive Recovery', atlasDriver.sendRecoveryTriggerOn, None)
           ]

for button in buttons:
    qb = QtGui.QPushButton(button.name)
    qb.connect('clicked()', button.callback)
    qb.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

    s = qb.styleSheet
    s += "font: 36pt;"
    if button.color:
        s += "background-color: {:s}; color: white;".format(button.color)
    qb.setStyleSheet(s)
    l.addWidget(qb)

w.setWindowTitle('Atlas Recovery Button')
w.show()
w.resize(500,600)

consoleapp.ConsoleApp.start()
