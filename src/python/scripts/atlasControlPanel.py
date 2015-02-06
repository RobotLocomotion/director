from ddapp import atlasdriver
from ddapp import consoleapp
from PythonQt import QtCore, QtGui

atlasDriver = atlasdriver.init()


w = QtGui.QWidget()
l = QtGui.QVBoxLayout(w)

fb = QtGui.QPushButton('Freeze')
sb = QtGui.QPushButton('Stop')

fb.connect('clicked()', atlasDriver.sendFreezeCommand)
sb.connect('clicked()', atlasDriver.sendStopCommand)

fb.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
sb.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

l.addWidget(fb)
l.addWidget(sb)

w.setWindowTitle('Atlas Control Panel')
w.show()
w.resize(500,500)

consoleapp.ConsoleApp.start()
