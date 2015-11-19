import PythonQt
from PythonQt import QtCore

view = PythonQt.dd.ddQVTKWidgetView()
view.showMaximized()

QtCore.QCoreApplication.instance().exec_()
