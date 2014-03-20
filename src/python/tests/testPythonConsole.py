from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback


def startApplication(enableQuitTimer=False):
    appInstance = QtGui.QApplication.instance()
    if enableQuitTimer:
        quitTimer = TimerCallback()
        quitTimer.callback = appInstance.quit
        quitTimer.singleShot(0.1)
    appInstance.exec_()


def main():

    _console.show()
    startApplication(enableQuitTimer=True)


if __name__ == '__main__':
    main()
