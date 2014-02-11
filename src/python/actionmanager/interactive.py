from PythonQt import QtCore, QtGui
import signal
import time



def interruptHandler(signal, frame):
    showConsole()
    startApplication()

signal.signal(signal.SIGINT, interruptHandler)


def showConsole():
    _pythonManager.showConsole()

def startApplication():
    QtGui.QApplication.instance().exec_()


def main():
    for i in xrange(100):
        print i
        time.sleep(1)



if __name__ == '__main__':
    main()
