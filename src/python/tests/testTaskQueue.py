import PythonQt
from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import *

def startApplication(enableQuitTimer=False):
    appInstance = QtGui.QApplication.instance()
    if enableQuitTimer:
        quitTimer = TimerCallback()
        quitTimer.callback = appInstance.quit
        quitTimer.singleShot(0.1)
    appInstance.exec_()


def main():

    UserPromptTask.promptsEnabled = False

    q = AsyncTaskQueue()

    q.addTask(PrintTask('start'))
    q.addTask(DelayTask(0.1))
    q.addTask(UserPromptTask('Continue?', testingValue=False))
    #q.addTask(PauseTask())
    q.addTask(PrintTask('done'))
    q.addTask(QuitTask())

    q.start()

    globals().update(locals())

    #_console.show()
    startApplication(enableQuitTimer=False)


if __name__ == '__main__':
    main()
