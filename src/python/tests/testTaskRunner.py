from director import consoleapp
from director.taskrunner import TaskRunner


def testThread():
  print('on thread')
  taskRunner.callOnMain(testMain)

def testMain():
  print('on main')
  app.quit()


app = consoleapp.ConsoleApp()
app.showPythonConsole()

taskRunner = TaskRunner()
taskRunner.callOnThread(testThread)

app.start(enableAutomaticQuit=False)
