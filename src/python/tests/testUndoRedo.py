from __future__ import print_function
import PythonQt
from director import mainwindowapp
from director.undoredo import UndoCommand


class TestCommand(UndoCommand):

    def text(self):
        return 'test command'

    def undo(self):
        print('undo', self.text())

    def redo(self):
        print('redo', self.text())


class MergeableTestCommand(UndoCommand):

    def id(self):
        return 1

    def text(self):
        return 'test command'

    def undo(self):
        print('undo', self.text())

    def redo(self):
        print('redo', self.text())

    def merge(self, other):
        print('merging with:', other.userData().text())
        self._command.setText(self.text() + ' (merged)')
        return True


app = mainwindowapp.construct()

def test():

    cmd = MergeableTestCommand()
    cmd.push(app.undoStack)

    cmd2 = MergeableTestCommand()
    cmd2.push(app.undoStack)

    app.undoStack.undo()
    app.undoStack.redo()


    app.undoStack.beginMacro('my command macro')
    TestCommand().push(app.undoStack)
    TestCommand().push(app.undoStack)
    TestCommand().push(app.undoStack)
    app.undoStack.endMacro()

test()

app.app.start()
