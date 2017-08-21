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

    deleteCounter = 0
    def __del__(self):
        MergeableTestCommand.deleteCounter += 1

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

def test(skipClearTest=False):

    MergeableTestCommand.deleteCounter = 0
    MergeableTestCommand().push(app.undoStack)
    MergeableTestCommand().push(app.undoStack)

    # the two commands should merge and only one should be kept
    assert app.undoStack.count() == 1
    assert MergeableTestCommand.deleteCounter == 1

    app.undoStack.undo()
    app.undoStack.redo()

    assert app.undoStack.count() == 1
    assert MergeableTestCommand.deleteCounter == 1

    app.undoStack.beginMacro('my command macro')
    TestCommand().push(app.undoStack)
    TestCommand().push(app.undoStack)
    TestCommand().push(app.undoStack)
    app.undoStack.endMacro()

    # the macro should result in the stack growing by 1
    assert app.undoStack.count() == 2

    app.undoStack.undo()
    app.undoStack.redo()

    if skipClearTest:
        return

    # after clearing, the stack should be empty and
    # all commands should be deleted
    app.undoStack.clear()
    assert app.undoStack.count() == 0
    assert MergeableTestCommand.deleteCounter == 2


test(skipClearTest=True)

app.app.start()
