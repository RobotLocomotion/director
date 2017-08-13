import PythonQt


class UndoCommand(object):

    def text(self):
        return ''

    def id(self):
        return -1

    def push(self, undoStack):
        self._command = PythonQt.dd.ddPythonUndoCommand(self.text(), self.undo, self.redo, self, self.merge, self.id())
        self._command.setParent(undoStack)
        self._command.push(undoStack)

    def undo(self):
        pass

    def redo(self):
        pass

    def merge(self, command):
        return False
