from PythonQt import QtCore, QtGui

class OutputConsole(object):

    def __init__(self):

        self.textEdit = QtGui.QTextEdit()
        self.textEdit.setWindowTitle('Output console')
        self.textEdit.readOnly = True
        self.scrollBar = self.textEdit.verticalScrollBar()

    def addToAppWindow(self, app, visible=True):
        self.dockWidget = app.addWidgetToDock(self.textEdit, QtCore.Qt.BottomDockWidgetArea, visible=visible)

    def clear(self):
        self.textEdit.clear()

    def scrollToBottom(self):
        self.scrollBar.setValue(self.scrollBar.maximum)

    def scrollToTop(self):
        self.scrollBar.setValue(self.scrollBar.minimum)

    def showDock(self):
        self.dockWidget.show()

    def toggleDock(self):
        self.dockWidget.setVisible(not self.dockWidget.visible)

    def appendPlainText(self, text):
        cursor = self.textEdit.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertText(text)
        self.scrollToBottom()

    def appendHtml(self, text):
        cursor = self.textEdit.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertHtml(text)
        self.scrollToBottom()

    def appendText(self, text, color=None, bold=False):
        '''Add text to the output console.  The color arg should be a string that is a valid CSS color
           string, for example: red, or #FF0000, or rgb(255,0,0).'''
        if color is not None:
            text = '<font color="%s">%s</font>' % (color, text)
        if bold:
            text = '<b>%s</b>' % text

        self.textEdit.append(text.replace('\n', '<br/>'))

    def _pygmentsDemo(self):

        from pygments import highlight
        from pygments.lexers import PythonLexer
        from pygments.formatters import HtmlFormatter

        code = 'def foo(x="bar"): return True'

        lexer = PythonLexer()
        formatter = HtmlFormatter()
        codeHtml = highlight(code, lexer, formatter)

        doc = self.textEdit.document
        doc.defaultStyleSheet = formatter.get_style_defs()
        doc.setHtml(codeHtml)
