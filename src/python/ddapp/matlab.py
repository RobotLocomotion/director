from subprocess import Popen, PIPE, STDOUT

import subprocess
import select
import os

from ddapp.simpletimer import SimpleTimer




def startMatlab():
    return subprocess.Popen(['matlab', '-nodisplay', '-nosplash'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)


def _readAllSoFar(proc, retVal=''):
    while proc.poll() is None and (select.select([proc.stdout],[],[],0)[0] != []):
        retVal += proc.stdout.read(1)
    return retVal


def getAppMatlabDir():
    return os.path.join(os.path.dirname(__file__), '../../../../src/matlab')


class MatlabCommunicator(object):

    def __init__(self, proc):
        self.proc = proc
        self.prompt = '>> '
        self.outputConsole = None
        self.echoToStdOut = True
        self.clearResult()

    def checkForResult(self):
        self.accumulatedOutput = _readAllSoFar(self.proc, self.accumulatedOutput)
        if  self.accumulatedOutput.endswith(self.prompt):
            self.outputLines = self.accumulatedOutput.split('\n')[:-1]
            return self.outputLines
        else:
            return None

    def isAlive(self):
        return (self.proc.poll() is None)

    def waitForResult(self, timeout=None):

        t = SimpleTimer()

        while self.isAlive():

            result = self.checkForResult()
            if result is not None:
                return result

            if timeout is not None and t.elapsed() > timeout:
                return None

    def _colorReplace(self, line):
        line = line.replace('[\x08', '<font color="orange">')
        line = line.replace(']\x08', '</font>')
        line = line.replace('}\x08', '<font color="red">')
        line = line.replace('{\x08', '</font>')
        return line

    def _colorStrip(self, line):
        line = line.replace('[\x08', '')
        line = line.replace(']\x08', '')
        line = line.replace('}\x08', '')
        line = line.replace('{\x08', '')
        return line

    def printResult(self):
        if not self.outputLines:
            return

        if self.outputConsole:
            self.outputConsole.append('<pre>' + 
                '<br/>'.join([self._colorReplace(line) for line in self.outputLines]) + '</pre>')

        if self.echoToStdOut or not self.outputConsole:
            print '\n'.join([self._colorStrip(line) for line in self.outputLines])

        if self.outputConsole:
            scrollBar = self.outputConsole.verticalScrollBar()
            scrollBar.setValue(scrollBar.maximum)

    def clearResult(self):
        self.accumulatedOutput = ''
        self.outputLines = []

    def getResult(self):
        return self.outputLines

    def getResultString(self):
        return self.accumulatedOutput

    def send(self, command):
        assert self.isAlive()
        self.clearResult()
        self.proc.stdin.write(command + '\n')

    def sendCommands(self, commands, display=True):

        for command in commands:
            self.send(command)
            self.waitForResult()
            if display:
                self.printResult()

    def waitForResultAsync(self, timeout=0.0):
        while self.waitForResult(timeout) is None:
            yield

    def sendCommandsAsync(self, commands, timeout=0.0, display=True):

        for command in commands:
            self.send(command)
            for _ in self.waitForResultAsync(timeout):
                yield
            if display:
                self.printResult()


    def getFloatArray(self, expression):

        self.send('disp(%s)' % expression)
        result = self.waitForResult()

        def parseRow(rowData):
            values = rowData.split()
            if len(values) == 1:
                return float(values[0])
            else:
                return [float(x) for x in values]

        try:
            return [parseRow(x) for x in result[:-1]]
        except:
            raise Exception('Failed to parse output as a float array.  Output was:\n%s' % '\n'.join(result))


    def interact(self):

        self.clearResult()
        previousEchoMode = self.echoToStdOut
        self.echoToStdOut = True

        while self.isAlive():

            command = raw_input('>>>')

            if not command:
                continue

            if command == 'break':
                break

            self.send(command)
            self.waitForResult()
            self.printResult()

        self.echoToStdOut = previousEchoMode
