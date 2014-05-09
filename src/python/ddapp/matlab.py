from subprocess import Popen, PIPE, STDOUT

import subprocess
import select
import os

from ddapp.simpletimer import SimpleTimer

import numpy as np



def startMatlab():
    return subprocess.Popen(['matlab', '-nodisplay', '-nosplash'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)


def _readAllSoFar(proc, retVal=''):
    while proc.poll() is None and (select.select([proc.stdout],[],[],0)[0] != []):
        retVal += proc.stdout.read(1)
    return retVal


def getAppMatlabDir():
    return os.path.join(os.path.dirname(__file__), '../../../../../src/matlab')


class MatlabCommunicator(object):

    def __init__(self, proc):
        self.proc = proc
        self.prompt = '>> '
        self.outputConsole = None
        self.echoToStdOut = True
        self.echoCommandsToStdOut = False
        self.writeCommandsToLogFile = False
        self.logFile = None
        self.logFileName = 'matlab_commands.m'
        self.clearResult()

    def checkForResult(self):
        self.accumulatedOutput = _readAllSoFar(self.proc, self.accumulatedOutput)
        if  self.accumulatedOutput.endswith(self.prompt):
            self.outputLines = self.accumulatedOutput.split('\n')[:-1]
            return self.outputLines
        else:
            return None

    def getLogFile(self):
        if self.logFile is None:
            self.logFile = open(self.logFileName, 'w')
        return self.logFile

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
        if self.echoCommandsToStdOut:
            print command
        if self.writeCommandsToLogFile:
            self.getLogFile().write(command + '\n')
            self.getLogFile().flush()

    def sendCommands(self, commands, display=True):

        commands = '\n'.join(commands).split('\n')
        for command in commands:
            self.send(command)
            self.waitForResult()
            if display:
                self.printResult()

    def waitForResultAsync(self, timeout=0.0):
        while self.waitForResult(timeout) is None:
            yield

    def sendCommandsAsync(self, commands, timeout=0.0, display=True):

        commands = '\n'.join(commands).split('\n')
        for command in commands:
            self.send(command)
            for _ in self.waitForResultAsync(timeout):
                yield
            if display:
                self.printResult()


    def getFloatArray(self, expression):

        self.send('disp(%s)' % expression)
        result = self.waitForResult()
        if len(result) and not result[-1]:
            result.pop()

        def parseRow(rowData):
            values = rowData.split()
            if len(values) == 1:
                return float(values[0])
            else:
                return [float(x) for x in values]

        try:
            return [parseRow(x) for x in result]
        except:
            raise Exception('Failed to parse output as a float array.  Output was:\n%s' % '\n'.join(result))

    def assignFloatArray(self, array, arrayName):

        if np.ndim(array) == 1:
            arrayStr = '[%s]' % ';'.join([repr(float(x)) for x in array])
        else:
            assert np.ndim(array) == 2
            arrayStr = '[%s]' % '; '.join([', '.join([repr(x) for x in row]) for row in array])

        self.send('%s = %s;' % (arrayName, arrayStr))
        self.waitForResult()

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
