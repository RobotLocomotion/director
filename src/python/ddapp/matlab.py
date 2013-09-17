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

    def printResult(self):
        if self.outputLines:
            print '\n'.join(self.outputLines)

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

        try:
            return [float(x) for x in result[:-1]]
        except:
            raise Exception('Failed to parse output as a float array.  Output was:\n%s' % '\n'.join(result))


    def interact(self):
        self.checkForResult()
        self.printResult()

        while self.isAlive():

            command = raw_input('>>>')

            if not command:
                continue

            if command == 'break':
                return

            self.send(command)
            self.waitForResult()
            self.printResult()
