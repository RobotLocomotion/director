from subprocess import Popen, PIPE, STDOUT

import subprocess
import select
import socket
import os

import ddapp
from ddapp.simpletimer import SimpleTimer

import numpy as np



def startMatlab():
    return subprocess.Popen(['matlab', '-nodisplay', '-nosplash'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)


def _readAllSoFar(proc, retVal=''):
    while proc.poll() is None and (select.select([proc.stdout],[],[],0)[0] != []):
        retVal += proc.stdout.read(1)
    return retVal




DEFALUT_MATLAB_SERVER_PORT=41576

class MatlabServer(object):

    def __init__(self, port=DEFALUT_MATLAB_SERVER_PORT):
        self.port = port
        self.proc = None
        self.sock = None

    def start(self):
        self.proc = startMatlab()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        self.sock.listen(1)

        while True:
            print 'waiting for client...'
            conn, addr = self.sock.accept()
            print 'client connected.'
            self.serve(conn)

    def serve(self, sock):

        sock.settimeout(0.001)
        while True:
            data = _readAllSoFar(self.proc, '')
            if data:
                sock.send(data)
            try:
                inData = sock.recv(1024)
                if inData:
                    self.proc.stdin.write(inData)
                else:
                    sock.close()
                    return
            except socket.timeout as e:
                pass
            except socket.error as e:
                print 'socket error:', e
                sock.close()
                return


class MatlabSocketClient(object):

    def __init__(self, host='127.0.0.1', port=DEFALUT_MATLAB_SERVER_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.connect()

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.sock.connect((self.host, self.port))
        except socket.error:
            self.sock = None
        else:
            self.sock.settimeout(0.001)


    def send(self, data):
        self.sock.send(data)

    def receive(self):
        if not self.isAlive():
            return ''

        try:
            inData = self.sock.recv(1024)
            if inData:
                return inData
            else:
                self.sock.close()
                self.sock = None
                return ''
        except socket.timeout as e:
            return ''

    def isAlive(self):
        return (self.sock is not None)


class MatlabPipeClient(object):

    def __init__(self):
        self.proc = startMatlab()

    def send(self, data):
        self.proc.stdin.write(data)

    def receive(self):
        return _readAllSoFar(self.proc, '')

    def isAlive(self):
        return (self.proc.poll() is None)


class MatlabCommunicator(object):

    def __init__(self, matlabClient):
        self.client = matlabClient
        self.prompt = '>> '
        self.outputConsole = None
        self.echoToStdOut = True
        self.echoCommandsToStdOut = False
        self.writeCommandsToLogFile = False
        self.logFile = None
        self.logFileName = '/tmp/matlab_commands.m'
        self.clearResult()

    def checkForResult(self):
        self.accumulatedOutput = self.accumulatedOutput + self.client.receive()
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
        return self.client.isAlive()

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
        self.client.send(command + '\n')
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

        def joinFloats(values, sep):
            maxLength = 180.0
            pieces = np.array_split(values, np.ceil(len(values)/maxLength))
            pieces = [sep.join([repr(float(x)) for x in piece]) for piece in pieces]
            return str(sep + '...\n').join(pieces)

        if np.ndim(array) == 1:
            arrayStr = '[%s]' % joinFloats(array, ';')
        else:
            assert np.ndim(array) == 2
            arrayStr = '[%s]' % ';...\n'.join([joinFloats(row, ',') for row in array])

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


if __name__ == '__main__':
    server = MatlabServer()
    server.start()
