from subprocess import Popen, PIPE, STDOUT

import subprocess
import select
import time


class timer(object):

    def __init__(self):
        self.reset()

    def now(self):
        return time.time()

    def elapsed(self):
        return self.now() - self.t0

    def reset(self):
        self.t0 = self.now()




def startMatlab():
    return subprocess.Popen(['matlab', '-nodisplay', '-nosplash'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)


def readAllSoFar(proc, retVal=''):
    while not proc.poll() and (select.select([proc.stdout],[],[],0)[0] != []):
        retVal += proc.stdout.read(1)
    return retVal


def getNextLine(p):

    retVal = ''
    while True:
        retVal += p.stdout.read(1)
        if retVal[-1] == '\n':
            return retVal[:-1]


def readForPrompt(p, timeout=-1.0):

    t = timer()
    prompt = '>> '
    output = ''

    while not p.poll():

        output = readAllSoFar(p, output)
        if output.endswith(prompt) or (timeout >= 0.0 and t.elapsed() > timeout):
            return output.split('\n')[:-1]


def send(p, inputStr):
    p.stdin.write(inputStr + '\n')


def interact():

    p = startMatlab()
    print '\n'.join(readForPrompt(p))

    while not p.poll():

        command = raw_input('>>>')

        if not command:
            continue

        send(p, command)

        if command in ['quit', 'exit']:
            p.wait()
            return

        print '\n'.join(readForPrompt(p))

