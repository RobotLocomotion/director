import os
import sys
import glob
import lcm
import time
import math
import random


messageTypes = {}
messageTypeToModule = {}

def loadMessageTypes(typesDict, typesModule, verbose=False):

    originalSize = len(typesDict)
    for name, value in typesModule.__dict__.iteritems():
        if hasattr(value, '_get_packed_fingerprint'):
            typesDict[value._get_packed_fingerprint()] = value
            messageTypeToModule[value] = typesModule

    if verbose:
        print 'loaded %d lcm message types from: %s' % (len(typesDict) - originalSize, typesModule.__name__)


def findLCMModules(searchDir):

    initFiles = glob.glob(os.path.join(searchDir, '*/__init__.py'))
    for initFile in initFiles:
        if open(initFile, 'r').readline() == '"""LCM package __init__.py file\n':
            moduleDir = os.path.dirname(initFile)
            moduleName = os.path.basename(moduleDir)
            #print 'loading module:', moduleName
            if moduleName == 'bot_procman':
                continue

            sys.path.insert(0, os.path.dirname(moduleDir))
            module = __import__(moduleName)
            sys.path.pop(0)
            loadMessageTypes(messageTypes, module)


def findLCMModulesInSysPath():
    for searchDir in sys.path:
        findLCMModules(searchDir)


def getUtime():
    return int(time.time() * 1e6)


lcmCatalog = {}


def decodeMessage(messageBytes):
    cls = getMessageClass(messageBytes)
    return cls.decode(messageBytes) if cls is not None else None

def getMessageClass(messageBytes):
    return messageTypes.get(messageBytes[:8])


def getModuleNameForMessageType(msgType):
    return messageTypeToModule[msgType].__name__


def getMessageTypeFullName(msgType):
    return '%s.%s' % (getModuleNameForMessageType(msgType), msgType.__name__)


def getMessageFullName(msg):
    if msg is None:
        return '<decode failure>'
    return getMessageTypeFullName(type(msg))


def onLCMMessage(channel, messageBytes, checkExistingChannel=False):

    if not checkExistingChannel and channel in lcmCatalog:
        return

    msg = decodeMessage(messageBytes)

    if not msg:
        if channel not in lcmCatalog:
            print 'failed to decode message on channel:', channel

    else:
        if channel not in lcmCatalog:
            print 'discovered channel/msg:', channel, getMessageFullName(msg)
        else:
            previousType = type(lcmCatalog[channel])
            if previousType != type(msg):
                print 'detected message type change: %s  %s --> %s' % (channel, getMessageTypeFullName(previousType), getMessageFullName(msg))

    lcmCatalog[channel] = msg



def printMessageFields(msg, indent=''):

    if not msg:
        return


    for slot in msg.__slots__:
        print '%s%s' % (indent, slot),
        value = getattr(msg, slot)
        if isinstance(value, float):
            print ': float'
        elif isinstance(value, int):
            print ': integer'
        elif isinstance(value, (str, unicode)):
            print ': string'
        elif isinstance(value, (list, tuple)):
            print ': array'
        elif type(value) in messageTypeToModule:
            print getMessageFullName(value)
            printMessageFields(value, indent + '  ')
        else:
            print ':', type(value)


def printLCMCatalog():

    channels = sorted(lcmCatalog.keys())

    for channel in channels:
        msg = lcmCatalog[channel]

        print '-----------------------'
        print 'channel:', channel
        print 'message:', getMessageFullName(msg)
        printMessageFields(lcmCatalog[channel], indent='  ')


def printLogFileDescription(filename):

    log = lcm.EventLog(filename, 'r')

    print 'reading %s' % filename
    print 'log file size: %.2f MB' % (log.size()/(1024.0**2))

    for event in log:
        onLCMMessage(event.channel, event.data)

    log.close()

    printLCMCatalog()


def spyLCMTraffic():

    lc = lcm.LCM()
    lc.subscribe('.+', onLCMMessage)

    try:
        while True:
          lc.handle()
    except KeyboardInterrupt:
        pass

    print
    print
    printLCMCatalog()


def main():

    findLCMModulesInSysPath()

    if len(sys.argv) > 1:
        filename = sys.argv[1]
        printLogFileDescription(filename)
    else:
        spyLCMTraffic()


if __name__ == '__main__':
    main()
