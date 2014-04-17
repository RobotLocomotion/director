import os
import sys
import time
import lcm
import functools
import threading
import glob
import json
import re
import numpy as np

from ddapp import lcmspy as spy


logCatalog = {}
utimeMap = {}

class FieldData(object):

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        keys = self.__dict__.keys()
        return 'FieldData(%s)' % ', '.join(['%s=%r' % (k,v) for k, v in self.__dict__.iteritems()])


def getRecentUtimes(utimeMap, seconds=60):

    utimes = np.array(utimeMap.keys())
    if not len(utimes):
        return None

    utimes.sort()
    endTime = utimes[-1]
    startTime = max(0, endTime - seconds*1e6)
    startIndex = utimes.searchsorted(startTime)
    utimes = utimes[startIndex:]

    #print utimes

    if not len(utimes):
        return None

    return utimes


class LogLookup(object):

    def __init__(self):
        self.utimeMap = None
        self.logs = {}

    def setUtimeMap(self, utimeMap):
        self.utimeMap = utimeMap

    def getImage(self, utime):
        filename, filepos = self.utimeMap[utime]

        log = self.logs.get(filename)
        if log is None:
            log = lcm.EventLog(filename, 'r')
            self.logs[filename] = log

        log.seek(filepos)
        event = log.read_next_event()
        msg = spy.decodeMessage(event.data)

        if hasattr(msg, 'images'):
            msg = msg.images[0]
        return msg, filename

    def closeLogs(self):
        for log in self.logs.values():
            log.close()
        self.logs = {}


class PlayThread(object):

    def __init__(self, utimes, logLookup, lc, speed):
        self.fps = 60
        self.shouldStop = False
        self.utimes = utimes
        self.logLookup = logLookup
        self.speed = speed
        self.lc = lc

    def start(self):
        self.shouldStop = False
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.shouldStop = True

    def mainLoop(self):
        startTime = time.time()

        while not self.shouldStop:

            elapsedUtime = int(1e6 * (time.time() - startTime)*self.speed)

            utimeIndex = self.utimes.searchsorted(self.utimes[0] + elapsedUtime)

            if utimeIndex == len(self.utimes):
                break

            utimeRequest = self.utimes[utimeIndex]
            image, filename = self.logLookup.getImage(utimeRequest)

            print 'elapsed:  %.2f    index: %d    play jitter:  %.3f' % (elapsedUtime*1e-6, utimeIndex, (utimeRequest - (self.utimes[0] + elapsedUtime)  )*1e-6)

            self.lc.publish('VIDEO_PLAYBACK_IMAGE', image.encode())

            time.sleep(1.0 / self.fps)


class ServerThread(object):

    def __init__(self):

        self.utimes = None
        self.playbackThread = None
        self.logLookup = LogLookup()
        self.lc = lcm.LCM()

    def start(self):
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.thread.start()

    def getUtimeIndex(self, data):

        assert 0.0 <= data.value <= 1.0
        return (len(self.utimes)-1)*data.value


    def onFrameRequest(self, data):

        self.stopPlaybackThread()

        if self.utimes is None:

            self.logLookup.setUtimeMap(dict(utimeMap))
            self.utimes = getRecentUtimes(self.logLookup.utimeMap)

            if self.utimes is None:
                print 'no utimes cataloged'
                return

            print 'starting review with utimes %d %d' % (self.utimes[0], self.utimes[1])


        utimeIndex = self.getUtimeIndex(data)
        utimeRequest = self.utimes[utimeIndex]
        image, filename = self.logLookup.getImage(utimeRequest)

        print 'index: %d    timeDelta:  %.3f    file: %s' % (utimeIndex, (self.utimes[-1] - self.utimes[utimeIndex])*1e-6, os.path.basename(filename))

        self.lc.publish('VIDEO_PLAYBACK_IMAGE', image.encode())


    def onResume(self, data):
        self.stopPlaybackThread()
        self.utimes = None
        self.logLookup.closeLogs()
        return


    def onPlay(self, data):
        if self.utimes is None:
            print 'cannot play.  no utimes available'
            return

        self.stopPlaybackThread()

        startIndex = self.getUtimeIndex(data)
        playbackUtimes = self.utimes[startIndex:]
        self.playbackThread = PlayThread(playbackUtimes, self.logLookup, self.lc, speed=data.speed)
        self.playbackThread.start()


    def stopPlaybackThread(self):
        if self.playbackThread:
            self.playbackThread.stop()
        self.playbackThread = None


    def unwrapCommand(self, msgBytes):
        msg = spy.decodeMessage(msgBytes)
        argDict = json.loads(msg.command)
        return FieldData(**argDict)


    def onControlMessage(self, channel, msgBytes):

        data = self.unwrapCommand(msgBytes)
        print data

        if data.command == 'request_frame':
            self.onFrameRequest(data)
        elif data.command == 'resume':
            self.onResume(data)
        elif data.command == 'play':
            self.onPlay(data)


    def mainLoop(self):

        self.lc.subscribe('VIDEO_PLAYBACK_CONTROL', self.onControlMessage)

        print 'starting lcm handle loop'
        while True:
          self.lc.handle()


def cropUtimeMap():

    global utimeMap

    if not len(utimeMap):
        return

    utimes = getRecentUtimes(utimeMap)
    cropTime = utimes[0]
    newMap = {}

    for utime, value in utimeMap.iteritems():
        if utime >= cropTime:
            newMap[utime] = value

    #print 'cropped utime map.  %d --> %d elements.' % (len(utimeMap), len(newMap))
    utimeMap = newMap


def updateLogInfo(filename, catalog):

    fieldData = catalog.get(filename)

    if not fieldData:
        print 'discovered new file:', filename
        fieldData = FieldData(filename=filename, fileSize=0, lastFilePos=0, channelTypes={}, imageStreams={})
        logCatalog[filename] = fieldData
        cropUtimeMap()


    log = lcm.EventLog(filename, 'r')
    fileSize = log.size()
    if fileSize == fieldData.fileSize:
        #print '  file size not changed, done.'
        return

    fieldData.fileSize = fileSize

    if fieldData.lastFilePos > 0:
        #print '  lastFilePos:', fieldData.lastFilePos, 'seeking and skipping next event'
        log.seek(fieldData.lastFilePos)
        event = log.read_next_event()


    #print 'reading file:', filename

    channelTypes = fieldData.channelTypes
    imageStreams = fieldData.imageStreams

    newPacketsCount = 0

    while True:

        filepos = log.tell()
        event = log.read_next_event()
        if not event:
            break

        timestamp = event.timestamp
        channel = event.channel

        if channel not in channelTypes:
            messageClass = spy.getMessageClass(event.data)
            channelTypes[channel] = messageClass

            if messageClass and messageClass.__name__ in ('image_t', 'images_t'):
                #print '  discovered image channel:', channel, messageClass.__name__
                imageStreams.setdefault(channel, [])

        positions = imageStreams.get(channel)
        if positions is not None:
            #positions.append((timestamp, filepos))
            utimeMap[timestamp] = (fieldData.filename, filepos)

        fieldData.lastFilePos = filepos
        newPacketsCount += 1

    log.close()
    #print '  processed new packets:', newPacketsCount
    #for channelName, streamData in imageStreams.iteritems():
    #    print '%s: %d frames' % (channelName, len(streamData))



def getExistingLogFiles(dirName):

    # sort log filenames by splitting string into
    # list of strings and numbers.
    # Example filename: lcmlog-2014-04-16.25
    # sort implementation taken from: http://stackoverflow.com/a/5967539

    def atoi(text):
        return int(text) if text.isdigit() else text

    def splitKeys(text):
        return [atoi(c) for c in re.split('(\d+)', text)]

    filenames = glob.glob(dirName + '/lcmlog-*')
    return sorted(filenames, key=splitKeys)


def pruneLogFiles(logFiles):

    maxNumberOfLogs = 20

    logFiles = list(logFiles)
    while len(logFiles) > maxNumberOfLogs:
        filename = logFiles.pop(0)
        print 'deleting:', filename
        os.remove(filename)

    return logFiles


def updateLogCatalog(dirName, logCatalog):


    logFiles = getExistingLogFiles(dirName)
    logFiles = pruneLogFiles(logFiles)

    if not logFiles:
        return

    for logFile in logFiles:
        updateLogInfo(logFile, logCatalog)


def watchThreadMainLoop(dirName):

    while True:
        updateLogCatalog(dirName, logCatalog)
        time.sleep(0.3)


def startWatchThread(dirName):
    watchThread = threading.Thread(target=watchThreadMainLoop, args=[dirName])
    watchThread.daemon = True
    watchThread.start()
    return watchThread


def main():

    try:
        logFileDir = sys.argv[1]
    except IndexError:
        print 'Usage: %s <log file directory>' % sys.argv[0]
        sys.exit(1)

    spy.findLCMModulesInSysPath()

    watchThread = startWatchThread(logFileDir)
    serverThread = ServerThread()
    serverThread.start()

    try:
        while watchThread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
