import os
import sys
import time
import lcm
import functools
import threading
import glob
import json
import re
import select
import numpy as np

from ddapp import lcmspy as spy


VIDEO_LCM_URL = 'udpm://239.255.76.50:7650?ttl=1'


class FieldData(object):

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        keys = self.__dict__.keys()
        return 'FieldData(%s)' % ', '.join(['%s=%r' % (k,v) for k, v in self.__dict__.iteritems()])


def getRecentUtimes(utimeMap, seconds):

    utimes = np.array(utimeMap.keys())
    if not len(utimes):
        return None

    utimes.sort()
    endTime = utimes[-1]
    startTime = max(0, endTime - seconds*1e6)
    startIndex = utimes.searchsorted(startTime)
    utimes = utimes[startIndex:]

    if not len(utimes):
        return None

    return utimes


class LCMPoller(object):

    def __init__(self, lc):
        self.lc = lc
        self.poll = select.poll()
        self.poll.register(self.lc.fileno())

    def handleLCM(self, timeout=100):
        if self.poll.poll(timeout):
            self.lc.handle()


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

    def __init__(self, utimes, logLookup, speed):
        self.fps = 60
        self.shouldStop = False
        self.utimes = utimes
        self.logLookup = logLookup
        self.speed = speed
        self.lc = lcm.LCM(VIDEO_LCM_URL)

    def start(self):
        self.shouldStop = False
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.shouldStop = True
        self.thread.join()

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

    def __init__(self, sharedUtimeMap):

        self.sharedUtimeMap = sharedUtimeMap
        self.utimes = None
        self.playbackThread = None
        self.syncThread = None
        self.timeWindow = 60
        self.logLookup = LogLookup()
        self.lc = lcm.LCM(VIDEO_LCM_URL)
        self.lc.subscribe('VIDEO_PLAYBACK_CONTROL', self.onControlMessage)

    def start(self):
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.shouldStop = False
        self.thread.start()

    def stop(self):
        self.shouldStop = True
        self.thread.join()

    def getUtimeIndex(self, data):

        assert 0.0 <= data.value <= 1.0
        return (len(self.utimes)-1)*data.value


    def onFrameRequest(self, data):

        self.stopPlaybackThread()

        if self.utimes is None:

            self.logLookup.setUtimeMap(dict(self.sharedUtimeMap))
            self.utimes = getRecentUtimes(self.logLookup.utimeMap, seconds=self.timeWindow)

            if self.utimes is None:
                print 'no utimes cataloged'
                return

            print 'starting review with utimes %d %d' % (self.utimes[0], self.utimes[1])


        utimeIndex = self.getUtimeIndex(data)
        utimeRequest = self.utimes[utimeIndex]
        image, filename = self.logLookup.getImage(utimeRequest)

        print 'location: %.2f  index: %d  utime: %d   timeDelta:  %.3f    file: %s' % (data.value, utimeIndex, utimeRequest, (self.utimes[-1] - self.utimes[utimeIndex])*1e-6, os.path.basename(filename))

        self.lc.publish('VIDEO_PLAYBACK_IMAGE', image.encode())


    def onResume(self, data):
        self.stopPlaybackThread()
        self.utimes = None
        self.logLookup.closeLogs()
        return


    def onPlay(self, data):

        self.stopPlaybackThread()

        if self.utimes is None:
            print 'cannot play.  no utimes available'
            return

        startIndex = self.getUtimeIndex(data)
        playbackUtimes = self.utimes[startIndex:]
        self.playbackThread = PlayThread(playbackUtimes, self.logLookup, speed=data.speed)
        self.playbackThread.start()


    def stopPlaybackThread(self):
        if self.playbackThread:
            self.playbackThread.stop()
        self.playbackThread = None

        if self.syncThread:
            self.syncThread.stop()
        self.syncThread = None


    def onLogSync(self):
        self.syncThread = LogSyncThread(self.sharedUtimeMap)
        self.syncThread.start()


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
        elif data.command == 'log-sync':
            self.onLogSync()


    def mainLoop(self):
        poll = LCMPoller(self.lc)
        while not self.shouldStop:
            poll.handleLCM()


class LogSyncThread(object):

    def __init__(self, sharedUtimeMap):

        self.sharedUtimeMap = sharedUtimeMap
        self.utimes = None
        self.logLookup = LogLookup()
        self.lastPublishTime = time.time()
        self.publishFrequency = 1/60.0
        self.lcListen = lcm.LCM()
        self.lc = lcm.LCM(VIDEO_LCM_URL)
        self.sub = self.lcListen.subscribe('EST_ROBOT_STATE', self.onControlMessage)


    def start(self):
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.shouldStop = False
        self.thread.start()

    def stop(self):
        self.shouldStop = True
        self.logLookup.closeLogs()
        self.thread.join()
        self.lcListen.unsubscribe(self.sub)


    def updateLastPublishTime(self):
        self.lastPublishTime = time.time()

    def getLastPublishElapsedTime(self):
        return time.time() - self.lastPublishTime


    def onFrameRequest(self, utimeRequest):

        if self.logLookup.utimeMap is None:

            self.logLookup.setUtimeMap(dict(self.sharedUtimeMap))
            assert len(self.logLookup.utimeMap)

            self.utimes = np.array(self.logLookup.utimeMap.keys())
            self.utimes.sort()


        requestIndex = self.utimes.searchsorted(utimeRequest)

        if requestIndex >= len(self.utimes):
            requestIndex = len(self.utimes)-1

        utimeFrame =  self.utimes[requestIndex]


        image, filename = self.logLookup.getImage(utimeFrame)

        print 'utime request: %d   utime frame:  %d   delta:  %f   file: %s' % (utimeRequest, utimeFrame,  (utimeFrame-utimeRequest)*1e-6, os.path.basename(filename))

        self.lc.publish('VIDEO_PLAYBACK_IMAGE', image.encode())
        self.updateLastPublishTime()


    def onControlMessage(self, channel, msgBytes):

        if self.getLastPublishElapsedTime() > self.publishFrequency:
            msg = spy.decodeMessage(msgBytes)
            self.onFrameRequest(msg.utime)


    def mainLoop(self):
        poll = LCMPoller(self.lcListen)
        while not self.shouldStop:
          poll.handleLCM()


class CatalogThread(object):


    def __init__(self, logDir, videoChannel):

        self.videoChannel = videoChannel
        self.logDir = logDir

        self.pruneEnabled = True
        self.maxNumberOfFiles = 30
        self.cropTimeWindow = 60*30
        self.utimeMap = {}
        self.catalog = {}


    def start(self):
        self.thread = threading.Thread(target=self.mainLoop)
        self.thread.daemon = True
        self.shouldStop = False
        self.thread.start()

    def stop(self):
        self.shouldStop = True
        self.thread.join()

    def mainLoop(self):
        while not self.shouldStop:
            self.updateCatalog()
            time.sleep(0.3)


    def updateCatalog(self):

        logFiles = self.getExistingLogFiles(self.logDir)

        if self.pruneEnabled:
            logFiles = self.pruneLogFiles(logFiles, self.maxNumberOfFiles)

        for logFile in logFiles:
            self.updateLogInfo(logFile)


    def updateLogInfo(self, filename):

        fieldData = self.catalog.get(filename)

        if not fieldData:
            print 'discovered new file:', filename
            fieldData = FieldData(filename=filename, fileSize=0, lastFilePos=0, channelTypes={})
            self.catalog[filename] = fieldData
            self.cropUtimeMap(self.utimeMap, self.cropTimeWindow)


        log = lcm.EventLog(filename, 'r')
        fileSize = log.size()

        # if the log file is the same size as the last time it was inspected
        # then there is no more work to do, return.
        if fileSize == fieldData.fileSize:
            return

        fieldData.fileSize = fileSize

        # seek to the last processed event, if one exists, then read past it
        if fieldData.lastFilePos > 0:
            log.seek(fieldData.lastFilePos)
            event = log.read_next_event()


        while True:

            filepos = log.tell()
            event = log.read_next_event()
            if not event:
                break

            fieldData.lastFilePos = filepos
            timestamp = event.timestamp
            channel = event.channel

            if channel == self.videoChannel:
                self.utimeMap[timestamp] = (filename, filepos)


            # maintain a catalog of channelName->messageType
            #if channel not in fieldData.channelTypes:
            #    messageClass = spy.getMessageClass(event.data)
            #    fieldData.channelTypes[channel] = messageClass


        log.close()


    @staticmethod
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


    @staticmethod
    def pruneLogFiles(logFiles, maxNumberOfFiles):

        logFiles = list(logFiles)

        while len(logFiles) > maxNumberOfFiles:
            filename = logFiles.pop(0)
            print 'deleting:', filename
            os.remove(filename)

        return logFiles


    @staticmethod
    def cropUtimeMap(utimeMap, timeWindow):

        if not len(utimeMap):
            return

        utimes = getRecentUtimes(utimeMap, timeWindow)
        cropTime = utimes[0]

        for utime in utimeMap.keys():
            if utime < cropTime:
                del utimeMap[utime]


def main():

    try:
        logFileDir = sys.argv[1]
    except IndexError:
        print 'Usage: %s <log file directory>' % sys.argv[0]
        sys.exit(1)

    spy.findLCMModulesInSysPath()

    catalogThread = CatalogThread(logFileDir, 'DECKLINK_VIDEO_CAPTURE')
    catalogThread.start()


    serverThread = ServerThread(catalogThread.utimeMap)
    serverThread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
