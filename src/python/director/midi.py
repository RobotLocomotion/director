

import pypm
import array
import time
import inspect
import threading

class TriggerFinger(object):
    pads = range(1,17)
    faders = range(17, 21)
    dials = range(21, 29)


_initialized = False

def init():
    global _initialized
    if not _initialized:
        pypm.Initialize()
        _initialized = True


def printDevices():
    init()
    for deviceId in xrange(pypm.CountDevices()):
        interf, name, inp, outp, opened = pypm.GetDeviceInfo(deviceId)

        if inp:
            inOutType = '(input)'
        elif outp:
            inOutType = '(output)'
        else:
            inOutType = '(unknown)'

        status = '(opened)' if opened == 1 else '(unopened)'

        print deviceId, name, inOutType, status


def findInputDevice(deviceName):

    init()
    for deviceId in xrange(pypm.CountDevices()):
        interf, name, inp, outp, opened = pypm.GetDeviceInfo(deviceId)
        if inp and (deviceName in name):
                return deviceId

def findOutDevice(deviceName):

    init()
    for deviceId in xrange(pypm.CountDevices()):
        interf, name, inp, outp, opened = pypm.GetDeviceInfo(deviceId)
        if outp and (deviceName in name):
                return deviceId


def findTriggerFinger():
    return findInputDevice('USB Trigger Finger MIDI')

def findKorgNanoKontrol2():
    deviceId = findInputDevice('nanoKONTROL2 SLIDER/KNOB')
    if deviceId is None:
        deviceId = findInputDevice('nanoKONTROL2 MIDI 1')
    return deviceId

class IOControl:
    def __init__(self, id, getter, data):
        assert id>0 and id<93
        arg=inspect.getargspec(getter).args
        assert (len(arg)==2 and arg[0]!='self') or (len(arg)==3 and arg[0]=='self')
        self.id=id
        self.getter=getter
        self.send=None
        self.data=data
        self.value=0

    def setter(self,val):
        if self.send is not None:
            val=int(min(max(0,val),127))
            self.value=val
            self.send([self.id, val])

    def setSender(self, sender):
        self.send=sender

class BehringerBCF200(object):
    def __init__(self, deviceIdIn=None, deviceIdOut=None):
        init()

        if deviceIdIn is None:
            deviceIdIn = findInputDevice('BCF2000 MIDI 1')
        if deviceIdOut is None:
            deviceIdOut = findOutDevice('BCF2000 MIDI 1')

        assert deviceIdIn is not None
        assert deviceIdOut is not None

        self.streamIn = pypm.Input(deviceIdIn)
        self.streamOut = pypm.Output(deviceIdOut,0)
        self.vars = [None]*96
        self.polling=True
        self.thread = threading.Thread(target=self.pollingThread)
        self.thread.setDaemon(True)
        self.thread.start();

    def pollingThread(self):
        while(self.polling):
            msg=self.getNextMessage()
            if msg is not None and msg[1]>0 and msg[1]<93 and self.vars[msg[1]] is not None:
                self.vars[msg[1]].getter(msg[2],self.vars[msg[1]].data)
                self.vars[msg[1]].value=msg[2]
            time.sleep(0)


    def addVarieble(self, var):
        self.vars[var.id]=var
        self.vars[var.id].send=self.sendMessage

    def clear(self):
        for var in self.vars:
            if var is not None:
                var.send=None
        self.vars = [None]*96

    def _convertMessage(self, streamMessage):
        '''
        Given a stream message [[b1, b2, b3, b4], timestamp]
        Return [timestamp, b1, b2, b3, b4]
        '''
        midiData, timestamp = streamMessage
        b1, b2, b3, b4 = midiData
        return [timestamp, b2, b3]

    def getNextMessage(self):
        '''
        Returns the next message in the buffer.
        Does not block.  Returns None if buffer is empty.
        '''
        if self.streamIn.Poll():
            message = self.streamIn.Read(1)[0]
            return self._convertMessage(message)

    def getMessages(self):
        '''
        Returns a list of all messages currently in the buffer, up to 1024 messages.
        Does not block.  Returns empty list if buffer is empty.
        '''
        if self.streamIn.Poll():
            messages = self.streamIn.Read(1024)
            return [self._convertMessage(message) for message in messages]
        return []

    def sendMessage(self, data):
        '''
        Sends data to the slider board.
        '''
        assert len(data)==2
        self.streamOut.Write([[[176,data[0],data[1]],0]])

class MidiReader(object):

    def __init__(self, deviceId=None):
        init()

        if deviceId is None:
            deviceId = findTriggerFinger()

        assert deviceId is not None

        self.stream = pypm.Input(deviceId)

    def _convertMessage(self, streamMessage):
        '''
        Given a stream message [[b1, b2, b3, b4], timestamp]
        Return [timestamp, b1, b2, b3, b4]
        '''
        midiData, timestamp = streamMessage
        b1, b2, b3, b4 = midiData
        return [timestamp, b1, b2, b3, b4]

    def getNextMessage(self):
        '''
        Returns the next message in the buffer.
        Does not block.  Returns None if buffer is empty.
        '''
        if self.stream.Poll():
            message = self.stream.Read(1)[0]
            return self._convertMessage(message)

    def getMessages(self):
        '''
        Returns a list of all messages currently in the buffer, up to 1024 messages.
        Does not block.  Returns empty list if buffer is empty.
        '''
        if self.stream.Poll():
            messages = self.stream.Read(1024)
            return [self._convertMessage(message) for message in messages]
        return []


