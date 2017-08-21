

import pypm
import array
import time

class TriggerFinger(object):
    pads = list(range(1,17))
    faders = list(range(17, 21))
    dials = list(range(21, 29))


_initialized = False

def init():
    global _initialized
    if not _initialized:
        pypm.Initialize()
        _initialized = True


def printDevices():
    init()
    for deviceId in range(pypm.CountDevices()):
        interf, name, inp, outp, opened = pypm.GetDeviceInfo(deviceId)

        if inp:
            inOutType = '(input)'
        elif outp:
            inOutType = '(output)'
        else:
            inOutType = '(unknown)'

        status = '(opened)' if opened == 1 else '(unopened)'

        print(deviceId, name, inOutType, status)


def findInputDevice(deviceName):
    init()
    for deviceId in range(pypm.CountDevices()):
        interf, name, inp, outp, opened = pypm.GetDeviceInfo(deviceId)
        if inp and (deviceName in name):
                return deviceId

def findOutputDevice(deviceName):
    init()
    for deviceId in range(pypm.CountDevices()):
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


