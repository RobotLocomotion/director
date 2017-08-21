from director import midi
from director.timercallback import TimerCallback


class KorgNanoKontrol(object):

    def __init__(self):
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.tick
        self.stop = self.timer.stop
        self.reader = None
        self.initReader()

        self.inputs = {
            'slider' : [0, 8, True],
            'dial' : [16, 8, True],
            'r_button' : [64, 8, False],
            'm_button' : [48, 8, False],
            's_button' : [32, 8, False],
            'track_left' : [58, 1, False],
            'track_right' : [59, 1, False],
            'cycle' : [46, 1, False],
            'marker_set' : [60, 1, False],
            'marker_left' : [61, 1, False],
            'marker_right' : [62, 1, False],
            'rewind' : [43, 1, False],
            'fastforward' : [44, 1, False],
            'stop' : [42, 1, False],
            'play' : [41, 1, False],
            'record' : [45, 1, False],
        }

        signalNames = []

        for inputName, inputDescription in self.inputs.items():
            channelStart, numChannels, isContinuous = inputDescription

            for i in range(numChannels):

                channelId = '' if numChannels == 1 else '_%d' % i
                if isContinuous:
                    signalNames.append('%s%s_value_changed' % (inputName, channelId))
                else:
                    signalNames.append('%s%s_pressed' % (inputName, channelId))
                    signalNames.append('%s%s_released' % (inputName, channelId))

        self.callbacks = callbacks.CallbackRegistry(signalNames)

    def start(self):
        self.initReader()
        if self.reader is not None:
            self.timer.start()

    def initReader(self):

        if self.reader:
            return
        try:
            self.reader = midi.MidiReader(midi.findKorgNanoKontrol2())
        except:
            print('midi controller not found.')
            self.reader = None

    def onMidiCommand(self, channel, value):

        #print channel, '%.2f' % value

        inputs = self.inputs

        for inputName, inputDescription in inputs.items():
            channelStart, numChannels, isContinuous = inputDescription

            if channelStart <= channel < (channelStart + numChannels):

                if numChannels > 1:
                    inputName = '%s_%d' % (inputName, channel - channelStart)

                if isContinuous:
                    self.onContinuousInput(inputName, value)
                elif value == 1:
                    self.onButtonDown(inputName)
                elif value == 0:
                    self.onButtonUp(inputName)

    def onContinuousInput(self, name, value):
        #print name, '%.2f' % value
        self.callbacks.process(name + '_value_changed', value)

    def onButtonDown(self, name):
        #print name, 'down'
        self.callbacks.process(name + '_pressed')

    def onButtonUp(self, name):
        #print name, 'up'
        self.callbacks.process(name + '_released')

    def tick(self):
        try:
            messages = self.reader.getMessages()
        except:
            messages = []

        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        for channel, value in targets.items():
            position = value/127.0
            self.onMidiCommand(channel, position)
