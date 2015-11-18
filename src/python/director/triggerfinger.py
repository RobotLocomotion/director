import sys
import time
import math

from ddapp import midi
from ddapp import lcmUtils
from ddapp import consoleapp
from ddapp.utime import getUtime
import drc as lcmdrc

from ddapp.timercallback import TimerCallback

class TriggerFingerPublisher():
	def __init__(self, lcmChannel):
		self.lcmChannel = lcmChannel
		self.reader = midi.MidiReader()
		self.timer = TimerCallback()
		self.timer.callback = self.tick
		self.msg = lcmdrc.trigger_finger_t()
	
	def startPublishing(self):	
		print 'Publishing on ' + self.lcmChannel
		self.timer.start()

	def publish(self):
		messages = self.reader.getMessages()
		
		for message in messages:
			channel = message[2]
			val = message[3] / 127.0
			if channel is 102:
				self.msg.slider1 = val
			elif channel is 103:
				self.msg.slider2 = val
			elif channel is 104:
				self.msg.slider3 = val
			elif channel is 105:
				self.msg.slider4 = val
			elif channel is 106:
				self.msg.knob1 = val
			elif channel is 107:
				self.msg.knob2 = val
			elif channel is 108:
				self.msg.knob3 = val
			elif channel is 109:
				self.msg.knob4 = val
			elif channel is 110:
				self.msg.knob5 = val
			elif channel is 111:
				self.msg.knob6 = val
			elif channel is 112:
				self.msg.knob7 = val
			elif channel is 113:
				self.msg.knob8 = val
		
		if len(messages) is not 0:
			self.msg.utime = getUtime()
			lcmUtils.publish(self.lcmChannel, self.msg)

	
	def tick(self):
		self.publish()


def main(argv):
	if len(argv) is not 2:
		print 'Usage: ddConsoleApp triggerfinger.py <LCM_CHANNEL>'
		return

	publisher = TriggerFingerPublisher(argv[1])
	publisher.startPublishing()
	consoleapp.ConsoleApp.start()

if __name__ == "__main__":
    main(sys.argv)