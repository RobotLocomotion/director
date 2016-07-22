from director.tasks.taskuserpanel import TaskUserPanel
from director import lcmUtils
from director import objectmodel as om
import director.tasks.robottasks as rt
import bot_core
import time
import warnings
from functools import partial

def custom_formatwarning(msg, *a):
    # ignore everything except the message
    return str(msg) + '\n'

warnings.simplefilter('always', UserWarning)
warnings.formatwarning = custom_formatwarning


# logic and communication
class FingerTest(object):
    def __init__(self, channel="DESIRED_HAND_ANGLES"):
        self.channel = channel
        self.finger_joint_list = ['IndexFingerMotorPitch1',     # 0
                                  'MiddleFingerMotorPitch1',    # 1
                                  'PinkyFingerMotorPitch1',     # 2
                                  'ThumbFingerMotorPitch1',     # 3
                                  'ThumbFingerMotorPitch2',     # 4
                                  'ThumbFingerMotorRoll'        # 5
        ]

        # never change these hard constraints
        self.hard_min = 0.0
        self.hard_max = 1.0

        self.min = self.hard_min
        self.max = 0.5
        self.hands = ['left', 'right']
        self.robot_name = ""

    def setMin(self, val_min):
        # set new min bound but check for hard constraints
        self.min = max(val_min, self.hard_min)

    def setMax(self, val_max):
        # set new max bound but check for hard constraints
        self.min = min(val_max, self.hard_max)

    # send desired values to all fingers, e.g. open and close fist
    def send(self, hand, angles):
        if hand not in self.hands:
            raise Exception("Provide one of "+str(self.hands)+" as hand!")

        msg = bot_core.joint_angles_t()
        msg.utime = int(time.time()*1000000)
        msg.num_joints = len(self.finger_joint_list)
        msg.robot_name = self.robot_name

        msg.joint_name = []
        for n in self.finger_joint_list:
            msg.joint_name.append(hand+n)

        if len(angles) != len(msg.joint_name):
            raise Exception("You need to provide exactly "+str(len(msg.joint_name))+" joint values!\n"
                            "Joints (in order): "+str(msg.joint_name))

        assert (len(self.finger_joint_list) == len(angles))
        msg.joint_position = angles

        lcmUtils.publish(self.channel, msg)

    def open(self, hand_name):
        self.send(hand_name, [self.min]*len(self.finger_joint_list))

    def open_left(self):
        self.open("left")

    def open_right(self):
        self.open("right")

    def close(self, hand_name, amount):
        if amount<self.min or amount>self.max:
            warnings.warn("The amount of closure must be in range [%f, %f]." % (self.min, self.max) )
            amount = max(amount,self.min)
            amount = min(amount, self.max)
        self.send(hand_name, [amount] * len(self.finger_joint_list))

    def close_left(self, amount):
        self.close("left", amount)

    def close_right(self, amount):
        self.close("right", amount)

    # send desired closure to individual finger
    def send_finger(self, hand, finger, amount):
        if hand not in self.hands:
            raise Exception("Provide one of " + str(self.hands) + " as hand!")
        if finger not in self.finger_joint_list:
            raise Exception("Provide one of " + str(self.finger_joint_list) + " as finger!")
        if amount < self.min or amount > self.max:
            warnings.warn("The amount of closure must be in range [%f, %f]. Value will be truncated to range." % (self.min, self.max))
            amount = max(amount, self.min)
            amount = min(amount, self.max)

        msg = bot_core.joint_angles_t()
        msg.utime = int(time.time() * 1000000)
        msg.num_joints = 1
        msg.robot_name = self.robot_name

        msg.joint_name = [hand + finger]
        msg.joint_position = [amount]

        lcmUtils.publish(self.channel, msg)

    def open_finger(self, hand, finger):
        self.send_finger(hand, finger, 0)

    def close_finger(self, hand, finger, amount):
        self.send_finger(hand, finger, amount)

# GUI
class FingerTestTaskPanel(TaskUserPanel):
    def __init__(self,fingertest):
        self.ft = fingertest
        TaskUserPanel.__init__(self, windowTitle='Finger test')

        self.active_hand_label = "Active Hand"
        self.min_label = "Min Amount"
        self.max_label = "Max Amount"

        self.active_hand = "left"

        self.step = 0.1

        self.addButtons()
        self.addProperties()
        self.addTasks()


    def addButtons(self):
        self.addManualButton('Open Left Hand', self.ft.open_left)
        self.addManualButton('Open Right Hand', self.ft.open_right)
        self.addManualSpacer()
        self.addManualButton('Open Active Hand', self.open)
        self.addManualButton('Max Close Active Hand', self.close_max)
        self.addManualSpacer()
        self.addManualButton('New Task Sequence', self.addTasks)

    def addProperties(self):
        # active hand
        self.params.addProperty(self.active_hand_label, 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        # angle limits
        self.params.addProperty(self.min_label, self.ft.min, attributes=om.PropertyAttributes(decimals=1, minimum=self.ft.hard_min, maximum=self.ft.hard_max, singleStep=0.1))
        self.params.addProperty(self.max_label, self.ft.max, attributes=om.PropertyAttributes(decimals=1, minimum=self.ft.hard_min, maximum=self.ft.hard_max, singleStep=0.1))

    def addTasks(self):
        self.taskTree.removeAllTasks()
        # tasks
        openSequenceTask = self.taskTree.addGroup('Opening/Closing Hand Sequence')
        self.taskTree.onAddTask(rt.CallbackTask(callback=self.open, name="Open"), parent=openSequenceTask, copy=False)

        if(self.ft.min < self.step):
            v = self.ft.min+self.step
        else:
            v = self.ft.min
        while v<=self.ft.max:
            self.taskTree.onAddTask(rt.CallbackTask(callback=partial(self.close, v), name="Close "+str(v)), parent=openSequenceTask, copy=False)
            self.taskTree.onAddTask(rt.CallbackTask(callback=self.open, name="Open"), parent=openSequenceTask, copy=False)
            v += self.step

    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == self.active_hand_label:
            self.active_hand = self.params.getPropertyEnumValue(self.active_hand_label).lower()
            self.appendMessage("Active hand: " + self.active_hand)

        if propertyName == self.min_label:
            self.ft.min = self.params.getProperty(self.min_label)

        if propertyName == self.max_label:
            self.ft.max = self.params.getProperty(self.max_label)

    def open(self):
        self.appendMessage("Opening "+self.active_hand+" hand")
        self.ft.open(self.active_hand)

    def close(self, amount):
        self.appendMessage("Closing " + self.active_hand + " hand by " + str(amount))
        self.ft.close(self.active_hand, amount)

    def close_max(self):
        self.close(self.ft.max)