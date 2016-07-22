from director.tasks.taskuserpanel import TaskUserPanel
from director import lcmUtils
import bot_core
import time
import warnings

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

        self.min = 0.0
        self.max = 1.0
        self.hands = ['left', 'right']
        self.robot_name = ""

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

    def close_left(self, amount=0.5):
        self.close("left", amount)

    def close_right(self, amount=0.5):
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

        self.addManualButton('Open Left Hand', self.ft.open_left)
        self.addManualButton('Open Right Hand', self.ft.open_right)

        self.addManualSpacer()
        self.addManualButton('Close 50% Left Hand', self.ft.close_left)
        self.addManualButton('Close 50% Right Hand', self.ft.close_right)