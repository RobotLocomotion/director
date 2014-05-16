import math
import takktile
from time import sleep

from ddapp import lcmUtils

sensorLocationLeft = {}

sensorLocationLeft[10] = ['left_finger_middle_link_3', [0.01,  0.015, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[11] = ['left_finger_middle_link_3', [0.022, 0.022, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[12] = ['left_finger_middle_link_3', [0.01,  0.015,  0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[13] = ['left_finger_middle_link_3', [0.022, 0.022,  0.008], [0.0, 0.0, 0.0]]

sensorLocationLeft[20] = ['left_finger_1_link_3', [0.01,  0.015, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[21] = ['left_finger_1_link_3', [0.022, 0.022, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[22] = ['left_finger_1_link_3', [0.01,  0.015,  0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[23] = ['left_finger_1_link_3', [0.022, 0.022,  0.008], [0.0, 0.0, 0.0]]

sensorLocationLeft[33] = ['left_finger_2_link_3', [0.01,  0.015, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[32] = ['left_finger_2_link_3', [0.022, 0.022, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[31] = ['left_finger_2_link_3', [0.01,  0.015,  0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[30] = ['left_finger_2_link_3', [0.022, 0.022,  0.008], [0.0, 0.0, 0.0]]

sensorLocationLeft[50] = ['l_hand_face', [-0.04, 0.0,  0.03], [0.0, 0.0, 0.0]]
sensorLocationLeft[51] = ['l_hand_face', [-0.04, 0.0,  0.02], [0.0, 0.0, 0.0]]
sensorLocationLeft[52] = ['l_hand_face', [-0.04, 0.0,  0.01], [0.0, 0.0, 0.0]]
sensorLocationLeft[53] = ['l_hand_face', [-0.04, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[54] = ['l_hand_face', [-0.04, 0.0, -0.01], [0.0, 0.0, 0.0]]
sensorLocationLeft[60] = ['l_hand_face', [-0.02, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[61] = ['l_hand_face', [-0.01, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[62] = ['l_hand_face', [ 0.00, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[63] = ['l_hand_face', [ 0.01, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[64] = ['l_hand_face', [ 0.02, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[70] = ['l_hand_face', [ 0.04, 0.0,  0.03], [0.0, 0.0, 0.0]]
sensorLocationLeft[71] = ['l_hand_face', [ 0.04, 0.0,  0.02], [0.0, 0.0, 0.0]]
sensorLocationLeft[72] = ['l_hand_face', [ 0.04, 0.0,  0.01], [0.0, 0.0, 0.0]]
sensorLocationLeft[73] = ['l_hand_face', [ 0.04, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationLeft[74] = ['l_hand_face', [ 0.04, 0.0, -0.01], [0.0, 0.0, 0.0]]

sensorLocationRight = {}
#Fill this in

class TakktileVis(object):

    def __init__(self, name, topic, sensorLocations, windowSize = 10, pubRate = 6):
        self.name = name
        self.sensorLocations = sensorLocations

        lcmUtils.addSubscriber(topic, takktile.state_t, self.drawBlips)

        self.active = False
        self.doTare = True

        self.taredSensorValues = {}
        self.tareCount = 0
        self.tareWindow = windowSize

        self.pubRate = pubRate
        self.pubCount = 0

    def tare(self):
        self.doTare = True

    def tareData(self, data):
        if self.doTare:
            for i in range(data.data_length):
                if data.id[i] in self.taredSensorValues:
                    self.taredSensorValues[data.id[i]] += data.force[i] / self.tareWindow
                else:
                    self.taredSensorValues[data.id[i]] = data.force[i] / self.tareWindow
            self.tareCount += 1

        if self.tareCount >= self.tareWindow:
            self.doTare = False
            self.tareCount = 0

    def drawBlips(self, data):

        if self.doTare:
            self.tareData(data)

        elif self.active:
            self.pubCount +=1

            if self.pubCount >= self.pubRate:
                self.pubCount = 0
                sensorValues = {}
                for i in range(data.data_length):
                    sensorValues[data.id[i]] = data.force[i]

                d = DebugData()
                for x in sensorValues.keys():
                    frame, pos, rpy = sensorLocation[x]
                    f = robotStateModel.getLinkFrame(frame)
                    t = transformUtils.frameFromPositionAndRPY(pos, rpy)
                    t.PostMultiply()
                    t.Concatenate(f)
                    redColor = 1.0 - (sensorValues[x] / self.taredSensorValues[x])
                    greenColor = 1 - redColor
                    d.addSphere(t.GetPosition(),
                                radius=0.005,
                                color=[redColor,greenColor,0])
                vis.updatePolyData(d.getPolyData(), self.name, colorByName='RGB255')

            else:
                return

