import math
import takktile
from time import sleep

from ddapp import lcmUtils
from ddapp.debugVis import DebugData
from ddapp import roboturdf
from ddapp import transformUtils
from ddapp import visualization as vis

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

    def __init__(self, name, topic, sensorLocations, model, windowSize = 10):
        self.name = name
        self.sensorLocations = sensorLocations
        self.robotStateModel = model

        sub = lcmUtils.addSubscriber(topic, takktile.state_t, self.drawBlips)
        sub.setSpeedLimit(30)

        self.active = False
        self.doTare = True

        self.taredSensorValues = {}
        self.tareCount = 0
        self.tareWindow = windowSize

        self.frames = {}

    def getFrameNames
    # get frame names from sensor location dict

    def updateFrames(self):
        if not self.frames:
            for name in self.getFrameNames():
                self.frames[name] = vtk.vtkTransform()
        for name, frame in self.frames.iteritems():
            frame.SetMatrix(self.robotStateModel.getLinkFrame(name).GetMatrix())

    def createSpheres(self):
        self.d = DebugData()

        for x in sensorValues.keys():
            frame, pos, rpy = self.sensorLocations[x]

            t = transformUtils.frameFromPositionAndRPY(pos, rpy)
            t.PostMultiply()
            t.Concatenate(self.frames[frame])


    def tare(self):
        self.taredSensorValues = {}
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
            sensorValues = {}
            for i in range(data.data_length):
                sensorValues[data.id[i]] = data.force[i]

            for x in sensorValues.keys():
                frame, pos, rpy = self.sensorLocations[x]

                t = transformUtils.frameFromPositionAndRPY(pos, rpy)
                t.PostMultiply()
                t.Concatenate(f)
                redColor = 1.0 - (sensorValues[x] / self.taredSensorValues[x])
                greenColor = 1 - redColor
                d.addSphere(t.GetPosition(),
                            radius=0.005,
                            color=[redColor,greenColor,0],
                            resolution=8)
            vis.updatePolyData(d.getPolyData(), self.name, colorByName='RGB255')


