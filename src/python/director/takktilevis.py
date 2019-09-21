import math
import takktile
from time import sleep

from director import lcmUtils
from director.debugVis import DebugData
from director import roboturdf
from director import transformUtils
from director import visualization as vis
from director import vtkAll as vtk

sensorLocationLeft = {}

# This finger is colloquially called the "thumb"
sensorLocationLeft[10] = ['left_finger_middle_link_3', [0.01,  0.015, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[11] = ['left_finger_middle_link_3', [0.022, 0.022, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[12] = ['left_finger_middle_link_3', [0.01,  0.015,  0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[13] = ['left_finger_middle_link_3', [0.022, 0.022,  0.008], [0.0, 0.0, 0.0]]

# Fingers 1 and 2 below are the two fingers that have the base 'scissor' joint
sensorLocationLeft[20] = ['left_finger_1_link_3', [0.01,  0.015, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[21] = ['left_finger_1_link_3', [0.022, 0.022, -0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[22] = ['left_finger_1_link_3', [0.01,  0.015,  0.008], [0.0, 0.0, 0.0]]
sensorLocationLeft[23] = ['left_finger_1_link_3', [0.022, 0.022,  0.008], [0.0, 0.0, 0.0]]

# NOTE: the locations of this sensors is slightly different, thats because this is one of
# the sensors where the wire comes out the other side (relative to the two pads above), resulting
# in a sensor that is rotated 180 degrees. The wires are on the other side to allow for cable
# routing down the ouside of the finger.
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


# NOTE: These sensor locations for a 'right' hand are currently the one and only
# sensor pad connected via the USB interface instead of the ethernet interface.
# for some reason the numbers
# Only the palm works when connected to USB, the finger pads don't work
sensorLocationRight = {}
sensorLocationRight[25] = ['r_hand_face', [-0.04, 0.0,  0.03], [0.0, 0.0, 0.0]]
sensorLocationRight[26] = ['r_hand_face', [-0.04, 0.0,  0.02], [0.0, 0.0, 0.0]]
sensorLocationRight[27] = ['r_hand_face', [-0.04, 0.0,  0.01], [0.0, 0.0, 0.0]]
sensorLocationRight[28] = ['r_hand_face', [-0.04, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[29] = ['r_hand_face', [-0.04, 0.0, -0.01], [0.0, 0.0, 0.0]]
sensorLocationRight[30] = ['r_hand_face', [-0.02, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[31] = ['r_hand_face', [-0.01, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[32] = ['r_hand_face', [ 0.00, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[33] = ['r_hand_face', [ 0.01, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[34] = ['r_hand_face', [ 0.02, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[35] = ['r_hand_face', [ 0.04, 0.0,  0.03], [0.0, 0.0, 0.0]]
sensorLocationRight[36] = ['r_hand_face', [ 0.04, 0.0,  0.02], [0.0, 0.0, 0.0]]
sensorLocationRight[37] = ['r_hand_face', [ 0.04, 0.0,  0.01], [0.0, 0.0, 0.0]]
sensorLocationRight[38] = ['r_hand_face', [ 0.04, 0.0,  0.00], [0.0, 0.0, 0.0]]
sensorLocationRight[39] = ['r_hand_face', [ 0.04, 0.0, -0.01], [0.0, 0.0, 0.0]]


class TakktileVis(object):

    def __init__(self, name, topic, sensorLocations, model, windowSize = 10):
        self.name = name
        self.sensorLocations = sensorLocations
        self.frameNames = self.getFrameNames()
        self.robotStateModel = model

        sub = lcmUtils.addSubscriber(topic, takktile.state_t, self.drawSpheres)
        sub.setSpeedLimit(60)

        self.active = False
        self.doTare = True

        self.taredSensorValues = {}
        self.tareCount = 0
        self.tareWindow = windowSize

        self.frames = {}

    def getFrameNames(self):
        names = []
        for key in list(self.sensorLocations.keys()):
            if not self.sensorLocations[key][0] in names:
                names.append(self.sensorLocations[key][0])
        return names

    def updateFrames(self):
        if not self.frames:
            for name in self.frameNames:
                self.frames[name] = vtk.vtkTransform()
        for name, frame in self.frames.items():
            frame.SetMatrix(self.robotStateModel.getLinkFrame(name).GetMatrix())

    def createSpheres(self, sensorValues):
        d = DebugData()

        for key in list(sensorValues.keys()):
            frame, pos, rpy = self.sensorLocations[key]

            t = transformUtils.frameFromPositionAndRPY(pos, rpy)
            t.PostMultiply()
            t.Concatenate(self.frames[frame])
            d.addSphere(t.GetPosition(),
                        radius=0.005,
                        color=self.getColor(sensorValues[key], key),
                        resolution=8)
        vis.updatePolyData(d.getPolyData(), self.name, colorByName='RGB255')

    def extractSensorData(self, message):
        sensorValues = {}
        for i in range(message.data_length):
            sensorValues[message.id[i]] = message.force[i]
        return sensorValues

    def getColor(self, currentValue, sensorId):
        green = 0.0
        blue = 0.0
        red = 0.0

        delta = float(currentValue) / self.taredSensorValues[sensorId]
        if delta > 1.0:
            green = 1.0/(2*delta)
            blue = 1.0-green
            red = 0.0
        else:
            green = delta
            red = 1.0-green
            blue = 0.0

        return [red, green, blue]

    def tare(self):
        self.taredSensorValues = {}
        self.doTare = True

    def updateTare(self, data):
        if self.doTare:
            for i in range(data.data_length):
                if data.id[i] in self.taredSensorValues:
                    self.taredSensorValues[data.id[i]] += float(data.force[i]+1) / self.tareWindow
                else:
                    self.taredSensorValues[data.id[i]] = float(data.force[i]+1) / self.tareWindow
            self.tareCount += 1

        if self.tareCount >= self.tareWindow:
            self.doTare = False
            self.tareCount = 0

    def drawSpheres(self, message):

        if self.doTare:
            self.updateTare(message)

        elif self.active:
            self.updateFrames()
            sensorValues = self.extractSensorData(message)
            self.createSpheres(sensorValues)


