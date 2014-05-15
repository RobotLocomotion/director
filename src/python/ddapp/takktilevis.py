import math
import takktile
from time import sleep

sensor_location = {}
sensor_location[50] = [-0.04, 0.0, 0.04]
sensor_location[51] = [-0.04, 0.0, 0.03]
sensor_location[52] = [-0.04, 0.0, 0.02]
sensor_location[53] = [-0.04, 0.0, 0.01]
sensor_location[54] = [-0.04, 0.0, 0.00]

sensor_location[60] = [-0.02, 0.0, 0.04]
sensor_location[61] = [-0.02, 0.0, 0.03]
sensor_location[62] = [-0.02, 0.0, 0.02]
sensor_location[63] = [-0.02, 0.0, 0.01]
sensor_location[64] = [-0.02, 0.0, 0.00]

sensor_location[70] = [-0.0, 0.0, 0.04]
sensor_location[71] = [-0.0, 0.0, 0.03]
sensor_location[72] = [-0.0, 0.0, 0.02]
sensor_location[73] = [-0.0, 0.0, 0.01]
sensor_location[74] = [-0.0, 0.0, 0.00]


def drawBlips(data):
    global sensor_locations
    hand = 'l'

    for i in range(data.data_length):
        sensorValues = {}
        sensorValues[data.id[i]] = data.force[i]

    f = robotStateModel.getLinkFrame(hand+'_hand_face')

    d = DebugData()
    for x in sensorValues.keys():
        t = transformUtils.frameFromPositionAndRPY(sensor_location[x], [0,0,0])
        t.PostMultiply()
        t.Concatenate(f)
        redColor = 1.0 - (sensorValues[x] / 500.0)
        greenColor = 1 - redColor
        print redColor, greenColor
        d.addSphere(t.GetPosition(),
                    radius=0.005, 
                    color=[redColor,greenColor,0])

    vis.showPolyData(d.getPolyData(),hand+'_takktile',colorByName='RGB255')



lcmUtils.addSubscriber('TAKKTILE_RAW_LEFT', takktile.state_t, drawBlips)

