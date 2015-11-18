
from PythonQt import QtCore, QtGui
import functools

from ddapp import applogic as app

_spreadsheetView = None
def getSpreadsheetView():
    return _spreadsheetView


def setSpreadsheetColumnData(columnIndex, name, data):

    sv = getSpreadsheetView()
    model = sv.model()

    model.item(0, columnIndex).setText(name)
    for i, value in enumerate(data):
          model.item(i + 1, columnIndex).setText(value)


def updateSpreadsheetPoses(poseCollection, poseName=None):

    poseMap = poseCollection.map()
    for i, poseName in enumerate(sorted(poseMap.keys())):
        setSpreadsheetColumnData(i + 3, poseName, poseMap[poseName])


def initSpreadsheetColumns(costCollection):

    jointNames = [
      'base_x',
      'base_y',
      'base_z',
      'base_roll',
      'base_pitch',
      'base_yaw',
      'back_bkz',
      'back_bky',
      'back_bkx',
      'l_arm_usy',
      'l_arm_shx',
      'l_arm_ely',
      'l_arm_elx',
      'l_arm_uwy',
      'l_leg_hpz',
      'l_leg_hpx',
      'l_leg_hpy',
      'l_leg_kny',
      'l_leg_aky',
      'l_leg_akx',
      'l_arm_mwx',
      'r_arm_usy',
      'r_arm_shx',
      'r_arm_ely',
      'r_arm_elx',
      'r_arm_uwy',
      'r_leg_hpz',
      'r_leg_hpx',
      'r_leg_hpy',
      'r_leg_kny',
      'r_leg_aky',
      'r_leg_akx',
      'r_arm_mwx',
      'neck_ay',
    ]

    costData = [
      0.0,
      0.0,
      100.0,
      100.0,
      0.0,
    ]

    costData += [100.0 for i in xrange(len(jointNames) - len(costData))]
    costCollection.setItem('default_costs', costData)

    setSpreadsheetColumnData(0, 'joint_names', jointNames)
    setSpreadsheetColumnData(1, 'default_costs', costData)


def init(poseCollection, costCollection):

    global _spreadsheetView
    _spreadsheetView = app.getViewManager().createView('Spreadsheet View', 'Spreadsheet View')

    updateMethod = functools.partial(updateSpreadsheetPoses, poseCollection)
    poseCollection.connect('itemChanged(const QString&)', updateMethod)
    poseCollection.connect('itemAdded(const QString&)', updateMethod)
    poseCollection.connect('itemRemoved(const QString&)', updateMethod)

    initSpreadsheetColumns(costCollection)
    updateMethod()

