import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp.debugVis import DebugData

import numpy as np
import math
from time import time
from time import sleep

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class HandControlPanel(object):

    def __init__(self, lDriver, rDriver, robotStateModel, robotStateJointController, view):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.drivers = {}
        self.drivers['left'] = lDriver
        self.drivers['right'] = rDriver

        self.storedCommand = {'left': None, 'right': None}

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddHandControl.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.widget.advanced.sendButton.setEnabled(True)

        # Store calibration for wrist f/t sensors
        self.ft_right_bias = np.array([0.]*6);
        self.ft_left_bias = np.array([0.]*6);
        self.ft_left_calib_pts = [];
        self.ft_left_calib_des = [];
        self.ft_right_calib_pts = [];
        self.ft_right_calib_des = [];
        self.view = view

        # connect the callbacks
        self.widget.basic.openButton.clicked.connect(self.openClicked)
        self.widget.basic.closeButton.clicked.connect(self.closeClicked)
        self.widget.basic.waitOpenButton.clicked.connect(self.waitOpenClicked)
        self.widget.basic.waitCloseButton.clicked.connect(self.waitCloseClicked)
        self.widget.advanced.sendButton.clicked.connect(self.sendClicked)
        self.widget.advanced.calibrateButton.clicked.connect(self.calibrateClicked)
        self.widget.advanced.setModeButton.clicked.connect(self.setModeClicked)
        self.widget.advanced.regraspButton.clicked.connect(self.regraspClicked)
        self.widget.advanced.dropButton.clicked.connect(self.dropClicked)
        self.widget.advanced.repeatRateSpinner.valueChanged.connect(self.rateSpinnerChanged)
        self.ui.fingerControlButton.clicked.connect(self.fingerControlButton)
        self.widget.sensors.rightTareButton.clicked.connect(self.tareRightFT)
        self.widget.sensors.leftTareButton.clicked.connect(self.tareLeftFT)
        self.widget.sensors.rightCalibButton.clicked.connect(self.calibRightFT)
        self.widget.sensors.leftCalibButton.clicked.connect(self.calibLeftFT)
        self.widget.sensors.rightCalibClearButton.clicked.connect(self.calibRightClearFT)
        self.widget.sensors.leftCalibClearButton.clicked.connect(self.calibLeftClearFT)
        PythonQt.dd.ddGroupBoxHider(self.ui.sensors)
        PythonQt.dd.ddGroupBoxHider(self.ui.fingerControl)

        # Bug fix... for some reason one slider is set as disabled
        self.ui.fingerASlider.setEnabled(True)

        # create a timer to repeat commands
        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.targetFps = 20
        self.updateTimer.start()

    def recomputeBiases(self):
        ft_right_bias_new = np.array([0.0]*6);
        for i in range(0, len(self.ft_right_calib_pts)):
            ft = self.ft_right_calib_pts[i];
            des = self.ft_right_calib_des[i];
            ft_right_bias_new[0:3] += (ft[0:3] - des);
            ft_right_bias_new[4:6] += ft[4:6];
        if (len(self.ft_right_calib_pts) > 0):
            ft_right_bias_new /= float(len(self.ft_right_calib_pts));
        self.ft_right_bias = ft_right_bias_new;

        ft_left_bias_new = np.array([0.0]*6);
        for i in range(0, len(self.ft_left_calib_pts)):
            ft = self.ft_left_calib_pts[i];
            des = self.ft_left_calib_des[i];
            ft_left_bias_new[0:3] += (ft[0:3] - des);
            ft_left_bias_new[4:6] += ft[4:6];
        if (len(self.ft_left_calib_pts) > 0):
            ft_left_bias_new /= float(len(self.ft_left_calib_pts));
        self.ft_left_bias = ft_left_bias_new;

    def calibRightFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            ft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
            ft[0] = ft[0]*-1.;
            ft[1] = ft[1]*-1.;
            ft[3] = ft[3]*-1.;
            ft[4] = ft[4]*-1.;
            handMass = 1.0;
            handCOM = [0., 0.1, 0.];
            # force is just down
            rHandFrame = self.robotStateModel.getLinkFrame('r_hand_force_torque')
            rHandForce = rHandFrame.GetInverse().TransformPoint([0,0,-9.8*handMass]);
            # currently calibrating just force :P
            self.ft_right_calib_pts.append(ft);
            self.ft_right_calib_des.append(rHandForce);
        self.recomputeBiases();
            
    def calibLeftFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            ft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
            handMass = 1.0;
            handCOM = [0., 0.1, 0.];
            # force is just down
            lHandFrame = self.robotStateModel.getLinkFrame('l_hand_force_torque')
            lHandForce = lHandFrame.GetInverse().TransformPoint([0,0,-9.8*handMass]);
            # currently calibrating just force :P
            self.ft_left_calib_pts.append(ft);
            self.ft_left_calib_des.append(lHandForce);
        self.recomputeBiases();

    def calibRightClearFT(self):
        self.ft_right_calib_pts = [];
        self.ft_right_calib_des = [];
        self.ft_right_bias = np.array([0.0]*6);
            
    def calibLeftClearFT(self):
        self.ft_left_calib_pts = [];
        self.ft_left_calib_des = [];
        self.ft_left_bias = np.array([0.0]*6);


    def tareRightFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            self.ft_right_bias = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
            self.ft_right_bias[0] = self.ft_right_bias[0]*-1.;
            self.ft_right_bias[1] = self.ft_right_bias[1]*-1.;
            self.ft_right_bias[3] = self.ft_right_bias[3]*-1.;
            self.ft_right_bias[4] = self.ft_right_bias[4]*-1.;
            
    def tareLeftFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            self.ft_left_bias = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
            
    def doFTDraw(self):
        frames = [];
        fts = [];
        vis_names = [];
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            if (self.ui.rightVisCheck.checked):
                rft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
                rft[0] = -1.*rft[0]; # right FT looks to be rotated 180deg around the z axis
                rft[1] = -1.*rft[1];
                rft[3] = -1.*rft[3];
                rft[4] = -1.*rft[4];
                rft -= self.ft_right_bias;
                fts.append(rft);
                frames.append(self.robotStateModel.getLinkFrame('r_hand_force_torque'));
                vis_names.append('ft_right');
            if (self.ui.leftVisCheck.checked):
                lft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
                lft -= self.ft_left_bias;
                fts.append(lft);
                frames.append(self.robotStateModel.getLinkFrame('l_hand_force_torque'));
                vis_names.append('ft_left');
            
        for i in range(0, len(frames)):
            frame = frames[i];
            ft = fts[i];
            offset = [0., 0., 0.];
            p1 = frame.TransformPoint(offset)
            scale = 1.0/25.0; # todo add slider for this?
            scalet = 1.0 / 2.5; 
            p2f = frame.TransformPoint(offset + ft[0:3]*scale);
            p2t = frame.TransformPoint(offset + ft[3:6]);
            normalt = (np.array(p2t) - np.array(p1));
            normalt = normalt /  np.linalg.norm(normalt);
            d = DebugData()
            # force
            if (np.linalg.norm(np.array(p2f) - np.array(p1)) < 0.1):
                d.addLine(p1, p2f, color=[1.0, 0.0, 0.0])
            else:
                d.addArrow(p1, p2f, color=[1.,0.,0.])
            # torque
            if (self.ui.torqueVisCheck.checked):
                d.addCircle(p1, normalt, scalet*np.linalg.norm(ft[3:6]));
            # frame (largely for debug)
            vis.updateFrame(frame, vis_names[i]+'frame', view=self.view);
            vis.updatePolyData(d.getPolyData(), vis_names[i], view=self.view)

    def getModeInt(self, inputStr):
        if inputStr == 'Basic':
            return 0
        if inputStr == 'Pinch':
            return 1
        if inputStr == 'Wide':
            return 2
        if inputStr == 'Scissor':
            return 3
        return 0

    def openClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.widget.advanced.closePercentSpinner.setValue(0.0)

        position = 0.0
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def closeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.widget.advanced.closePercentSpinner.setValue(100.0)

        position = 100.0
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def waitOpenClicked(self):
        sleep(10)
        self.openClicked()

    def waitCloseClicked(self):
        sleep(10)
        self.closeClicked()

    def sendClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        position = float(self.widget.advanced.closePercentSpinner.value)
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def setModeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].setMode(mode)
        self.storedCommand[side] = None

    def calibrateClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.drivers[side].sendCalibrate()
        self.storedCommand[side] = None

    def dropClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.drivers[side].sendDrop()
        self.storedCommand[side] = None

    def regraspClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        position = float(self.widget.advanced.closePercentSpinner.value)
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendRegrasp(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def rateSpinnerChanged(self):
        self.updateTimer.targetFps = self.ui.repeatRateSpinner.value

    def fingerControlButton(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        if not self.ui.scissorControl.isChecked():
            self.drivers[side].sendFingerControl(int(self.ui.fingerAValue.text),
                                                 int(self.ui.fingerBValue.text),
                                                 int(self.ui.fingerCValue.text),
                                                 float(self.widget.advanced.forcePercentSpinner.value),
                                                 float(self.widget.advanced.velocityPercentSpinner.value),
                                                 None,
                                                 self.getModeInt(self.widget.advanced.modeBox.currentText))
        else:
            self.drivers[side].sendFingerControl(int(self.ui.fingerAValue.text),
                                                 int(self.ui.fingerBValue.text),
                                                 int(self.ui.fingerCValue.text),
                                                 float(self.widget.advanced.forcePercentSpinner.value),
                                                 float(self.widget.advanced.velocityPercentSpinner.value),
                                                 int(self.ui.scissorValue.text),
                                                 0)  # can ignore mode because scissor will override
        self.storedCommand[side] = None

    def updatePanel(self):

        if self.ui.repeaterCheckBox.checked and self.storedCommand['left']:
            position, force, velocity, mode = self.storedCommand['left']
            self.drivers['left'].sendCustom(position, force, velocity, mode)
        if self.ui.repeaterCheckBox.checked and self.storedCommand['right']:
            position, force, velocity, mode = self.storedCommand['right']
            self.drivers['right'].sendCustom(position, force, velocity, mode)
        if (self.ui.rightVisCheck.checked or self.ui.leftVisCheck.checked):
            self.doFTDraw();



def _getAction():
    return app.getToolBarActions()['ActionHandControlPanel']


def init(driverL, driverR, model, jc, view):

    global panel
    global dock

    panel = HandControlPanel(driverL, driverR, model, jc, view)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
