from PyQt4 import QtCore, QtGui, uic

import lcm
import sys
import json
import time
import os
import drc as lcmdrc
import bot_core as lcmbotcore
import atlas
import functools

# allow control-c to kill the program
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


class LCMWrapper(object):
    '''
    A basic class provided some convenience methods around LCM.
    '''

    def __init__(self):
        self.lc = lcm.LCM()

    def subscribe(self, channel, callback):
        return self.lc.subscribe(channel, callback)

    def captureMessage(self, channel, messageClass):

        messages = []
        def handleMessage(channel, messageData):
            messages.append(messageClass.decode(messageData))

        subscription = self.subscribe(channel, handleMessage)

        while not messages:
            self.lc.handle()
            time.sleep(0.01)

        self.lc.unsubscribe(subscription)
        return messages[0]

    def publish(self, channel, message):
        self.lc.publish(channel, message.encode())


def captureRobotState():
    '''
    Blocks until a new LCM message is received on the EST_ROBOT_STATE channel,
    returns the new message.
    '''
    return lcmWrapper.captureMessage('EST_ROBOT_STATE', lcmbotcore.robot_state_t)


def capturePostureGoal():
    '''
    Blocks until a new LCM message is received on the POSTURE_GOAL channel,
    returns the new message.
    '''
    return lcmWrapper.captureMessage('POSTURE_GOAL', lcmdrc.joint_angles_t)


def capturePoseFromMessage(messageCaptureFunction):
    msg = messageCaptureFunction()
    joints = dict()
    for joint, position in zip(msg.joint_name, msg.joint_position):
        joints[joint] = position
    return joints


def getUtime():
    return int(time.time() * 1e6)


directorConfigFile = None
directorConfig = None

def setDirectorConfigFile(filename):
    global directorConfig, directorConfigFile
    directorConfigFile = filename
    directorConfig = None

def getDefaultDirectorConfigFile():
    return os.path.join(os.environ['DRC_BASE'], 'software/models/atlas_v5/director_config.json')

def getDirectorConfig():
    global directorConfig, directorConfigFile
    if directorConfig is None:

        if directorConfigFile is None:
            directorConfigFile = getDefaultDirectorConfigFile()

        with open(directorConfigFile) as configFile:
            directorConfig = json.load(configFile)
    return directorConfig


def getJointSets():
    '''
    Returns a dictionary of joint sets.
    '''

    config = getDirectorConfig()
    jointGroups = config['teleopJointGroups']

    jointSets = {}
    groups = ['left arm', 'right arm', 'back', 'left leg', 'right leg', 'base']
    for group in jointGroups:
        groupName = group['name'].lower()
        if groupName in groups:
            jointSets[groupName] = group['joints']

    return jointSets


def findPrefixInJointNames(jointNames, armJointList):
    for name in jointNames:
        if name in armJointList:
            return True
    return False

def getLeftArmInJoints(joints, jointGroups):
    for jointGroup in jointGroups:
        if jointGroup['name'] == 'Left Arm':
            return findPrefixInJointNames(joints.keys(), jointGroup['joints'])
    return False

def getRightArmInJoints(joints, jointGroups):
    for jointGroup in jointGroups:
        if jointGroup['name'] == 'Right Arm':
            return findPrefixInJointNames(joints.keys(), jointGroup['joints'])
    return False

def getJointNamesForPoseType(poseType):
    '''
    Returns a list of joint names for each part of the robot described in the
    poseType argument.  For example, if poseType is the string 'left arm, right arm',
    then the joint names for the left and right arms is returned.  Supported robot
    parts are left arm, right arm, back.
    '''
    jointSets = getJointSets()
    jointNames = []
    for name, jointSet in jointSets.iteritems():
        if name in poseType:
            jointNames += jointSet
    return jointNames


def updateComboStrings(combo, strings, defaultSelection):
    '''
    Clears the given combobox and then adds the given strings to the combo.
    Restores the combo's current value, or if the combo was empty, uses the
    string given in defaultSelection.
    '''
    currentText = str(combo.currentText()) if combo.count() else defaultSelection
    combo.clear()
    for text in strings:
        if not text:
            combo.insertSeparator(combo.count())
            continue

        combo.addItem(text)
        if text == currentText:
            combo.setCurrentIndex(combo.count() - 1)


def loadConfig(filename):
    '''
    Reads the contents of filename and parses it as a json document, returns
    the result as a dict.
    '''
    assert os.path.isfile(filename)
    with open(filename, 'r') as infile:
        return json.load(infile)


def saveConfig(config, filename):
    '''
    Overwrites the file at filename with a json string generated from the given
    config argument, a dict.
    '''
    with open(filename, 'w') as outfile:
        json.dump(config, outfile, indent=2, separators=(',', ': '), sort_keys=True)


def storePose(poseType, captureMethod, group, name, description, outFile):

    jointSet = getJointNamesForPoseType(poseType)
    assert len(jointSet)

    poseJoints = captureMethod['function']()

    joints = dict()
    for joint, position in poseJoints.iteritems():
        if joint in jointSet:
            joints[joint] = position

    posture = dict()
    posture['name'] = name
    posture['description'] = description
    posture['allow_mirror'] = True
    posture['joints'] = joints

    configFile = getDirectorConfig()
    jointGroups = configFile['teleopJointGroups']

    # determine a default value for nominal_handedness
    hasLeft = getLeftArmInJoints(joints, jointGroups)
    hasRight = getRightArmInJoints(joints, jointGroups)
    posture['nominal_handedness'] = 'none'
    if hasLeft != hasRight:
        posture['nominal_handedness'] = 'left' if hasLeft else 'right'

    config = loadConfig(outFile)
    postures = config.setdefault(group, [])

    for existingPosture in postures:
        if existingPosture['name'] == name:
            postures.remove(existingPosture)

    postures.append(posture)
    saveConfig(config, outFile)


def applyMirror(joints):
    '''
    joints is a dict where the keys are joint name strings and the values are
    joint positions.  This function renames left arm and right arm joints
    and flips the sign of the joint position as required, and also flips the sign
    on back_bkz. Note that other back joints are not modified by this function.
    Returns the result as a new dictionary in the same format.
    '''

    def toLeft(jointName):
        '''
        If the right arm joint can be found in the list, insert the left
        hand instead. this assumes that the corresponding joint is in the
        same position in each list
        '''
        if jointName in rightArmJointList:
            return leftArmJointList[rightArmJointList.index(jointName)]
        return jointName

    def toRight(jointName):
        if jointName in leftArmJointList:
            return rightArmJointList[leftArmJointList.index(jointName)]
        return jointName

    def flipLeftRight(jointName):
        if jointName in leftArmJointList:
            return toRight(jointName)
        else:
            return toLeft(jointName)

    signFlips = getDirectorConfig()['mirrorJointSignFlips']

    configFile = getDirectorConfig()
    jointGroups = configFile['teleopJointGroups']
    leftArmJointList = filter(lambda thisJointGroup: thisJointGroup['name'] == 'Left Arm', jointGroups)[0]['joints']
    rightArmJointList = filter(lambda thisJointGroup: thisJointGroup['name'] == 'Right Arm', jointGroups)[0]['joints']

    flipped = {}
    for name, position in joints.iteritems():
        name = flipLeftRight(name)
        if name in signFlips:
            position = -position

        flipped[name] = position
    return flipped


def publishPostureGoal(joints, postureName, channel='POSTURE_GOAL'):
    '''
    Given a dict mapping joint name strings to joint positions, creates a
    joint_angles_t LCM message and publishes the result on the given channel name.
    '''
    msg = lcmdrc.joint_angles_t()
    msg.utime = getUtime()
    for name, position in joints.iteritems():
        msg.joint_name.append(name)
        msg.joint_position.append(position)
    msg.num_joints = len(msg.joint_name)
    lcmWrapper.publish(channel, msg)
    lcmWrapper.publish('POSTURE_GOAL_CANNED', msg)

    publishSystemStatus('sending posture goal: ' + postureName)


def publishTrajGoal(name, channel=''):

    msg = lcmdrc.behavior_command_t()
    msg.utime = getUtime()
    msg.command = name
    lcmWrapper.publish('EE_TRAJ_GOAL', msg)

    publishSystemStatus('sending EE traj goal: ' + name)


def publishSystemStatus(text):

    msg = lcmbotcore.system_status_t()
    msg.utime = getUtime()
    msg.system = 5
    msg.importance = 0
    msg.frequency = 0
    msg.value = text
    lcmWrapper.publish('SYSTEM_STATUS', msg)


class SendPosturePanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.selectedPosture = None
        self.setup()


    def setup(self):
        self.ui.postureFilter.hide()
        self.ui.postureFilterLabel.hide()
        self.ui.connect(self.ui.sendLeftButton, QtCore.SIGNAL('clicked()'), self.onLeftClicked)
        self.ui.connect(self.ui.sendRightButton, QtCore.SIGNAL('clicked()'), self.onRightClicked)
        self.ui.connect(self.ui.sendDefaultButton, QtCore.SIGNAL('clicked()'), self.onDefaultClicked)
        self.ui.connect(self.ui.sendPostureGroupCombo, QtCore.SIGNAL('currentIndexChanged(const QString&)'), self.onGroupComboChanged)
        self.ui.connect(self.ui.postureListWidget, QtCore.SIGNAL('currentRowChanged(int)'), self.onPostureSelected)
        self.updateGroupCombo()
        self.updatePostureListWidget()


    def updateGroupCombo(self):
        groupNames = self.ui.getGroupNames()

        try:
            groupNames.remove('General')
        except ValueError:
            pass

        groupNames.insert(0, '')
        groupNames.insert(0, 'General')
        groupNames.insert(0, 'All')
        self.ui.sendPostureGroupCombo.blockSignals(True)
        updateComboStrings(self.ui.sendPostureGroupCombo, groupNames, 'All')
        self.ui.sendPostureGroupCombo.blockSignals(False)

    def setSelectedGroup(self, groupName):
        index = self.ui.sendPostureGroupCombo.findText(groupName)
        if index < 0: index = 0
        self.ui.sendPostureGroupCombo.setCurrentIndex(index)


    def onGroupComboChanged(self):
        self.updatePostureListWidget()

    def getSelectedGroup(self):
        return str(self.ui.sendPostureGroupCombo.currentText())

    def updatePostureListWidget(self):
        groupName = self.getSelectedGroup()
        self.currentPostures = self.ui.getPosturesInGroup(groupName)

        self.ui.postureListWidget.blockSignals(True)
        self.ui.postureListWidget.clear()
        for posture in self.currentPostures:
            self.ui.postureListWidget.addItem(posture['name'])
        self.ui.postureListWidget.setCurrentRow(0)
        self.ui.postureListWidget.blockSignals(False)

        self.onPostureSelected()

    def getSelectedPosture(self):
        currentItem = self.ui.postureListWidget.currentItem()
        if not currentItem:
            return None

        postureName = str(currentItem.text())
        for posture in self.currentPostures:
            if posture['name'] == postureName:
                return posture


    def getPostureCanBeMirrored(self, posture):
        return posture['allow_mirror'] and self.getNominalHandedness(posture) in ('left', 'right')

    def getNominalHandedness(self, posture):
        handedness = posture['nominal_handedness']
        assert handedness in ('left', 'right', 'none')
        return handedness

    def onPostureSelected(self):

        self.selectedPosture = self.getSelectedPosture()
        self.updateDescriptionLabel()

        self.ui.sendDefaultButton.setVisible(False)
        self.ui.sendLeftButton.setVisible(True)
        self.ui.sendRightButton.setVisible(True)
        self.ui.sendDefaultButton.setEnabled(False)
        self.ui.sendLeftButton.setEnabled(False)
        self.ui.sendRightButton.setEnabled(False)
        if not self.selectedPosture:
            return

        if self.getPostureCanBeMirrored(self.selectedPosture):
            self.ui.sendLeftButton.setEnabled(True)
            self.ui.sendRightButton.setEnabled(True)
        else:
            self.ui.sendLeftButton.setVisible(False)
            self.ui.sendRightButton.setVisible(False)
            self.ui.sendDefaultButton.setVisible(True)
            self.ui.sendDefaultButton.setEnabled(True)


    def updateDescriptionLabel(self):
        description = self.selectedPosture['description'] if self.selectedPosture else 'none'
        self.ui.descriptionLabel.setText('Description: ' + str(description))


    def onGroupsChanged(self):
        self.updateGroupCombo()

    def onPostureAdded():
        self.updatePostureListWidget()

    def onLeftClicked(self):
        joints = self.selectedPosture['joints']
        if self.getNominalHandedness(self.selectedPosture) == 'right':
            joints = applyMirror(joints)
        publishPostureGoal(joints, self.selectedPosture['name'] + ' left')

    def onRightClicked(self):
        joints = self.selectedPosture['joints']
        if self.getNominalHandedness(self.selectedPosture) == 'left':
            joints = applyMirror(joints)
        publishPostureGoal(joints, self.selectedPosture['name'] + ' right')

    def onDefaultClicked(self):
        joints = self.selectedPosture['joints']
        publishPostureGoal(joints, self.selectedPosture['name'] + ' default')

    def saveSettings(self, settings):
        settings.setValue('sendPose/currentGroup', self.getSelectedGroup())

    def restoreSettings(self, settings):
        self.setSelectedGroup(settings.value('sendPose/currentGroup', 'All').toString())


class SendEETrajPanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.selectedTraj = None
        self.setup()


    def setup(self):
        self.ui.sendTrajDefaultButton.setVisible(False)
        self.ui.connect(self.ui.sendTrajLeftButton, QtCore.SIGNAL('clicked()'), self.onLeftClicked)
        self.ui.connect(self.ui.sendTrajRightButton, QtCore.SIGNAL('clicked()'), self.onRightClicked)
        self.ui.connect(self.ui.sendTrajDefaultButton, QtCore.SIGNAL('clicked()'), self.onDefaultClicked)
        #self.ui.connect(self.ui.trajListWidget, QtCore.SIGNAL('currentRowChanged(int)'), self.onTrajSelected)
        self.updateListWidget()

    def updateListWidget(self):

        trajList = ['ladder_reach_up']

        self.ui.trajListWidget.blockSignals(True)
        self.ui.trajListWidget.clear()
        for name in sorted(trajList):
            self.ui.trajListWidget.addItem(name)
        self.ui.trajListWidget.setCurrentRow(0)
        self.ui.trajListWidget.blockSignals(False)

        #self.onTrajSelected()

    def getSelectedTraj(self):
        currentItem = self.ui.trajListWidget.currentItem()
        if not currentItem:
            return None

        name = str(currentItem.text())
        return name

    def onLeftClicked(self):
        traj = self.getSelectedTraj()
        publishTrajGoal(traj + ':LEFT')

    def onRightClicked(self):
        traj = self.getSelectedTraj()
        publishTrajGoal(traj + ':RIGHT')

    def onDefaultClicked(self):
        traj = self.getSelectedTraj()
        publishTrajGoal(traj + ':DEFAULT')

    def saveSettings(self, settings):
        pass

    def restoreSettings(self, settings):
        pass


class CapturePanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.captureMethods = []
        self.setup()


    def setup(self):
        self.ui.connect(self.ui.captureButton, QtCore.SIGNAL('clicked()'), self.onCaptureClicked)
        self.ui.connect(self.ui.groupCombo, QtCore.SIGNAL('currentIndexChanged(const QString&)'), self.onGroupComboChanged)
        self.updateGroupCombo()
        self.initCaptureMethods()


    def updateGroupCombo(self):
        groupNames = self.ui.getGroupNames()
        try:
            groupNames.remove('General')
        except ValueError:
            pass

        groupNames.insert(0, '')
        groupNames.insert(0, 'General')
        groupNames.append('')
        groupNames.append('New group...')
        self.ui.groupCombo.blockSignals(True)
        updateComboStrings(self.ui.groupCombo, groupNames, 'General')
        self.ui.groupCombo.blockSignals(False)

    def setSelectedGroup(self, groupName):
        index = self.ui.groupCombo.findText(groupName)
        if index < 0: index = 0
        self.ui.groupCombo.setCurrentIndex(index)

    def getSelectedGroup(self):
        return str(self.ui.groupCombo.currentText())

    def onGroupComboChanged(self):

        if str(self.ui.groupCombo.currentText()) == 'New group...':
            groupName = self.ui.messageBoxInput('Enter new group name', 'Group name:')
            if not groupName or groupName == '':
                self.setSelectedGroup('General')
                return

            groupName = str(groupName)

            self.ui.addNewGroup(groupName)
            self.setSelectedGroup(groupName)

    def onGroupsChanged(self):
        self.updateGroupCombo()

    def saveSettings(self, settings):
        settings.setValue('sendPose/currentGroup', self.getSelectedGroup())

    def restoreSettings(self, settings):
        self.setSelectedGroup(settings.value('capturePanel/currentGroup', 'General').toString())


    def initCaptureMethods(self):
        self.addCaptureMethod('EST_ROBOT_STATE lcm channel', functools.partial(capturePoseFromMessage, captureRobotState))

    def getCaptureMethod(self, name):
        for method in self.captureMethods:
            if method['name'] == name:
                return method

    def addCaptureMethod(self, name, function):
        if self.getCaptureMethod(name):
            raise Exception('Refusing to re-add capture method: %s' % name)

        self.captureMethods.append(dict(name=name, function=function))
        captureNames = [method['name'] for method in self.captureMethods]
        updateComboStrings(self.ui.captureChannelCombo, captureNames, self.captureMethods[0]['name'])

    def onCaptureClicked(self):

        captureMethod = self.getCaptureMethod(self.ui.captureChannelCombo.currentText())
        group = str(self.ui.groupCombo.currentText())
        name = str(self.ui.nameEdit.text())
        description = str(self.ui.descriptionEdit.text())
        poseType = str(self.ui.jointSetCombo.currentText())
        outFile = self.ui.getPoseConfigFile()

        if not name:
            self.ui.showWarning('Empty field', 'Please enter a name into the text box.')
            return

        if not description:
            self.ui.showWarning('Empty field', 'Please enter a description into the text box.')
            return

        existingPostures = self.ui.getPosturesInGroup(group)
        for posture in existingPostures:
            if posture['name'] == name:
                reply = self.ui.showQuestion('Overwrite posture?', 'Posture with name "%s" already exists.\nDo you want to overwrite?' % name,
                                                  QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)

                if reply == QtGui.QMessageBox.No:
                    return

        storePose(poseType, captureMethod, group, name, description, outFile)
        self.ui.onPostureAdded()



class MainWindow(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)
        uifile = QtCore.QFile(':/ui/ddRobotPoseGui.ui')
        isOpen = uifile.open(uifile.ReadOnly)
        if not isOpen:
            uifile = os.path.join(os.environ['DRC_BASE'], 'software/director/src/app/ddRobotPoseGui.ui')
            assert os.path.isfile(uifile)

        uic.loadUi(uifile, self)

        self.setWindowTitle('Robot Pose Utility')

        if not self.checkEnvironment():
            return

        assert directorConfigFile is not None
        self.configFile = os.path.join(os.path.dirname(directorConfigFile), getDirectorConfig()['postureDatabaseFile'])
        if not self.checkConfigFile():
            return

        self.setup()
        self.restoreSettings()
        self.messageBoxWarning = functools.partial(QtGui.QMessageBox.warning, self)
        self.messageBoxQuestion = functools.partial(QtGui.QMessageBox.question, self)

    def setup(self):
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+W'), self), QtCore.SIGNAL('activated()'), self.close)
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+Q'), self), QtCore.SIGNAL('activated()'), self.close)
        self.capturePanel = CapturePanel(self)
        self.sendPosturePanel = SendPosturePanel(self)
        self.sendTrajPanel = SendEETrajPanel(self)

    def showWarning(self, title, message):
        return self.messageBoxWarning(title, message)

    def showQuestion(self, title, message, buttons, defaultButton):
        return self.messageBoxQuestion(title, message, buttons, defaultButton)

    def showInput(self, title, message):
        return self.messageBoxInput(title, message)

    def getSettings(self):
        return QtCore.QSettings('mitdrc', 'RobotPoseGUI')

    def saveSettings(self):
        settings = self.getSettings()
        settings.setValue('currentTabIndex', self.tabWidget.currentIndex())
        self.capturePanel.saveSettings(settings)
        self.sendPosturePanel.saveSettings(settings)

    def restoreSettings(self):
        settings = self.getSettings()
        self.tabWidget.setCurrentIndex(settings.value('currentTabIndex', 0).toInt()[0])
        self.capturePanel.restoreSettings(settings)
        self.sendPosturePanel.restoreSettings(settings)


    def closeEvent(self, event):
        self.saveSettings()

    def checkEnvironment(self):
        if not os.path.isdir(os.environ['DRC_BASE']):
            self.showWarning('Environment not set', 'Error loading configuration.  DRC_BASE environment variable is not set to a valid directory.')
            self.setEnabled(False)
            return False
        return True

    def checkConfigFile(self):

        configFile = self.getPoseConfigFile()
        if not os.path.isfile(configFile):
            self.showWarning('Config file not found', 'Config file not found: %s' % configFile)
            self.setEnabled(False)
            return False

        json.load(open(configFile, 'r'))

        return True

    def getPoseConfigFile(self):
        return self.configFile

    def loadConfigFile(self):
        if not self.checkConfigFile():
            return
        config = json.load(open(self.getPoseConfigFile(), 'r'))
        if not self.checkPostures(config):
            return {}
        return config

    def getGroupNames(self):
        if not self.checkConfigFile():
            return []
        return sorted(self.loadConfigFile().keys())


    def checkPostures(self, config):
        for groupName, postures in config.iteritems():
            for i, posture in enumerate(postures):
                for name in ['name', 'description', 'joints', 'nominal_handedness']:
                    if name not in posture:
                        self.ui.showWarning('Format error', 'Format error in posture %d of group "%s".  Missing attribute: "%s".' % (i, groupName, name))
                        self.currentConfig = {}
                        return False
        return True


    def getPosturesInGroup(self, groupName):

        config = self.loadConfigFile()

        postures = []
        if groupName == 'All':
            for group, postureList in config.iteritems():
                for posture in postureList:
                    posture['name'] = '%s - %s' % (group, posture['name'])
                postures += postureList
        else:
            postures = config.get(groupName, [])

        return sorted(postures, key=lambda x: x['name'])


    def addNewGroup(self, groupName):
        config = self.loadConfigFile()
        config.setdefault(groupName, [])
        saveConfig(config, self.getPoseConfigFile())

        self.capturePanel.onGroupsChanged()
        self.sendPosturePanel.onGroupsChanged()

    def onPostureAdded(self):
        self.sendPosturePanel.updatePostureListWidget()




def main():


    # create a global instance of the LCMWrapper
    global lcmWrapper
    lcmWrapper = LCMWrapper()

    try:
        configFile = os.path.abspath(sys.argv[1])
        setDirectorConfigFile(configFile)
    except IndexError:
        setDirectorConfigFile(getDefaultDirectorConfigFile())

    # start the application
    app = QtGui.QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()
