

class HandControlMacros(object):

    @staticmethod
    def openLeft():
        handcontrolpanel.panel.ui.leftButton.click()
        handcontrolpanel.panel.ui.openButton.animateClick()

    @staticmethod
    def openRight():
        handcontrolpanel.panel.ui.rightButton.click()
        handcontrolpanel.panel.ui.openButton.animateClick()

    @staticmethod
    def closeLeft():
        handcontrolpanel.panel.ui.leftButton.click()
        handcontrolpanel.panel.widget.advanced.modeBox.setCurrentIndex(0)
        handcontrolpanel.panel.ui.closeButton.animateClick()

    @staticmethod
    def closeRight():
        handcontrolpanel.panel.ui.rightButton.click()
        handcontrolpanel.panel.widget.advanced.modeBox.setCurrentIndex(0)
        handcontrolpanel.panel.ui.closeButton.animateClick()

    @staticmethod
    def pinchLeft():
        handcontrolpanel.panel.ui.leftButton.click()
        handcontrolpanel.panel.widget.advanced.modeBox.setCurrentIndex(2)
        handcontrolpanel.panel.ui.closeButton.animateClick()

    @staticmethod
    def pinchRight():
        handcontrolpanel.panel.ui.rightButton.click()
        handcontrolpanel.panel.widget.advanced.modeBox.setCurrentIndex(2)
        handcontrolpanel.panel.ui.closeButton.animateClick()



midiController = viewbehaviors.MidiBehaviorControl()
if midiController.reader:
    midiController.start()

midiController.callbacks.connect('s_button_7_pressed', HandControlMacros.openRight)
midiController.callbacks.connect('m_button_7_pressed', HandControlMacros.closeRight)
midiController.callbacks.connect('r_button_7_pressed', HandControlMacros.pinchRight)
midiController.callbacks.connect('s_button_6_pressed', HandControlMacros.openLeft)
midiController.callbacks.connect('m_button_6_pressed', HandControlMacros.closeLeft)
midiController.callbacks.connect('r_button_6_pressed', HandControlMacros.pinchLeft)


def reachWithLeft():
    teleoppanel.panel.endEffectorTeleop.setBaseConstraint('xyz only')
    teleoppanel.panel.endEffectorTeleop.setBackConstraint('limited')
    teleoppanel.panel.endEffectorTeleop.setLFootConstraint('fixed')
    teleoppanel.panel.endEffectorTeleop.setRFootConstraint('fixed')
    teleoppanel.panel.endEffectorTeleop.setLHandConstraint('ee fixed')
    teleoppanel.panel.endEffectorTeleop.setRHandConstraint('arm fixed')


def reachWithRight():
    teleoppanel.panel.endEffectorTeleop.setBaseConstraint('xyz only')
    teleoppanel.panel.endEffectorTeleop.setBackConstraint('limited')
    teleoppanel.panel.endEffectorTeleop.setLFootConstraint('fixed')
    teleoppanel.panel.endEffectorTeleop.setRFootConstraint('fixed')
    teleoppanel.panel.endEffectorTeleop.setLHandConstraint('arm fixed')
    teleoppanel.panel.endEffectorTeleop.setRHandConstraint('ee fixed')


#app.addToolbarMacro('left arm', reachWithLeft)
#app.addToolbarMacro('right arm', reachWithRight)
#app.addToolbarMacro('plan nominal', planNominal)
