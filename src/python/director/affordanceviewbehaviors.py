from director import affordanceitems
from director import viewbehaviors
from director import visualization as vis
import numpy as np


_affordanceManager = None


def setGlobalAffordanceManager(affordanceManager):
    global _affordanceManager
    _affordanceManager = affordanceManager


def getGlobalAffordanceManager():
    return _affordanceManager


def getActions(view, pickedObj, pickedPoint):

    affordanceObj = pickedObj if isinstance(pickedObj, affordanceitems.AffordanceItem) else None
    affordanceManager = getGlobalAffordanceManager()

    def addNewFrame():
        t = transformUtils.copyFrame(affordanceObj.getChildFrame().transform)
        t.PostMultiply()
        t.Translate(np.array(pickedPoint) - np.array(t.GetPosition()))
        newFrame = vis.showFrame(t, '%s frame %d' % (affordanceObj.getProperty('Name'), len(affordanceObj.children())), scale=0.2, parent=affordanceObj)
        affordanceObj.getChildFrame().getFrameSync().addFrame(newFrame, ignoreIncoming=True)

    def copyAffordance():
        desc = dict(affordanceObj.getDescription())
        del desc['uuid']
        desc['Name'] = desc['Name'] + ' copy'
        aff = affordanceManager.newAffordanceFromDescription(desc)
        aff.getChildFrame().setProperty('Edit', True)

    def onPromoteToAffordance():
        affObj = affordanceitems.MeshAffordanceItem.promotePolyDataItem(pickedObj)
        affordanceManager.registerAffordance(affObj)

    actions = []

    if affordanceManager and affordanceObj:
        actions.extend([
            ('Copy affordance', copyAffordance),
            ('Add new frame', addNewFrame),
        ])

    if type(pickedObj) == vis.PolyDataItem:
        actions.extend([
            ('Promote to Affordance', onPromoteToAffordance),
        ])

    return actions


def registerActions():
    viewbehaviors.registerContextMenuActions(getActions)
