from ddapp import lcmUtils
import drc as lcmdrc

import ddapp.objectmodel as om
from ddapp import affordance

def findAffordanceByUid(affs, uid):
    '''
    Searches a list of affordance messages for an affordance with the given uid.
    '''
    if not uid:
        return None
    for aff in affs:
        if aff.uid == uid:
            return aff

def findAffordanceByFriendlyName(affs, friendlyName):
    '''
    Searches a list of affordance messages for an affordance with the given friendly_name.
    '''
    if not friendlyName:
        return None
    for aff in affs:
        if aff.friendly_name == friendlyName:
            return aff


class ServerAffordanceItem(om.ObjectModelItem):

    def __init__(self, name, message):

        om.ObjectModelItem.__init__(self, name, om.Icons.Robot)

        self.message = message
        self.addProperty('uid', message.uid)
        self.addProperty('otdf_type', message.otdf_type)
        #self.addProperty('origin_xyz', message.origin_xyz)
        #self.addProperty('origin_rpy', message.origin_rpy)

    def _onPropertyChanged(self, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertyName)

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'uid':
            return om.PropertyAttributes(decimals=0, minimum=0, maximum=1e6, singleStep=1, hidden=False)
        elif propertyName in ('origin_xyz', 'origin_rpy'):
            return om.PropertyAttributes(decimals=3, minimum=-1e6, maximum=1e6, singleStep=0.01, hidden=False)
        else:
            return om.ObjectModelItem.getPropertyAttributes(self, propertyName)

    def onRemoveFromObjectModel(self):
        if self.message:
            affordance.deleteAffordance(self.message)


class AffordanceServerListener(object):

    def __init__(self):
        self.subscriber = lcmUtils.addSubscriber('AFFORDANCE_COLLECTION', lcmdrc.affordance_collection_t, self.onAffordanceCollection)
        self.lastMsg = None
        self.affordances = []
        self.serverAffs = {}

    def onAffordanceCollection(self, msg):
        self.lastMsg = msg

        for aff in self.affordances:

            serverAff = findAffordanceByUid(msg.affs, aff.params.get('uid')) \
                        or findAffordanceByFriendlyName(msg.affs, aff.params.get('friendly_name'))
            if serverAff:
                aff.onServerAffordanceUpdate(serverAff)
            else:
                aff.params['uid'] = 0
                aff.setProperty('uid', 0)

        affsToDelete = self.serverAffs.keys()
        for aff in msg.affs:
            affName = '%s_%r' % (aff.otdf_type, aff.uid)
            serverAff = self.serverAffs.get(affName)
            if not serverAff:
                serverAff = ServerAffordanceItem(affName, aff)
                om.addToObjectModel(serverAff, om.getOrCreateContainer('server affordances'))
                self.serverAffs[affName] = serverAff
            else:
                affsToDelete.remove(affName)
        for affName in affsToDelete:
            aff = self.serverAffs.pop(affName)
            aff.message = None
            om.removeFromObjectModel(aff)

    def registerAffordance(self, aff):
        if aff not in self.affordances:
            self.affordances.append(aff)

    def unregisterAffordance(self, aff):
        if aff in self.affordances:
            self.affordances.remove(aff)

listener = AffordanceServerListener()
