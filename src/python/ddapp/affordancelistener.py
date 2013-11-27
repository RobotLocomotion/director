from ddapp import lcmUtils
import drc as lcmdrc

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


class AffordanceServerListener(object):

    def __init__(self):
        self.subscriber = lcmUtils.addSubscriber('AFFORDANCE_COLLECTION', lcmdrc.affordance_collection_t, self.onAffordanceCollection)
        self.lastMsg = None
        self.affordances = []

    def onAffordanceCollection(self, msg):
        self.lastMsg = msg

        for aff in self.affordances:

            serverAff = findAffordanceByUid(msg.affs, aff.params.get('uid')) \
                        or findAffordanceByFriendlyName(msg.affs, aff.params.get('friendly_name'))
            if serverAff:
                aff.onServerAffordanceUpdate(serverAff)
            else:
                aff.params['uid'] = 0

    def registerAffordance(self, aff):
        if aff not in self.affordances:
            self.affordances.append(aff)

    def unregisterAffordance(self, aff):
        if aff in self.affordances:
            self.affordances.remove(aff)

listener = AffordanceServerListener()
