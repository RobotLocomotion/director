from ddapp import lcmUtils
import drc as lcmdrc

class AffordanceServerListener(object):

    def __init__(self):
        self.subscriber = lcmUtils.addSubscriber('AFFORDANCE_COLLECTION', lcmdrc.affordance_collection_t, self.onAffordanceCollection)
        self.lastMsg = None

    def onAffordanceCollection(self, msg):
        self.lastMsg = msg


listener = AffordanceServerListener()
