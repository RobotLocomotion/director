import time
import pprint
import uuid
import json
from collections import OrderedDict
from director.thirdparty import numpyjsoncoder
from director import callbacks
from director.utime import getUtime
from director.uuidutil import newUUID

try:
    import bot_core as lcmbotcore
    USE_LCM = True
except ImportError:
    USE_LCM = False

if USE_LCM:
    from director import lcmUtils


class LCMObjectCollection(object):

    DESCRIPTION_UPDATED_SIGNAL = 'DESCRIPTION_UPDATED_SIGNAL'
    DESCRIPTION_REMOVED_SIGNAL = 'DESCRIPTION_REMOVED_SIGNAL'

    def __init__(self, channel):
        self.collection = OrderedDict()
        self.collectionId = newUUID()
        self.sentCommands = set()
        self.sentRequest = None
        self.channel = channel
        self.callbacks = callbacks.CallbackRegistry([self.DESCRIPTION_UPDATED_SIGNAL,
                                                     self.DESCRIPTION_REMOVED_SIGNAL])
        self.sub = None
        self._modified()

        if USE_LCM:
            self.sub = lcmUtils.addSubscriber(self.channel, messageClass=lcmbotcore.system_status_t, callback=self._onCommandMessage)
            self.sub.setNotifyAllMessagesEnabled(True)

    def __del__(self):
        if self.sub:
            lcmUtils.removeSubscriber(self.sub)

    def connectDescriptionUpdated(self, func):
        return self.callbacks.connect(self.DESCRIPTION_UPDATED_SIGNAL, func)

    def disconnectDescriptionUpdated(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectDescriptionRemoved(self, func):
        return self.callbacks.connect(self.DESCRIPTION_REMOVED_SIGNAL, func)

    def disconnectDescriptionRemoved(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def getDescriptionId(self, desc):
        return desc['uuid']

    def prettyPrintCollection(self):
        print(json.dumps(json.loads(numpyjsoncoder.encode(self.collection)), indent=2))

    def getDescription(self, descriptionId):
        return self.collection[descriptionId]

    def updateDescription(self, desc, publish=True, notify=True):
        self.collection[self.getDescriptionId(desc)] = desc
        self._modified()
        if publish and USE_LCM:
            msg = self._newCommandMessage('update', description=desc)
            lcmUtils.publish(self.channel, msg)

        if notify:
            self.callbacks.process(self.DESCRIPTION_UPDATED_SIGNAL, self, self.getDescriptionId(desc))

    def removeDescription(self, descriptionId, publish=True, notify=True):

        try:
            del self.collection[descriptionId]
            self._modified()
        except KeyError:
            pass

        if publish and USE_LCM:
            msg = self._newCommandMessage('remove', descriptionId=descriptionId,)
            lcmUtils.publish(self.channel, msg)

        if notify:
            self.callbacks.process(self.DESCRIPTION_REMOVED_SIGNAL, self, descriptionId)

    def sendEchoRequest(self):
        self.sentRequest = newUUID()
        msg = self._newCommandMessage('echo_request', requestId=self.sentRequest)
        lcmUtils.publish(self.channel, msg)


    def sendEchoResponse(self, requestId=None):
        if requestId is None:
            requestId = newUUID()
        msg = self._newCommandMessage('echo_response', requestId=requestId, descriptions=self.collection)
        lcmUtils.publish(self.channel, msg)

    def handleEchoResponse(self, data):
        #if data['requestId'] != self.sentRequest:
        #    return

        self.sentRequest = None
        for desc in list(data['descriptions'].values()):
            self.updateDescription(desc, publish=False)

    def _modified(self):
        self.mtime = getUtime()

    def _newCommandMessage(self, commandName, **commandArgs):
        commandId = newUUID()
        self.sentCommands.add(commandId)
        commandArgs['commandId'] = commandId
        commandArgs['collectionId'] = self.collectionId
        commandArgs['command'] = commandName
        msg = lcmbotcore.system_status_t()
        msg.value = numpyjsoncoder.encode(commandArgs)
        msg.utime = getUtime()
        return msg

    def _onCommandMessage(self, msg):

        data = numpyjsoncoder.decode(msg.value)

        commandId = data['commandId']
        if commandId in self.sentCommands:
            self.sentCommands.remove(commandId)
            return

        command = data['command']

        if command == 'update':
            desc = data['description']
            self.updateDescription(desc, publish=False)

        elif command == 'remove':
            self.removeDescription(data['descriptionId'], publish=False)

        elif command == 'echo_request':
            self.sendEchoResponse(data['requestId'])

        elif command == 'echo_response':
            self.handleEchoResponse(data)
