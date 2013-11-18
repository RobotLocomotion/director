from ddapp import lcmUtils
from ddapp import transformUtils
import PythonQt
import bot_core as lcmbotcore

import subprocess
import select
import weakref


_icpSub = None
_icpCallbacks = []
_icpTransforms = []
_initialTransform = None
_registrarProc = None


def addICPCallback(func):
    #_icpCallbacks.append(weakref.ref(func))
    _icpCallbacks.append(func)


def storeInitialTransform():
    global _initialTransform
    _initialTransform = _icpTransforms[-1] if len(_icpTransforms) else None
    print 'stored initial icp transform'


def getInitialTransform():
    return _initialTransform


def onICPCorrection(messageData):

    messageClass = lcmbotcore.rigid_transform_t
    m = messageClass.decode(messageData.data())
    t = transformUtils.transformFromPose(m.trans, m.quat)

    _icpTransforms.append(t)

    print 'appending icp transform %d' % len(_icpTransforms)

    for func in _icpCallbacks:
        func(t)


def initICPCallback():

    global _icpSub
    lcmThread = lcmUtils.getGlobalLCMThread()
    _icpSub = PythonQt.dd.ddLCMSubscriber('MAP_LOCAL_CORRECTION', lcmThread)
    _icpSub.connect('messageReceived(const QByteArray&)', onICPCorrection)
    lcmThread.addSubscriber(_icpSub)


def _readAllSoFar(proc, retVal=''):
    while proc.poll() is None and (select.select([proc.stdout],[],[],0)[0] != []):
        retVal += proc.stdout.read(1)
    return retVal


def getRegistarOutput():

    global _registrarProc
    output = _readAllSoFar(_registrarProc)
    return output


def startMapsRegistrar():

    global _registrarProc
    _registrarProc = subprocess.Popen(['maps-registrar'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return _registrarProc
