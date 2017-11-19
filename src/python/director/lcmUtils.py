import lcm
import PythonQt
import datetime
import pickle
import atexit
import socket
import os
import subprocess
import tempfile
import imp
import sys
import re

class GlobalLCM(object):

  _handle = None
  _lcmThread = None

  @classmethod
  def get(cls):
      if cls._handle == None:
          cls._handle = lcm.LCM()
      return cls._handle

  @classmethod
  def getThread(cls):
      if cls._lcmThread == None:
          cls._lcmThread = PythonQt.dd.ddLCMThread()
          atexit.register(cls.finalize)
          cls._lcmThread.start()
      return cls._lcmThread

  @classmethod
  def finalize(cls):
      if cls._lcmThread:
          cls._lcmThread.delete()
          cls._lcmThread = None


def getGlobalLCM():
    return GlobalLCM.get()


def getGlobalLCMThread():
    return GlobalLCM.getThread()


def captureMessage(channel, messageClass, lcmHandle=None):

    lcmHandle = lcmHandle or getGlobalLCM()

    messages = []
    def handleMessage(channel, messageData):
        messages.append(messageClass.decode(messageData))

    subscription = lcmHandle.subscribe(channel, handleMessage)

    while not messages:
        lcmHandle.handle()

    lcmHandle.unsubscribe(subscription)
    return messages[0]


def captureMessageAsync(channel, messageClass):

    lcmThread = getGlobalLCMThread()
    sub = PythonQt.dd.ddLCMSubscriber(channel)

    messages = []
    def handleMessage(messageData):
        messages.append(messageClass.decode(messageData.data()))
        lcmThread.removeSubscriber(sub)

    sub.connect('messageReceived(const QByteArray&, const QString&)', handleMessage)
    lcmThread.addSubscriber(sub)

    while not messages:
        yield None

    yield messages[0]


def captureMessageCallback(channel, messageClass, callback):

    subscriber = None
    messages = []
    def handleMessage(message):
        getGlobalLCMThread().removeSubscriber(subscriber)
        if not messages:
            messages.append(True)
            callback(message)

    subscriber = addSubscriber(channel, messageClass, handleMessage)
    return subscriber


def addSubscriber(channel, messageClass=None, callback=None, historicalLoader=None, callbackNeedsChannel=False):

    lcmThread = getGlobalLCMThread()
    subscriber = PythonQt.dd.ddLCMSubscriber(channel, lcmThread)

    def handleMessage(messageData, channel):
        loadSuccessful = False
        try:
            msg = messageClass.decode(messageData.data())
            loadSuccessful = True
        except ValueError:
            if historicalLoader is not None:
                # try:
                    msg = historicalLoader.decode(messageClass.__module__.split('.')[-1], messageData.data())
                    loadSuccessful = True
                # except ValueError:
                #     pass
        if loadSuccessful:
            if callbackNeedsChannel:
                callback(msg, channel=channel)
            else:
                callback(msg)
        else:
            print('error decoding message on channel:', channel)

    if callback is not None:
        if messageClass is not None:
            subscriber.connect('messageReceived(const QByteArray&, const QString&)', handleMessage)
        else:
            subscriber.connect('messageReceived(const QByteArray&, const QString&)', callback)
    else:
        subscriber.setCallbackEnabled(False)

    lcmThread.addSubscriber(subscriber)
    return subscriber


def removeSubscriber(subscriber):
    lcmThread = getGlobalLCMThread()
    lcmThread.removeSubscriber(subscriber)
    if subscriber.parent() == lcmThread:
        subscriber.setParent(None)


def getNextMessage(subscriber, messageClass=None, timeout=0):

    messageData = subscriber.getNextMessage(timeout).data()

    if not messageData:
        return None

    if messageClass is not None:
        try:
            msg = messageClass.decode(messageData)
        except ValueError:
            print('error decoding message on channel:', subscriber.channel())
            return None

        return msg
    else:
        return messageData


class MessageResponseHelper(object):

    def __init__(self, responseChannel, responseMessageClass=None):
        self.subscriber = addSubscriber(responseChannel)
        self.messageClass = responseMessageClass

    def waitForResponse(self, timeout=5000, keepAlive=True):
        response = getNextMessage(self.subscriber, self.messageClass, timeout)
        if not keepAlive:
            self.finish()
        return response

    def finish(self):
        removeSubscriber(self.subscriber)

    @staticmethod
    def publishAndWait(channel, message, responseChannel, responseMessageClass=None, timeout=5000):
        helper = MessageResponseHelper(responseChannel, responseMessageClass)
        publish(channel, message)
        return helper.waitForResponse(timeout, keepAlive=False)


def publish(channel, message):
    getGlobalLCM().publish(channel, message.encode())


def dumpMessage(msg, filename):
    pickle.dump((type(msg), msg.encode()), open(filename, 'w'))


def loadMessage(filename):
    cls, bytes = pickle.load(open(filename))
    return cls.decode(bytes)


class MessageCollector(object):

    def __init__(self, channel, messageClass=None):
        self.messages = []
        self.channel = channel
        self.messageClass = messageClass
        self.subscriber = None
        self.start()

    def __del__(self):
        self.stop()

    def start(self):
        if not self.subscriber:
            self.subscriber = addSubscriber(self.channel, messageClass=self.messageClass, callback=self.onMessage)

    def stop(self):
        if self.subscriber:
            removeSubscriber(self.subscriber)
            self.subscriber = None

    def clear(self):
        del self.messages[:]

    def onMessage(self, msg):
        self.messages.append(msg)


class LogPlayerCommander(object):

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.logPlayerIp = '127.0.0.1'
        self.logPlayerPort = 53261

    def sendCommand(self, msg):
        self.socket.sendto(msg, (self.logPlayerIp, self.logPlayerPort))

    def togglePlay(self):
        self.sendCommand('PLAYPAUSETOGGLE')

    def step(self):
        self.sendCommand('STEP')

    def faster(self):
        self.sendCommand('FASTER')

    def slower(self):
        self.sendCommand('SLOWER')

    def back(self):
        self.sendCommand('BACK5')

    def forward(self):
        self.sendCommand('FORWARD5')


class TypeNotFoundError(Exception):
    pass

class HistoricalLCMLoader(object):
    """
    A helper class which can be added to a call to addSubscriber in order to allow the subscriber to decode messages which were generated with an older version of the LCM type definitions.
    """
    def __init__(self, package_name, lcmtypes_path, repo_path):
        self.package_name = package_name
        self.lcmtypes_path = lcmtypes_path
        self.repo_path = repo_path
        self.type_cache = {}
        self._mru_shas_cache = {}
        self._initialized = False
        self._build_dir = None
        self._tmpdir = None
        self._source_dir = None

    @property
    def build_dir(self):
        if self._build_dir is None:
            self._build_dir = os.path.join(self.tmpdir, 'build')
            if not os.path.exists(self._build_dir):
                os.mkdir(self._build_dir)
        return self._build_dir

    @property
    def source_dir(self):
        if self._source_dir is None:
            self._source_dir = os.path.join(self.tmpdir, 'source')
            if not os.path.exists(self._source_dir):
                os.mkdir(self._source_dir)
        return self._source_dir

    @property
    def tmpdir(self):
        if self._tmpdir is None:
            self._tmpdir = os.path.join(tempfile.gettempdir(), 'temporary_lcmtypes')
            if not os.path.exists(self._tmpdir):
                os.mkdir(self._tmpdir)
        return self._tmpdir

    def buildTypeAtSHA(self, type_name, sha):
        """
        Build the python source files for the given type and revision. We rename the python module from its default (which is just the LCM package name) to [packagename][sha] to prevent namespace conflicts.
        """
        source_files = self.getOrCreateSourceFiles(type_name, sha, recursive=True)
        sha_source_dir = os.path.dirname(source_files[0])
        sha_build_dir = os.path.join(self.build_dir, sha)
        if not os.path.exists(sha_build_dir):
            os.makedirs(sha_build_dir)
        final_pkg_dir = os.path.join(sha_build_dir, self.package_name + str(sha))
        if not os.path.exists(final_pkg_dir):
            os.makedirs(final_pkg_dir)
        subprocess.check_call("lcm-gen --lazy -p --ppath {build:s} {source:s}".format(
                                      build=sha_build_dir,
                                      source=os.path.join(sha_source_dir, '*')),
                              shell=True)
        build_files = [f for f in os.listdir(os.path.join(sha_build_dir, self.package_name))
                       if f.endswith('.py')]
        build_type_names = [t.replace('.py', '') for t in build_files]
        for f in build_files:
            subprocess.check_call(r"perl -ne 's/{pkg:s}(?=\.({type_list:s}[^a-zA-Z0-9_]))/{pkg:s}{sha:s}/g; print;' < {infile:s} > {outfile:s}".format(
                    pkg=self.package_name,
                    type_list = '|'.join(build_type_names),
                    sha=sha,
                    infile=os.path.join(sha_build_dir, self.package_name, f),
                    outfile=os.path.join(final_pkg_dir, f)),
                                      shell=True)

    def getOrCreateBuildFile(self, type_name, sha):
        fname = type_name + '.py'
        target = os.path.join(self.build_dir, sha, self.package_name + str(sha), fname)
        if not os.path.exists(target):
            self.buildTypeAtSHA(type_name, sha)
        return target

    def getOrCreateSourceFiles(self, type_name, sha, recursive=False):
        """
        Find the LCM source files for the given type at the given revision, pulling them out of the git history as needed. Also finds the source files for the children of that type if recursive=True.
        """
        fname = self.package_name + '_' + type_name + '.lcm'
        source_dir = os.path.join(self.source_dir, sha)
        if not os.path.exists(source_dir):
            os.makedirs(source_dir)
        targets = [os.path.join(self.source_dir, sha, fname)]
        if not os.path.exists(targets[0]):
            try:
                subprocess.check_call("git -C {base:s} show {sha:s}:{typepath:s} > {fpath:s}".format(
                                        base=self.repo_path, sha=sha,
                                        typepath=os.path.join(self.lcmtypes_path, fname),
                                        fpath=targets[0]),
                                     shell=True)
            except subprocess.CalledProcessError:
                raise TypeNotFoundError("The target LCMtype cannot be found at this revision")
        if recursive:
            for child in self.getChildTypes(type_name, sha):
                targets.extend(self.getOrCreateSourceFiles(child, sha, recursive=True))
        return targets

    def getChildTypes(self, type_name, sha):
        """
        Find the children of a given type by parsing the output of lcm-gen -d
        """
        source_file = self.getOrCreateSourceFiles(type_name, sha)[0]
        children = []
        debug_data = subprocess.check_output("lcm-gen -d {fpath:s}".format(fpath=source_file),
                                             shell=True)
        debug_lines = debug_data.split('\n')
        for line in debug_lines:
            line = line.lstrip()
            match = re.match(r"{pkg:s}\.(?P<childname>[^\s]+)".format(pkg=self.package_name), line)
            if match:
                child = (match.groupdict()['childname'])
                if child != type_name:
                    children.append(child)
        return children

    def getSHAsForType(self, type_name):
        """
        Find the git SHAs for all revisions to a particular type
        """
        relative_type_path = os.path.join(self.lcmtypes_path, "{package_name:s}_{type_name:s}.lcm".format(
            package_name=self.package_name, type_name=type_name))
        cdata = subprocess.check_output("git --no-pager -C {0:s} log --pretty=oneline {1:s}".format(
            self.repo_path, relative_type_path), shell=True)
        shas = [c[:40] for c in cdata.split('\n') if len(c) >= 40]
        return shas

    def getSHAsForTypeAndChildren(self, type_name, processed=None):
        """
        Find the git SHAs for all revisions to a type *and* all of its children
        """
        shas = set([])
        if processed is None:
            processed = set([])

        shas.update(self.getSHAsForType(type_name))
        child_shas = set([])
        for sha in shas:
            try:
                for child in self.getChildTypes(type_name, sha):
                    if (child, sha) not in processed:
                        processed.add((child, sha))
                        child_shas.update(self.getSHAsForTypeAndChildren(child, processed))

            except TypeNotFoundError:
                continue
        shas.update(child_shas)
        return shas

    def getTypeAtSHA(self, type_name, sha):
        """
        Get the python class for a given LCM type at a given revision, building it as necessary
        """
        if not (type_name, sha) in self.type_cache:
            build_file = self.getOrCreateBuildFile(type_name, sha)
            build_dir = os.path.join(self.build_dir, sha)
            path = sys.path[:]
            sys.path.insert(0, build_dir)
            module = imp.load_source(type_name, build_file)
            sys.path = path
            self.type_cache[(type_name, sha)] = module.__dict__[type_name]

        return self.type_cache[(type_name, sha)]

    def decode(self, type_name, msg_data):
        """
        Try to decode an LCM message using its historical definitions. Uses a MRU (most recently used) queue of commit SHAs to try to ensure that repeated calls for messages of the same type are fast
        """
        if not self._initialized:
            print("Warning: Possible out-of-date LCM message received. I will now try to decode the message using older versions of the type definition. This will be slow the first time it happens.")
            self._initialized = True
        i = 0
        if not type_name in self._mru_shas_cache:
            self._mru_shas_cache[type_name] = list(self.getSHAsForTypeAndChildren(type_name))
        for i, sha in enumerate(self._mru_shas_cache[type_name]):
            try:
                msg_class = self.getTypeAtSHA(type_name, sha)
            except TypeNotFoundError:
                continue
            try:
                # print "Trying to decode using definition of type from commit {sha:s}".format(sha=sha[:8])
                msg_obj = msg_class.decode(msg_data)
            except ValueError as e:
                continue
            self._mru_shas_cache[type_name].pop(i)
            self._mru_shas_cache[type_name].insert(0, sha)
            return msg_obj
        raise ValueError("Unable to decode message data with any available type definitions.")


class LCMLoggerManager(object):
    '''
    This class provides some convenient methods for managing instances of
    the lcm-logger process.  You can start/stop instances of the lcm-logger
    process, and search for existing instances of the process.  It also has
    support for parsing command lines to attempt to extract log file names
    from running instances of lcm-logger.
    '''

    def __init__(self):
        self.filePatternPrefix = 'lcmlog'
        self.baseDir = os.path.expanduser('~/logs/raw')
        self.existingLoggerProcesses = {}

    @staticmethod
    def getTimeTag():
        return datetime.datetime.now().strftime('%Y-%m-%d__%H-%M-%S-%f')

    def startNewLogger(self, tag='', baseDir=None):
        filePattern = [self.filePatternPrefix, self.getTimeTag()]
        if tag:
            filePattern.append(tag)
        filePattern = '__'.join(filePattern)

        if baseDir is None:
            baseDir = self.baseDir

        fileArg = os.path.join(baseDir, filePattern)
        command = ['lcm-logger', fileArg]

        devnull = open(os.devnull, 'w')
        p = subprocess.Popen(command, stdout=devnull, stderr=devnull)
        return p.pid

    def updateExistingLoggerProcesses(self):
        output = subprocess.check_output(['ps', '-eo', 'pid,command'])
        self.existingLoggerProcesses = {}
        for line in output.splitlines()[1:]:
            fields = line.split()
            pid = int(fields[0])
            processName = fields[1].strip()
            args = fields[2:]
            if 'lcm-logger' in processName and len(args) == 1 and 'lcmlog__' in args[0]:
                self.existingLoggerProcesses[pid] = (processName, args)
        return self.existingLoggerProcesses

    def getActiveLoggerPids(self):
        return sorted(self.existingLoggerProcesses.keys())

    def getActiveLogFilenames(self):
        files = []
        for pid, command in list(self.existingLoggerProcesses.items()):
            processName, args = command
            for arg in args:
                if os.path.isfile(arg):
                    files.append(arg)
        return files

    def killAllLoggingProcesses(self):
        for pid, command in list(self.existingLoggerProcesses.items()):
            processName, args = command
            os.system('kill %d' % pid)
