import ddapp
import os
import sys
import argparse



class DRCArgParser(object):

    def __init__(self):
        self._args = None
        self._parser = None

    def getArgs(self):
        if self._args is None:
            self.parseArgs()
        return self._args

    def getParser(self):
        if self._parser is None:
            self._parser = argparse.ArgumentParser()
            self.addDefaultArgs(self._parser)
        return self._parser

    def parseArgs(self):
        parser = self.getParser()
        sys.argv = [str(v) for v in sys.argv]
        self._args = parser.parse_args()


    def getDefaultBotConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(), 'software/config/drc_robot_02_mit.cfg')


    def addDefaultArgs(self, parser):

        parser.add_argument('-c', '--config_file', type=str, help='Robot cfg file',
                            default=self.getDefaultBotConfigFile())

        parser.add_argument('--matlab-host', type=str, help='Hostname to use external matlab server',
                            default=None)

        parser.add_argument('data_files', type=str, nargs='*',
                            help='data files to load at startup')

_argParser = None
def getGlobalArgParser():
    global _argParser
    if not _argParser:
        _argParser = DRCArgParser()
    return _argParser


def args():
    return getGlobalArgParser().getArgs()
