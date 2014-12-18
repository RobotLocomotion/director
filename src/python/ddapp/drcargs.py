import ddapp
import os
import sys
import argparse



class DRCArgParser(object):

    def __init__(self):
        self._args = None
        self._parser = None
        self.strict = False

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

        if not self.strict:
            self._args, unknown = parser.parse_known_args()
        else:
            self._args = parser.parse_args()


    def getDefaultBotConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(), 'software/config/drc_robot_02_mit.cfg')

    def getDefaultUrdfConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/atlas_v4/urdf_config.json')


    def addDefaultArgs(self, parser):

        parser.add_argument('-c', '--config_file', type=str, help='Robot cfg file',
                            default=self.getDefaultBotConfigFile())

        parser.add_argument('--matlab-host', type=str, help='Hostname to use external matlab server',
                            default=None)

        parser.add_argument('--urdf_config', type=str, help='JSON file specifying which urdfs to use',
                            default=self.getDefaultUrdfConfigFile())

        parser.add_argument('data_files', type=str, nargs='*',
                            help='data files to load at startup')


_argParser = None
def getGlobalArgParser():
    global _argParser
    if not _argParser:
        _argParser = DRCArgParser()
    return _argParser

def requireStrict():
    global _argParser
    _argParser = None
    getGlobalArgParser().strict = True

def args():
    return getGlobalArgParser().getArgs()
