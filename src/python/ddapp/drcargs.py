import ddapp
import os
import sys
import argparse
import json



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
        return os.path.join(ddapp.getDRCBaseDir(), 'software/config/drc_robot.cfg')

    def getDefaultDirectorConfigFile(self):
        return self.getDefaultAtlasV5DirectorConfigFile();

    def getDefaultAtlasV3DirectorConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/atlas_v3/director_config.json')

    def getDefaultAtlasV4DirectorConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/atlas_v4/director_config.json')

    def getDefaultAtlasV5DirectorConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/atlas_v5/director_config.json')

    def getDefaultValkyrieV1DirectorConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/valkyrie/director_config.json')

    def getDefaultValkyrieV2DirectorConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/val_description/director_config.json')

    def getDefaultKukaLWRConfigFile(self):
        return os.path.join(ddapp.getDRCBaseDir(),
                            'software/models/lwr_defs/director_config.json')

    def addDefaultArgs(self, parser):
        parser.add_argument('-c', '--config_file', type=str, help='Robot cfg file',
                            default=self.getDefaultBotConfigFile())

        parser.add_argument('--matlab-host', type=str, help='Hostname to use external matlab server')

        parser.add_argument('-exo', '--exo', action='store_true', dest='exo', help='Publish plannig requests to external planner instead of Drake')

        directorConfig = parser.add_mutually_exclusive_group(required=False)
        directorConfig.add_argument('-v3', '--atlas_v3', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV3DirectorConfigFile(),
                            help='Use Atlas V3')

        directorConfig.add_argument('-v4', '--atlas_v4', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV4DirectorConfigFile(),
                            help='Use Atlas V4')

        directorConfig.add_argument('-v5', '--atlas_v5', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV5DirectorConfigFile(),
                            help='Use Atlas V5')

        directorConfig.add_argument('-val1', '--valkyrie_v1', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieV1DirectorConfigFile(),
                            help='Use Valkyrie')

        directorConfig.add_argument('-val2', '--valkyrie_v2', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieV2DirectorConfigFile(),
                            help='Use Valkyrie')

        directorConfig.add_argument('-lwr', '--lwr', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultKukaLWRConfigFile(),
                            help='Use Kuka LWR')

        directorConfig.add_argument('--director_config', dest='directorConfigFile',
                            type=str,
                            help='JSON file specifying which urdfs to use')

        parser.set_defaults(directorConfigFile=self.getDefaultDirectorConfigFile())

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

_directorConfig = None
def getDirectorConfig():
    global _directorConfig
    if not _directorConfig:
        with open(args().directorConfigFile) as directorConfigFile:
            _directorConfig = json.load(directorConfigFile)
    return _directorConfig
