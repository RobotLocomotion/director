import director
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


        def flatten(l):
            return [item for sublist in l for item in sublist]

        # now flatten some list of lists
        self._args.data_files = flatten(self._args.data_files)


    def getDefaultBotConfigFile(self):
        return os.path.join(director.getDRCBaseDir(), 'software/config/val/robot.cfg')

    def getDefaultDirectorConfigFile(self):
        return self.getDefaultValkyrieDirectorConfigFile();

    def getDefaultAtlasV3DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/atlas_v3/director_config.json')

    def getDefaultAtlasV4DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/atlas_v4/director_config.json')

    def getDefaultAtlasV5DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/atlas_v5/director_config.json')

    def getDefaultValkyrieDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/val_description/director_config.json')

    def getDefaultValkyrieSimpleDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/val_description/director_config_simple.json')

    def getDefaultKukaLWRConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/lwr_defs/director_config.json')

    def getDefaultHuskyConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/husky_description/director_config.json')

    def getDefaultDualArmHuskyConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'software/models/dual_arm_husky_description/director_config.json')


    def _isPyDrakeAvailable(self):
        try:
            import pydrake
        except ImportError:
            return False
        if 'DRAKE_RESOURCE_ROOT' not in os.environ:
            return False
        return True

    def addDrakeConfigShortcuts(self, directorConfig):

        import pydrake

        directorConfig.add_argument(
            '--iiwa-drake',
            dest='directorConfigFile',
            action='store_const',
            const=pydrake.common.FindResourceOrThrow('drake/examples/kuka_iiwa_arm/director_config.json'),
            help='Use KUKA IIWA from drake/examples')

    def addOpenHumanoidsConfigShortcuts(self, directorConfig):

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

        directorConfig.add_argument('-val', '--valkyrie', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieDirectorConfigFile(),
                            help='Use Valkyrie (Default)')

        directorConfig.add_argument('-val_simple', '--valkyrie_simple', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieSimpleDirectorConfigFile(),
                            help='Use Valkyrie (Simple/Primitive Shapes)')

        directorConfig.add_argument('-lwr', '--lwr', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultKukaLWRConfigFile(),
                            help='Use Kuka LWR')

        directorConfig.add_argument('-husky', '--husky', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultHuskyConfigFile(),
                            help='Use Husky')

        directorConfig.add_argument('-dual_arm_husky', '--dual_arm_husky', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultDualArmHuskyConfigFile(),
                            help='Use Dual Arm Husky')

    def addDefaultArgs(self, parser):

        parser.add_argument('-c', '--bot-config', '--config_file', dest='config_file',
                            metavar='filename', type=str, help='Robot cfg file')

        parser.add_argument('--matlab-host', metavar='hostname', type=str,
                            help='hostname to connect with external matlab server')

        directorConfig = parser.add_mutually_exclusive_group(required=False)
        directorConfig.add_argument('--director-config', '--director_config', dest='directorConfigFile',
                                    type=str, default='', metavar='filename',
                                    help='JSON file specifying which urdfs to use')

        if director.getDRCBaseIsSet():
            self.addOpenHumanoidsConfigShortcuts(directorConfig)
            parser.set_defaults(directorConfigFile=self.getDefaultDirectorConfigFile(),
                                config_file=self.getDefaultBotConfigFile())

        if self._isPyDrakeAvailable():
            self.addDrakeConfigShortcuts(directorConfig)

        parser.add_argument('--data', type=str, nargs='+', dest='data_files',
                            default=[], action='append', metavar='filename',
                            help='data files to load at startup')

        parser.add_argument('--script', '--startup', type=str, nargs='+', dest='scripts',
                            default=[], action='append', metavar='filename',
                            help='python scripts to run at startup')


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


class DirectorConfig(object):

    def __init__(self, filename):

        self.filename = filename
        if not os.path.isfile(filename):
            raise Exception('Director config file not found: %s' % filename)

        self.dirname = os.path.dirname(os.path.abspath(filename))
        self.config = json.load(open(filename))

        if self.config.get('fixedPointFile'):
            self.config['fixedPointFile'] = os.path.join(self.dirname, self.config['fixedPointFile'])

        self.urdfConfig = self.config['urdfConfig']
        for key, urdf in list(self.urdfConfig.items()):
            self.urdfConfig[key] = os.path.join(self.dirname, urdf)

    _defaultInstance = None

    @classmethod
    def getDefaultInstance(cls):
        if cls._defaultInstance is None:
            if not args().directorConfigFile:
                raise Exception('Director config file is not defined. '
                                'Use --director-config <filename> on the command line.')
            cls._defaultInstance = DirectorConfig(args().directorConfigFile)
        return cls._defaultInstance


def getDirectorConfig():
    return DirectorConfig.getDefaultInstance().config
