import os

class PackageMap(object):

    def __init__(self):
        self.map = {}

    def addPackageDir(self, path):
        packageName = os.path.basename(path)
        if packageName:
            self.addPackage(packageName, path)

    def addPackage(self, packageName, path):

        if packageName in self.map:
            print('warning, skipping package path:', path)
            print('existing package path:', packageName, self.map[packageName])
            return

        self.map[packageName] = path

    def printPackageMap(self):
        for package, path in self.map.items():
            print(package, path)

    def getPath(self, packageName):
        return self.map.get(packageName)

    def populateFromSearchPaths(self, paths):
        if isinstance(paths, str):
            paths = [paths]
        packagePaths = []
        for path in paths:
            for root, dirnames, filenames in os.walk(path):
                if os.path.isfile(os.path.join(root, 'package.xml')) or os.path.isfile(os.path.join(root, 'manifest.xml')):
                    packagePaths.append(root)

        for path in packagePaths:
            self.addPackageDir(path)

    def populateFromEnvironment(self, environmentVariables):

        searchPaths = []
        for varName in environmentVariables:
            searchPaths.extend(os.environ.get(varName, '').split(':'))

        self.populateFromSearchPaths(searchPaths)

    @staticmethod
    def isPackageUrl(filename):
        return filename.lower().startswith('package://')

    def resolveFilename(self, filename):
        if not self.isPackageUrl(filename):
            return None

        relPath = filename[10:]

        try:
            idx = relPath.index('/')
            packageName = relPath[:idx]
            relPath = relPath[idx+1:]
        except:
            print('failed to parse package filename:', filename)
            return None

        packagePath = self.getPath(packageName)

        if not packagePath:
            print('failed to resolve filename, unknown package:', filename)
            return None

        return os.path.join(packagePath, relPath)


def test():

    m = PackageMap()

    assert m.getPath('bar') is None

    m.addPackageDir('/home/user/bar')
    assert m.getPath('bar') == '/home/user/bar'

    path = '/tmp/package_test/foo'
    packageFile = os.path.join(path, 'package.xml')


    if not os.path.isfile(packageFile):
        if not os.path.isdir(path):
            os.makedirs(path)
        open(packageFile, 'w').close()

    os.environ['TEST_PACKAGE'] = '/tmp/package_test'

    m.populateFromEnvironment(['TEST_PACKAGE'])

    assert m.getPath('foo') == '/tmp/package_test/foo'
    assert m.resolveFilename('package://foo/bar.obj') == '/tmp/package_test/foo/bar.obj'

    m.printPackageMap()


if __name__ == '__main__':
    test()
