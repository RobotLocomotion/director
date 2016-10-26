import os
import sys
import subprocess
import commands
import re
import shutil

def findMachOFiles(searchPath, recursive=True):
    maxDepthArg = '-maxdepth 1' if not recursive else ''
    filenames = commands.getoutput('find "%s" %s -exec file {} \;| grep Mach-O | sed "s/:.*//"' % (searchPath, maxDepthArg)).splitlines()
    return filenames


def findDependencies(filename):
    filenames = commands.getoutput('otool -l "%s" | grep " name" | sort | uniq | sed -e "s/ *name //g" | grep -v "@" | sed "s/ (offset.*)//"' % filename).splitlines()

    if 'executable' not in getMachOType(filename):
        try:
            fileId = getMachOId(filename)
        except RuntimeError:
            pass
        else:
            if fileId in filenames:
                filenames.remove(fileId)

    return filenames


def getMachOType(filename):
    return commands.getoutput('file "%s" | sed "s/.*: //"' % filename)


def getMachOId(filename):

    output = commands.getoutput('otool -D %s' % filename)
    match = re.match('[^:]+:\s*([^\s]+)', output)
    if not match:
        raise RuntimeError('Could not determine id for %s' % filename)
    return match.group(1)


def isFramework(filename):
    return '.framework' in filename


class Dependency(object):

    def __init__(self, path):
        self.old_path = path
        self.old_realpath = os.path.realpath(path)
        self.old_id = getMachOId(path)

        self.new_path = None
        self.new_realpath = None
        self.new_id = None

        self.copied = False


    def __hash__(self):
        return self.old_realpath.__hash__()

    def __eq__(self, other):
        return self.old_realpath == other.old_realpath

    def __repr__(self):
        return "Dependency(%s : %s)" % (self.old_realpath)

    def copyToDirectory(self, directory):

        if isFramework(self.old_realpath):

            m = re.match('(.*)/(\w+\.framework)/(.*)', self.old_realpath)

            frameworkDir = os.path.join(m.group(1), m.group(2))
            destDir = os.path.join(directory, m.group(2))

            if not os.path.isdir(destDir):
                print '    --> copy:', frameworkDir, destDir
                shutil.copytree(frameworkDir, destDir, symlinks=True)
                self.copied = True

            self.new_id = os.path.join(m.group(2), m.group(3))
            self.new_realpath = os.path.join(destDir, m.group(3))
            self.new_path = self.new_realpath

            self._changeId()

        else:
            self.new_id = os.path.basename(self.old_realpath)
            self.new_realpath = os.path.join(directory, self.new_id)
            self.new_path = self.new_realpath

            if not os.path.isfile(self.new_realpath):
                print '    --> copy:', self.old_realpath, self.new_realpath
                shutil.copy(self.old_realpath, self.new_realpath)
                self.copied = True

            self._changeId()

            #print 'id:', self.new_id


    def _changeId(self):
        commands.getoutput('chmod u+w "%s"' % self.new_realpath)
        commands.getoutput('install_name_tool -id "%s" "%s"' % (self.new_id, self.new_realpath))
        #commands.getoutput('chmod a-w "%s"' % filedest)


def resolveDependencyName(dep, libraryDir):
    if os.path.isfile(dep):
        return dep


    buildDir = os.path.realpath(sys.argv[1])

    depInLibDir = os.path.join(buildDir, 'lib', dep)

    print '  dep name is not file:', dep
    print '  trying depInLibDir:', depInLibDir

    if os.path.isfile(depInLibDir):
        return depInLibDir

    return dep

def scan(packageDir, libraryDir, processed, excludedDeps, excludeFunc):

    files = findMachOFiles(packageDir)

    numCopied = 0

    for filename in files:

        if not os.path.isfile(filename):
            print 'warning, skipping unexpected mach-o file:', filename
            continue

        filename = os.path.realpath(filename)

        if filename in processed:
            continue

        processed.add(filename)

        print 'processing:', filename

        deps = findDependencies(filename)

        installNameChanges = []

        for dep in deps:

            dep = resolveDependencyName(dep, libraryDir)
            if not os.path.isfile(dep):
                print '  warning, skipping dependency not found:', dep
                raise Exception()
                continue

            if excludeFunc(dep):
                excludedDeps.add(os.path.realpath(dep))
                continue

            dep = Dependency(dep)


            print '  dep:', dep.old_path
            if dep.old_path != dep.old_realpath:
                print '    realpath:', dep.old_realpath


            dep.copyToDirectory(libraryDir)

            if dep.copied:
                numCopied += 1


            relativePathToDependency = os.path.relpath(dep.new_realpath, os.path.dirname(filename))
            newInstallName = '@loader_path/' + relativePathToDependency

            print '    new relative name:', newInstallName

            for old in (dep.old_path, dep.old_realpath, dep.old_id):
                installNameChanges.append((old, newInstallName))

        installNameArgs = ['-change "%s" "%s"' % (old, new) for old, new in installNameChanges]
        installNameArgs = list(set(installNameArgs))

        #for arg in installNameArgs:
        #    print '    ', arg

        commands.getoutput('chmod u+w "%s"' % filename)
        commands.getoutput('install_name_tool %s "%s"' % (' '.join(installNameArgs), filename))
        relativePathToLibraryDir = os.path.relpath(libraryDir, os.path.dirname(filename))
        commands.getoutput('install_name_tool -add_rpath "@loader_path/%s" "%s"' % (relativePathToLibraryDir, filename))

    return numCopied


def main():


    buildDir = sys.argv[1]
    packageDir = sys.argv[2]
    libraryDir = sys.argv[3]


    assert os.path.isdir(buildDir)
    assert os.path.isdir(packageDir)
    assert os.path.isdir(libraryDir)
    assert '..' not in os.path.relpath(libraryDir, packageDir)

    prefixes = [os.path.realpath(buildDir), '/usr/local']

    def isExcluded(filename):
        for prefix in prefixes:
            if filename.startswith(prefix):
                return False
        return True


    excludedDeps = set()
    processed = set()


    while scan(packageDir, libraryDir, processed, excludedDeps, isExcluded) > 0:
        continue


    excludedDeps = sorted(list(excludedDeps))
    print
    print 'other dependencies:'
    for filename in excludedDeps:
        print filename


if __name__ == '__main__':
    main()
