import os
import sys
import subprocess


def findElfFiles(searchPath, recursive=True):
    maxDepthArg = '-maxdepth 1' if not recursive else ''
    filenames = subprocess.check_output('find "%s" %s -exec file {} \;| grep ELF | sed "s/:.*//"' % (searchPath, maxDepthArg), shell=True).splitlines()
    return filenames


def main():

    workDir = sys.argv[1]
    baseDir = sys.argv[2]
    patchElfCommand = sys.argv[3]

    assert os.path.isdir(workDir)
    assert os.path.isdir(baseDir)
    assert os.path.isfile(patchElfCommand)


    files = findElfFiles(baseDir)

    for elfFile in files:

        print 'processing:', elfFile

        # read the existing RPATH value from the elf file
        rpath = subprocess.check_output('"%s" --print-rpath "%s"' % (patchElfCommand, elfFile), shell=True).strip()

        # if the RPATH is empty, then we can skip this file
        if not rpath:
            continue


        print '  old rpath:', rpath
        newPaths = []

        # loop over each path in the RPATH string and replace absoluate paths with $ORIGIN
        for path in rpath.split(':'):
            if not path:
                continue

            path = path.replace('$ORIGIN', os.path.dirname(elfFile))
            assert os.path.isdir(path)

            # if the RPATH includes a dir outside the base dir then this is an error
            relativePathToBaseDir = os.path.relpath(path, baseDir)
            if '..' in relativePathToBaseDir:
                raise Exception('found RPATH outside of install prefix: %s' % rpath)

            relativePathToElf = os.path.relpath(path, os.path.dirname(elfFile))
            newPaths.append('$ORIGIN/%s' % relativePathToElf)

        newRpath = ':'.join(newPaths)
        print '  new rpath:', newRpath

        newElfFile = os.path.join(workDir, os.path.relpath(elfFile, baseDir))
        assert os.path.isfile(newElfFile)
        subprocess.check_output('"%s" --set-rpath "%s" --force-rpath "%s"' % (patchElfCommand, newRpath.replace('$', '\\$'), newElfFile), shell=True)


if __name__ == '__main__':
    main()
