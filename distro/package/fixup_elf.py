import os
import sys
import subprocess
import commands



def findElfFiles(searchPath, recursive=True):
    maxDepthArg = '-maxdepth 1' if not recursive else ''
    filenames = commands.getoutput('find "%s" %s -exec file {} \;| grep ELF | sed "s/:.*//"' % (searchPath, maxDepthArg)).splitlines()
    return filenames


def main():

    baseDir = sys.argv[1]
    libraryDir = sys.argv[2]
    patchElfCommand = sys.argv[3]

    assert os.path.isfile(patchElfCommand)
    assert os.path.isdir(libraryDir)
    assert os.path.isdir(baseDir)
    assert '..' not in os.path.relpath(libraryDir, baseDir)


    files = findElfFiles(baseDir)

    for elfFile in files:

        print 'processing:', elfFile
        relativePathToLibraryDir = os.path.relpath(libraryDir, os.path.dirname(elfFile))

        status, output = commands.getstatusoutput('"%s" --set-rpath "\\$ORIGIN/%s" --force-rpath "%s"' % (patchElfCommand, relativePathToLibraryDir, elfFile))
        if status != 0:
            print 'patchelf error on file: %s\n%s' % (elfFile, output)
            return

        #print '    $ORIGIN/%s' % relativePathToLibraryDir


if __name__ == '__main__':
    main()
