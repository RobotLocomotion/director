import os
import glob

def getTaskDescriptionFiles(searchDir):
    assert os.path.isdir(searchDir)
    files = glob.glob(os.path.join(searchDir, '*.py'))
    files = [f for f in files if os.path.basename(f) != '__init__.py']
    return files


def loadTaskDescriptionsFromFile(descriptionFile):
    descriptions = []
    globalVars = {}
    execfile(descriptionFile, globalVars)

    for key, value in globalVars.iteritems():
        if isinstance(value, list):
            descriptions.append((key, value))

    return descriptions


def loadTaskDescriptions(searchDir=None):
    searchDir = searchDir or os.path.dirname(__file__)
    descriptionFiles = getTaskDescriptionFiles(searchDir)
    descriptions = []
    for filename in descriptionFiles:
        descriptions.extend(loadTaskDescriptionsFromFile(filename))
    return descriptions
