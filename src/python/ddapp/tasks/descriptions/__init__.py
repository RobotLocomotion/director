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

def getDefaultSearchDirectory():
    return os.path.dirname(__file__)

def loadTaskDescription(name, searchDir=None):
    searchDir = searchDir or getDefaultSearchDirectory()
    filename = os.path.join(searchDir, name + '.py')
    return loadTaskDescriptionsFromFile(filename)[0][1]

def loadTaskDescriptions(searchDir=None):
    searchDir = searchDir or getDefaultSearchDirectory()
    descriptionFiles = getTaskDescriptionFiles(searchDir)
    descriptions = []
    for filename in descriptionFiles:
        descriptions.extend(loadTaskDescriptionsFromFile(filename))
    return descriptions
