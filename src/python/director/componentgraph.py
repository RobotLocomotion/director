from collections import OrderedDict
from director.fieldcontainer import FieldContainer
from director.thirdparty.toposort import toposort_flatten


class ComponentGraph(object):

    def __init__(self):
        self._graph = dict()

    def getComponentGraph(self):
        return self._graph

    def getComponentNames(self):
        return self._graph.keys()

    def getComponentDependencies(self, componentName):
        componentGraph = self.getComponentGraph()
        assert componentName in componentGraph
        allDeps = set()
        toProcess = [componentName]

        while len(toProcess):
            name = toProcess.pop(0)
            deps = componentGraph[name]
            toProcess.extend(list(deps))
            allDeps = allDeps.union(deps)

        return list(allDeps)

    def printComponentGraph(self):

        componentGraph = self.getComponentGraph()

        for name in sorted(componentGraph.keys()):
            deps = self.getComponentDependencies(name)
            wants = componentGraph[name]
            needs = list(set(deps).difference(wants))
            print name
            print '       wants -->', ', '.join(wants) or 'none'
            print '  also needs -->', ', '.join(needs) or 'none'

    def addComponent(self, name, deps):
        self._graph[name] = set(deps)


class ComponentFactory(object):

    def __init__(self):
        self.componentGraph = ComponentGraph()
        self.addComponents(self.componentGraph)

    def addComponents(self, componentGraph):
        pass

    def initDefaultOptions(self, options):
        pass

    def getDisabledOptions(self, **kwargs):
        options = self.getDefaultOptions()
        for opt in options._fields:
            options[opt] = False
        self.setDependentOptions(options, **kwargs)
        return options

    def getDefaultOptions(self, **kwargs):

        options = dict()
        for name in self.componentGraph.getComponentNames():
            options['use'+name] = True

        options = FieldContainer(**options)
        self.initDefaultOptions(options)
        self.setDependentOptions(options, **kwargs)
        return options

    def _toComponentName(self, optionName):
        assert optionName[:3] == 'use'
        return optionName[3:]

    def _toOptionName(self, componentName):
        return 'use' + componentName

    def _joinFields(self, fieldsList):
        f = FieldContainer()
        for fields in fieldsList:
            f._add_fields(**dict(fields))
        return f

    def setDependentOptions(self, options, **kwargs):

        # verify the given args exist in the options fields
        for name in kwargs.keys():
            if name not in options._fields:
                raise Exception('unknown option given: ' + name)

        for name, enabled in kwargs.iteritems():
            setattr(options, name, enabled)

            # if option is being enabled, also enable its dependencies
            if enabled:
                name = self._toComponentName(name)
                dependencies = self.componentGraph.getComponentDependencies(name)
                for dep in dependencies:
                    setattr(options, self._toOptionName(dep), True)

        return options

    def _verifyOptions(self, options):

        for name, enabled in options:

            if enabled:
                name = self._toComponentName(name)
                dependencies = self.componentGraph.getComponentDependencies(name)
                for dep in dependencies:
                    if not getattr(options, self._toOptionName(dep)):
                        raise Exception('Component %s depends on component %s, but %s is disabled.' % (name, dep, dep))


    def construct(self, options=None, **kwargs):

        options = options or self.getDefaultOptions()
        if isinstance(options, dict):
            options = self.setDependentOptions(self.getDefaultOptions(), **options)
        self._verifyOptions(options)

        componentGraph = self.componentGraph

        for name in componentGraph.getComponentNames():
            if 'use'+name not in options.__dict__:
                raise Exception('Missing use option: ' + 'use'+name)
            if 'init'+name not in dir(self):
                raise Exception('Missing init function: ' + 'init'+name)

        initOrder = toposort_flatten(componentGraph._graph)

        componentFields = OrderedDict()
        defaultFields = FieldContainer(options=options, **kwargs)

        for name in initOrder:

            initFunction = getattr(self, 'init'+name)
            isEnabled = getattr(options, 'use'+name)

            if isEnabled:
                dependencies = componentGraph.getComponentDependencies(name)
                inputFields = self._joinFields([defaultFields] + [componentFields[dep] for dep in dependencies])
                newFields = initFunction(inputFields)

                if not newFields:
                    newFields = FieldContainer()
                componentFields[name] = newFields


#        for componentName, componentFields in app.iteritems():
#            print componentName, 'exports fields:'
#            for name in componentFields._fields:
#                print '  ', name

        fields = self._joinFields([defaultFields] + componentFields.values())
        return fields
