from collections import OrderedDict
from director.fieldcontainer import FieldContainer
from director.thirdparty.toposort import toposort_flatten


class ComponentGraph(object):

    def __init__(self):
        self._graph = dict()

    def getComponentGraph(self):
        return self._graph

    def getComponentNames(self):
        return list(self._graph.keys())

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
            print(name)
            print('       wants -->', ', '.join(wants) or 'none')
            print('  also needs -->', ', '.join(needs) or 'none')

    def addComponent(self, name, deps):
        self._graph[name] = set(deps)


class ComponentFactory(object):

    def __init__(self):
        self.componentGraph = ComponentGraph()
        self.initFunctions = {}
        self.componentFields = {}
        self.defaultOptions = FieldContainer()

    def register(self, factoryClass):

        fact = factoryClass()
        components, disabledComponents = fact.getComponents()

        for name in sorted(components.keys()):
            if name in self.componentGraph.getComponentNames():
                raise Exception('Component %s from %s has already been registered.' % (name, factoryClass.__name__))

            if not hasattr(fact, 'init'+name):
                raise Exception('Missing init function for component %s' % name)

        for name in disabledComponents:
            if name not in list(components.keys()):
                raise Exception('Unknown component %s found in list of disabled components.' % name)

        options = dict()
        for name, deps in list(components.items()):
            self.componentGraph.addComponent(name, deps)
            self.initFunctions[name] = getattr(fact, 'init'+name)
            isEnabled = name not in disabledComponents
            options['use'+name] = isEnabled

        self.defaultOptions._add_fields(**options)

    def getDisabledOptions(self, **kwargs):
        options = self.getDefaultOptions()
        for opt in options._fields:
            options[opt] = False
        self.setDependentOptions(options, **kwargs)
        return options

    def getDefaultOptions(self, **kwargs):
        options = FieldContainer(**dict(self.defaultOptions))
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
        for name in list(kwargs.keys()):
            if name not in options._fields:
                raise Exception('unknown option given: ' + name)

        for name, enabled in list(kwargs.items()):
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
        defaultFields = FieldContainer(options=options, **kwargs)

        initOrder = toposort_flatten(self.componentGraph.getComponentGraph())
        for name in initOrder:
            isEnabled = getattr(options, 'use'+name)
            if isEnabled:
                self.initComponent(name, defaultFields)

        fields = self._joinFields([defaultFields] + list(self.componentFields.values()))
        return fields

    def printComponentFields(self):
        for k, v in sorted(self.componentFields.items()):
            print('%s:' % k)
            for name in v._fields:
                print('  ', name)

    def initComponent(self, name, defaultFields):

        initFunction = self.initFunctions[name]
        dependencies = self.componentGraph.getComponentDependencies(name)
        inputFields = self._joinFields([defaultFields] + [self.componentFields[dep] for dep in dependencies])
        newFields = initFunction(inputFields)

        if not newFields:
            newFields = FieldContainer()
        self.componentFields[name] = newFields
