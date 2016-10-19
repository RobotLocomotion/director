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

    def getDisabledOptions(self):
        options = self.getDefaultOptions()
        for opt in options._fields:
            setattr(options, opt, False)
        return options

    def getDefaultOptions(self):

        options = dict()
        for name in self.componentGraph.getComponentNames():
            options['use'+name] = True

        options = FieldContainer(**options)
        self.initDefaultOptions(options)
        return options

    def construct(self, options=None, **kwargs):

        options = options or self.getDefaultOptions()
        fields = FieldContainer(options=options, **kwargs)

        componentGraph = self.componentGraph

        for name in componentGraph.getComponentNames():
            if 'use'+name not in options.__dict__:
                raise Exception('Missing use option: ' + 'use'+name)
            if 'init'+name not in dir(self):
                raise Exception('Missing init function: ' + 'init'+name)

        initOrder = toposort_flatten(componentGraph._graph)

        for name in initOrder:

            initFunction = getattr(self, 'init'+name)
            isEnabled = getattr(options, 'use'+name)

            if isEnabled:
                initFunction(fields)

        return fields
