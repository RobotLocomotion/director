

class ViewBackgroundLightHandler(object):

    def __init__(self, viewOptions, grid, action):
        self.viewOptions = viewOptions
        self.action = action
        self.action.checkable = True
        self.action.connect('triggered()', self.toggle)

        self.properties = { viewOptions : {'Gradient background':True, 'Background color':[0.0, 0.0, 0.0], 'Background color 2':[0.3, 0.3, 0.3]},
                            grid : {'Surface Mode':'Wireframe', 'Alpha':0.05, 'Color':[1.0, 1.0, 1.0], 'Color By':0}
                          }

        self.cachedProperties = {}
        self.storeProperties()

    def storeProperties(self):

        def grab(obj, props):
            for key in props.keys():
                self.cachedProperties.setdefault(obj, dict())[key] = obj.getProperty(key)

        for obj, props in self.properties.iteritems():
            grab(obj, props)

    def applyProperties(self, properties):

        def send(obj, props):
            for key, value in props.iteritems():
                obj.setProperty(key, value)

        for obj, props in properties.iteritems():
            send(obj, props)

    def toggle(self):
        if self.action.checked:
            self.storeProperties()
            self.applyProperties(self.properties)
        else:
            self.applyProperties(self.cachedProperties)
