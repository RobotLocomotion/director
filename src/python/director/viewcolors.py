

class ViewBackgroundLightHandler(object):

    def __init__(self, viewOptions, grid, action):
        self.viewOptions = viewOptions
        self.action = action
        self.action.checkable = True
        self.action.connect('triggered()', self._onChecked)

        self.properties = { viewOptions : {'Gradient background':True, 'Background color':[0.0, 0.0, 0.0], 'Background color 2':[0.3, 0.3, 0.3]},
                            grid : {'Surface Mode':'Wireframe', 'Alpha':0.05, 'Color':[1.0, 1.0, 1.0], 'Color By':0}
                          }

        self.cachedProperties = {}
        self.storeProperties()

    def storeProperties(self):

        def grab(obj, props):
            for key in list(props.keys()):
                self.cachedProperties.setdefault(obj, dict())[key] = obj.getProperty(key)

        for obj, props in list(self.properties.items()):
            grab(obj, props)

    def applyProperties(self, properties):

        def send(obj, props):
            for key, value in list(props.items()):
                obj.setProperty(key, value)

        for obj, props in list(properties.items()):
            send(obj, props)

    def setEnabled(self, enabled):
        if self.isEnabled() != enabled:
            self.action.checked = enabled
            self._onChecked()

    def isEnabled(self):
        return bool(self.action.checked)

    def toggle(self):
        self.setEnabled(not self.isEnabled())

    def _onChecked(self):
        if self.action.checked:
            self.storeProperties()
            self.applyProperties(self.properties)
        else:
            self.applyProperties(self.cachedProperties)
