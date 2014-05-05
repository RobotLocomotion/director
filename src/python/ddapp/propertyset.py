from ddapp import callbacks
from ddapp.fieldcontainer import FieldContainer

import re
from collections import OrderedDict


def cleanPropertyName(s):
    """
    Generate a valid python property name by replacing all non-alphanumeric characters with underscores and adding an initial underscore if the first character is a digit
    """
    return re.sub(r'\W|^(?=\d)','_',s).lower()  # \W matches non-alphanumeric, ^(?=\d) matches the first position if followed by a digit


class PropertyAttributes(FieldContainer):

    def __init__(self, **kwargs):

        self._add_fields(
          decimals = 5,
          minimum = -1e4,
          maximum = 1e4,
          singleStep = 1,
          hidden = False,
          enumNames = None,
          readOnly = False,
          )

        self._set_fields(**kwargs)


class PropertySet(object):

    PROPERTY_CHANGED_SIGNAL = 'PROPERTY_CHANGED_SIGNAL'
    PROPERTY_ADDED_SIGNAL = 'PROPERTY_ADDED_SIGNAL'
    PROPERTY_ATTRIBUTE_CHANGED_SIGNAL = 'PROPERTY_ATTRIBUTE_CHANGED_SIGNAL'

    def __init__(self):
        self.callbacks = callbacks.CallbackRegistry([self.PROPERTY_CHANGED_SIGNAL,
                                                     self.PROPERTY_ADDED_SIGNAL,
                                                     self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL])

        self._properties = OrderedDict()
        self._attributes = {}
        self._alternateNames = {}

    def propertyNames(self):
        return self._properties.keys()

    def hasProperty(self, propertyName):
        return propertyName in self._properties

    def connectPropertyChanged(self, func):
        return self.callbacks.connect(self.PROPERTY_CHANGED_SIGNAL, func)

    def disconnectPropertyChanged(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPropertyAdded(self, func):
        return self.callbacks.connect(self.PROPERTY_ADDED_SIGNAL, func)

    def disconnectPropertyAdded(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPropertyAttributeChanged(self, func):
        return self.callbacks.connect(self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL, func)

    def disconnectPropertyAttributeChanged(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def getProperty(self, propertyName):
        assert self.hasProperty(propertyName)
        return self._properties[propertyName]

    def addProperty(self, propertyName, propertyValue, attributes=None):
        alternateName = cleanPropertyName(propertyName)
        if propertyName not in self._properties and alternateName in self._alternateNames:
            raise ValueError('Adding this property would conflict with a different existing property with alternate name {:s}'.format(alternateName))
        self._alternateNames[alternateName] = propertyName
        self._properties[propertyName] = propertyValue
        self._attributes[propertyName] = attributes or PropertyAttributes()

        self.callbacks.process(self.PROPERTY_ADDED_SIGNAL, self, propertyName)

    def setProperty(self, propertyName, propertyValue):
        assert self.hasProperty(propertyName)

        names = self.getPropertyAttribute(propertyName, 'enumNames')
        if names and type(propertyValue) != int:
            propertyValue = names.index(propertyValue)

        self.oldPropertyValue = (propertyName, self.getProperty(propertyName))
        self._properties[propertyName] = propertyValue
        self.oldPropertyValue = None
        self.callbacks.process(self.PROPERTY_CHANGED_SIGNAL, self, propertyName)

    def getPropertyAttribute(self, propertyName, propertyAttribute):
        assert self.hasProperty(propertyName)
        return getattr(self._attributes[propertyName], propertyAttribute)

    def setPropertyAttribute(self, propertyName, propertyAttribute, value):
        assert self.hasProperty(propertyName)
        attributes = self._attributes[propertyName]
        assert hasattr(attributes, propertyAttribute)
        setattr(attributes, propertyAttribute, value)
        self.callbacks.process(self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL, self, propertyName. propertyAttribute)

    def __getattribute__(self, name):
        try:
            alternateNames = object.__getattribute__(self, '_alternateNames')
            if name in alternateNames:
                return object.__getattribute__(self, 'getProperty')(alternateNames[name])
            else:
                raise AttributeError()
        except AttributeError:
            return object.__getattribute__(self, name)


class PropertyPanelHelper(object):

    @staticmethod
    def addPropertiesToPanel(properties, panel):

        for propertyName in properties.propertyNames():
            value = properties.getProperty(propertyName)
            attributes = properties._attributes[propertyName]
            if value is not None and not attributes.hidden:
                PropertyPanelHelper._addProperty(panel, propertyName, attributes, value)

    @staticmethod
    def onPropertyValueChanged(panel, properties, propertyName):
        prop = panel.findProperty(propertyName)
        if prop is not None:
            prop.setValue(properties.getProperty(propertyName))

    @staticmethod
    def _setPropertyAttributes(prop, attributes):

        prop.setAttribute('decimals', attributes.decimals)
        prop.setAttribute('minimum', attributes.minimum)
        prop.setAttribute('maximum', attributes.maximum)
        prop.setAttribute('singleStep', attributes.singleStep)
        if attributes.enumNames:
            prop.setAttribute('enumNames', attributes.enumNames)

    @staticmethod
    def _addProperty(panel, name, attributes, value):

        if isinstance(value, list) and not isinstance(value[0], str):
            groupName = '%s [%s]' % (name, ', '.join([str(v) for v in value]))
            groupProp = panel.addGroup(groupName)
            for v in value:
                p = panel.addSubProperty(name, v, groupProp)
                PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return groupProp
        elif attributes.enumNames:
            p = panel.addEnumProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return p
        else:
            p = panel.addProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return p
