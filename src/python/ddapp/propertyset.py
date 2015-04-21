from ddapp import callbacks
from ddapp.fieldcontainer import FieldContainer

import re
import numpy as np
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

from PythonQt import QtGui

def fromQColor(propertyName, propertyValue):
    if isinstance(propertyValue, QtGui.QColor):
        return [propertyValue.red()/255.0, propertyValue.green()/255.0, propertyValue.blue()/255.0]
    else:
        return propertyValue

def toQProperty(propertyName, propertyValue):
    if 'color' in propertyName.lower() and isinstance(propertyValue, (list, tuple)) and len(propertyValue) == 3:
        return QtGui.QColor(propertyValue[0]*255.0, propertyValue[1]*255.0, propertyValue[2]*255.0)
    elif isinstance(propertyValue, np.float):
        return float(propertyValue)
    elif isinstance(propertyValue, (list, tuple, np.ndarray)) and len(propertyValue) and isinstance(propertyValue[0], np.float):
        return [float(x) for x in propertyValue]
    else:
        return propertyValue


class PropertySet(object):

    PROPERTY_CHANGED_SIGNAL = 'PROPERTY_CHANGED_SIGNAL'
    PROPERTY_ADDED_SIGNAL = 'PROPERTY_ADDED_SIGNAL'
    PROPERTY_ATTRIBUTE_CHANGED_SIGNAL = 'PROPERTY_ATTRIBUTE_CHANGED_SIGNAL'

    def __getstate__(self):
        d = dict(_properties=self._properties, _attributes=self._attributes)
        return d

    def __setstate__(self, state):
        self.__init__()
        attrs = state['_attributes']
        for propName, propValue in state['_properties'].iteritems():
            self.addProperty(propName, propValue, attributes=attrs.get(propName))


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

    def assertProperty(self, propertyName):
        assert self.hasProperty(propertyName), "Missing property: {:s}".format(propertyName)

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
        self.assertProperty(propertyName)
        return self._properties[propertyName]

    def getPropertyEnumValue(self, propertyName):
        self.assertProperty(propertyName)
        return self._attributes[propertyName].enumNames[self._properties[propertyName]]

    def removeProperty(self, propertyName):
        assert self.hasProperty(propertyName)
        alternateName = cleanPropertyName(propertyName)
        del self._alternateNames[alternateName]
        del self._properties[propertyName]
        del self._attributes[propertyName]

    def addProperty(self, propertyName, propertyValue, attributes=None):
        alternateName = cleanPropertyName(propertyName)
        if propertyName not in self._properties and alternateName in self._alternateNames:
            raise ValueError('Adding this property would conflict with a different existing property with alternate name {:s}'.format(alternateName))

        propertyValue = fromQColor(propertyName, propertyValue)

        self._alternateNames[alternateName] = propertyName
        self._properties[propertyName] = propertyValue
        self._attributes[propertyName] = attributes or PropertyAttributes()

        self.callbacks.process(self.PROPERTY_ADDED_SIGNAL, self, propertyName)

    def setPropertyIndex(self, propertyName, newIndex):
        assert self.hasProperty(propertyName)
        currentIndex = self._properties.keys().index(propertyName)
        inds = range(len(self._properties))
        inds.remove(currentIndex)
        inds.insert(newIndex, currentIndex)
        items = self._properties.items()
        self._properties = OrderedDict([items[i] for i in inds])

    def setProperty(self, propertyName, propertyValue):
        self.assertProperty(propertyName)
        propertyValue = fromQColor(propertyName, propertyValue)

        names = self.getPropertyAttribute(propertyName, 'enumNames')
        if names and type(propertyValue) != int:
            propertyValue = names.index(propertyValue)

        self.oldPropertyValue = (propertyName, self.getProperty(propertyName))
        self._properties[propertyName] = propertyValue
        self.oldPropertyValue = None
        self.callbacks.process(self.PROPERTY_CHANGED_SIGNAL, self, propertyName)

    def getPropertyAttribute(self, propertyName, propertyAttribute):
        self.assertProperty(propertyName)
        return getattr(self._attributes[propertyName], propertyAttribute)

    def setPropertyAttribute(self, propertyName, propertyAttribute, value):
        self.assertProperty(propertyName)
        attributes = self._attributes[propertyName]
        assert hasattr(attributes, propertyAttribute), "Missing attribute: {:s}".format(propertyAttribute)
        setattr(attributes, propertyAttribute, value)
        self.callbacks.process(self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL, self, propertyName, propertyAttribute)

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
        prop = panel.getProperty(propertyName)

        if prop is not None:

            propertyValue = properties.getProperty(propertyName)
            propertyValue = toQProperty(propertyName, propertyValue)

            if isinstance(propertyValue, list):
                for i, subValue in enumerate(propertyValue):
                    panel.getSubProperty(prop, i).setValue(subValue)

                groupName = PropertyPanelHelper.getPropertyGroupName(propertyName, propertyValue)
                prop.setPropertyName(groupName)

            else:
                prop.setValue(propertyValue)

    @staticmethod
    def setPropertyFromPanel(prop, propertiesPanel, propertySet):

        if prop.isSubProperty():
            if not propertiesPanel.getParentProperty(prop):
                return

            propertyIndex = propertiesPanel.getSubPropertyIndex(prop)
            propertyName = prop.propertyName()
            propertyName = propertyName[:propertyName.index('[')]

            propertyValue = propertySet.getProperty(propertyName)
            propertyValue = list(propertyValue)
            propertyValue[propertyIndex] = prop.value()

            propertySet.setProperty(propertyName, propertyValue)

            groupName = PropertyPanelHelper.getPropertyGroupName(propertyName, propertyValue)
            propertiesPanel.getParentProperty(prop).setPropertyName(groupName)

        else:

            propertyName = prop.propertyName()
            propertyValue = prop.value()
            propertyValue = fromQColor(propertyName, propertyValue)
            propertySet.setProperty(propertyName, propertyValue)


    @staticmethod
    def _setPropertyAttributes(prop, attributes):

        prop.setAttribute('decimals', attributes.decimals)
        prop.setAttribute('minimum', attributes.minimum)
        prop.setAttribute('maximum', attributes.maximum)
        prop.setAttribute('singleStep', attributes.singleStep)
        if attributes.enumNames:
            prop.setAttribute('enumNames', attributes.enumNames)

    @staticmethod
    def getPropertyGroupName(name, value):
        return '%s [%s]' % (name, ', '.join(['%.2f' % v if isinstance(v, float) else str(v) for v in value]))


    @staticmethod
    def _addProperty(panel, name, attributes, value):

        value = toQProperty(name, value)

        if isinstance(value, list):
            groupName = PropertyPanelHelper.getPropertyGroupName(name, value)
            groupProp = panel.addGroup(name, groupName)
            for v in value:
                p = panel.addSubProperty(name, v, groupProp)
                PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return groupProp
        elif attributes.enumNames:
            p = panel.addEnumProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            p.setValue(value)
            return p
        else:
            p = panel.addProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return p


class PropertyPanelConnector(object):
    def __init__(self, propertySet, propertiesPanel):
        self.propertySet = propertySet
        self.propertiesPanel = propertiesPanel
        self.propertySet.connectPropertyAdded(self._onPropertyAdded)
        self.propertySet.connectPropertyChanged(self._onPropertyChanged)
        self.propertySet.connectPropertyAttributeChanged(self._onPropertyAttributeChanged)
        self.propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', self._onPanelPropertyChanged)

        self._blockSignals = True
        PropertyPanelHelper.addPropertiesToPanel(self.propertySet, self.propertiesPanel)
        self._blockSignals = False

    def _onPropertyAdded(self, propertySet, propertyName):
        self._blockSignals = True
        self.propertiesPanel.clear()
        PropertyPanelHelper.addPropertiesToPanel(propertySet, self.propertiesPanel)
        self._blockSignals = False

    def _onPropertyAttributeChanged(self, propertySet, propertyName, propertyAttribute):
        self._blockSignals = True
        self.propertiesPanel.clear()
        PropertyPanelHelper.addPropertiesToPanel(propertySet, self.propertiesPanel)
        self._blockSignals = False

    def _onPropertyChanged(self, propertySet, propertyName):
        self._blockSignals = True
        PropertyPanelHelper.onPropertyValueChanged(self.propertiesPanel, propertySet, propertyName)
        self._blockSignals = False

    def _onPanelPropertyChanged(self, panelProperty):
        if not self._blockSignals:
            PropertyPanelHelper.setPropertyFromPanel(panelProperty, self.propertiesPanel, self.propertySet)
