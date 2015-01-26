import pprint
import xml.dom.minidom
import sys
import re
import os
from collections import OrderedDict
from ddapp.thirdparty import NumericStringParser

from ddapp import roboturdf
from ddapp import propertyset
import ddapp.objectmodel as om


def getElementNodes(node):
      elementNodes = []
      if node.nodeType == xml.dom.Node.ELEMENT_NODE:
          elementNodes.append(node)
      for child in node.childNodes:
          elementNodes += getElementNodes(child)
      return elementNodes


class OtdfParser(object):

    def __init__(self):
        self.otdfFilename = ''
        self.otdfString = ''
        self.xmlDoc = None
        self.paramDict = OrderedDict()

    def setOtdfFilename(self, filename):
        self.otdfFilename = filename
        assert os.path.isfile(filename)
        otdfString = open(filename, 'r').read()
        self.setOtdfString(otdfString)

    def setOtdfString(self, otdfString):
        self.otdfString = otdfString
        self.xmlDoc = xml.dom.minidom.parseString(otdfString)
        self.parseParams()

    def parseParams(self):
        assert self.xmlDoc is not None
        self.paramDict = OrderedDict()
        paramElements = self.xmlDoc.getElementsByTagName('param')
        for param in paramElements:
            name = param.getAttribute('name')
            valueDefault = param.getAttribute('default_value')
            valueMin = param.getAttribute('min')
            valueMax = param.getAttribute('max')
            valueIncrement = param.getAttribute('inc')
            #print 'param:', name, '=', valueDefault
            self.paramDict[name] = (valueDefault, valueMin, valueMax, valueIncrement)


    def getParamValue(self, paramName):
        valueDefault, valueMin, valueMax, valueIncrement = self.paramDict[paramName]
        return valueDefault

    def parseAttributeExpression(self, matchObj):
        expression = matchObj.group(1)
        #print '\nexpression:', expression
        paramNames = reversed(sorted(self.paramDict.keys(), key=len))
        for paramName in paramNames:
            if paramName in expression:
                paramValue = self.getParamValue(paramName)
                #print '  subbing param:', paramName, '=', paramValue
                expression = expression.replace(paramName, str(float(paramValue)))

        #print 'after subs:', expression
        try:
            evaluated = NumericStringParser.NumericStringParser().eval(expression)
        except:
            print 'parser error:', expression
            return '0.0'
        #print 'evaluated:', evaluated
        return str(evaluated)


    def processAttribute(self, attrValue):
        result = re.sub('\$\{(.*?)\}', self.parseAttributeExpression, attrValue)
        return result


    def getUrdfFromOtdf(self):

        xmlDoc = xml.dom.minidom.parseString(self.otdfString)

        elementNodes = getElementNodes(xmlDoc)

        assert elementNodes[0].tagName == 'object'
        elementNodes[0].tagName = 'robot'

        for elementNode in elementNodes:
            #print 'element node:', elementNode.nodeName
            for attrName, attrValue in elementNode.attributes.items():
                #print '  ', attrName, '=', attrValue
                attrValue = self.processAttribute(attrValue)
                elementNode.setAttribute(attrName, attrValue)

        return xmlDoc.toxml()


    def getUpdatedOtdf(self):

        xmlDoc = xml.dom.minidom.parseString(self.otdfString)

        paramElements = xmlDoc.getElementsByTagName('param')
        for param in paramElements:
            paramValue = self.getParamValue(param.getAttribute('name'))
            param.setAttribute('default_value', str(float(paramValue)))

        return xmlDoc.toxml()

class OtdfModelItem(roboturdf.RobotModelItem):

    def __init__(self, filename):
        self.parser = OtdfParser()
        self.parser.setOtdfFilename(filename)
        model = self.createDefaultModel()
        assert model is not None
        roboturdf.RobotModelItem.__init__(self, model)

        self.setProperty('Name', os.path.basename(filename))
        self.setProperty('Textures', False)
        self.setPropertyAttribute('Textures', 'hidden', True)
        # add otdf properties

        basePropertyKeys = self.properties._properties.keys()
        otdfPropertyKeys = []
        for paramName, paramData in self.parser.paramDict.iteritems():
            otdfPropertyKeys.append(paramName)
            valueDefault, valueMin, valueMax, valueIncrement = paramData
            self.addProperty(paramName, float(valueDefault), om.PropertyAttributes(decimals=4, minimum=valueMin, maximum=valueMax, singleStep=valueIncrement, hidden=False))

        orderedProperties = OrderedDict()
        for name in otdfPropertyKeys + basePropertyKeys:
            orderedProperties[name] = self.properties._properties[name]
        self.properties._properties = orderedProperties

        self.updateModelPosition()

    def createDefaultModel(self):
        urdfString = self.parser.getUrdfFromOtdf()
        #print urdfString
        return roboturdf.loadRobotModelFromString(urdfString)

    def _onPropertyChanged(self, propertySet, propertyName):
        roboturdf.RobotModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in self.parser.paramDict:
            valueDefault, valueMin, valueMax, valueIncrement = self.parser.paramDict[propertyName]
            newValue = self.getProperty(propertyName)
            self.parser.paramDict[propertyName] = (newValue, valueMin, valueMax, valueIncrement)

            newModel = self.createDefaultModel()
            self.setModel(newModel)
            self.updateModelPosition()
            self._renderAllViews()

    def updateModelPosition(self):
        baseJoint = []
        jointNames = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for param in jointNames:
            baseJoint.append(self.getProperty(param) if self.hasProperty(param) else 0.0)
        self.model.setJointPositions(baseJoint, ['base_' + name for name in jointNames])


def openOtdf(filename, view):

    model = OtdfModelItem(filename)
    om.addToObjectModel(model)
    model.addToView(view)
    return model
