__author__ = 'manuelli'
from gurobipy import *
from ddapp import gurobiutils as grbUtils
import numpy as np
import scipy.linalg


class ContactFilterGurobi(object):

    def __init__(self, numContactsList=None):
        if numContactsList is None:
            numContactsList = [1]

        self.initializeModels(numContactsList)


    def setVerboseFlag(self, val):
        for int, d in self.modelData.iteritems():
            d['model'].setParam('OutputFlag', val)


    def initializeModels(self, numContactsList):

        self.modelData = {}

        for numContacts in numContactsList:

            d = dict()
            alphaVars = {}
            m = Model(str(numContacts) + " contact(s)")
            m.setParam('OutputFlag', False)

            for i in xrange(0,numContacts):
                for j in xrange(0,4):
                    alphaVars[i,j] = m.addVar(lb=0.0, name="alpha_%i_%i" % (i,j))

            m.update()
            d['model'] = m
            d['alphaVars'] = alphaVars
            self.modelData[numContacts] = d


    # solves a single measurement update step for numContacts
    def solve(self, numContacts, residual, H_list, W):
        if numContacts not in self.modelData.keys():
            raise ValueError("A model of size %i isn't available" % numContacts)


        H = np.concatenate(H_list, axis=1)
        Q = np.dot(np.dot(H.transpose(),W),H)
        f = -2.0*np.dot(np.dot(residual.transpose(), W),H)
        constant = np.dot(np.dot(residual.transpose(), W), residual)


        model = self.modelData[numContacts]['model']
        alphaVars = self.modelData[numContacts]['alphaVars']
        varList = [alphaVars[i,j] for i in xrange(0,numContacts) for j in xrange(0,4)]

        grbUtils.clearObjective(model)
        grbUtils.addObjective(model, Q, f, varList, constant=constant)
        model.optimize()
        return self.parseModelSolution(numContacts)


    def parseModelSolution(self, numContacts):
        d = {}
        d['alphaVals'] = {}

        m = self.modelData[numContacts]['model']
        alphaVars = self.modelData[numContacts]['alphaVars']
        for i in xrange(0,numContacts):
                for j in xrange(0,4):
                    d['alphaVals'][i,j] = alphaVars[i,j].getAttr('X')

        d['objectiveValue'] = m.getAttr('ObjVal')
        return d


    def test(self):
        Q = np.eye(4)
        f = np.arange(1,5)

        model = self.modelData[1]['model']
        alphaVars = self.modelData[1]['alphaVars']

        varList = [alphaVars[1,i] for i in xrange(1,5)]

        grbUtils.addQuadraticObjective(model, Q, varList)
        grbUtils.addLinearObjective(model, f, varList)






