__author__ = 'manuelli'
from gurobipy import *
from director import gurobiutils as grbUtils
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

    def createLassoModel(self, numContacts):
        d = {}
        m = Model("LASSO " + str(numContacts) + " contact(s)")
        alphaVars = {}
        betaVars = {}
        for i in xrange(0,numContacts):
            # add betaVar
            betaVars[i] = m.addVar(name="beta_%i" % (i,))
            for j in xrange(0,4):
                alphaVars[i,j] = m.addVar(lb=0.0, name="alpha_%i_%i" % (i,j))

        # this update is critical, otherwise the addition of the constraints and stuff won't work
        m.update()

        for i in xrange(0,numContacts):
            for j in xrange(0,4):
                m.addConstr(betaVars[i] - alphaVars[i,j] >= 0, "beta_%i >= alpha_%i_%i" %(i,i,j))

        m.update()
        d['model'] = m
        d['numContacts'] = numContacts
        d['alphaVars'] = alphaVars
        d['betaVars'] = betaVars

        return d

    # solves a given lasso model given the residual
    def solveLasso(self, d, residual, H_list, W, lam):
        if not (d['numContacts'] == len(H_list)):
            raise ValueError("the model must have same number of contacts as things in H_list")

        H = np.concatenate(H_list, axis=1)
        Q = np.dot(np.dot(H.transpose(),W),H)
        f = -2.0*np.dot(np.dot(residual.transpose(), W),H)
        constant = np.dot(np.dot(residual.transpose(), W), residual)

        numContacts = d['numContacts']
        model = d['model']
        alphaVars = d['alphaVars']
        betaVars = d['betaVars']


        alphaVarList = [alphaVars[i,j] for i in xrange(0,numContacts) for j in xrange(0,4)]
        betaVarList = [betaVars[i] for i in xrange(0,numContacts)]
        lamVec = lam*np.ones(d['numContacts'])

        grbUtils.clearObjective(model)
        grbUtils.addObjective(model, Q, f, alphaVarList, constant=constant)
        grbUtils.addLinearObjective(model, lamVec, betaVarList)
        model.optimize()


    def test2(self):
        m = Model("test")
        d = {}
        H = np.array([1,1])
        H_matrix = np.reshape(H,(1,2))
        Q = np.dot(H_matrix.transpose(),H_matrix)
        f = -2.0*H
        # Q = np.eye(2)
        # f = -2*np.ones(2)
        d['alpha_1'] = m.addVar(lb=0.0, name="alpha_1")
        d['alpha_2'] = m.addVar(lb=0.0, name="alpha_2")

        d['beta'] = m.addVar(name="beta")

        # this update turns out to be critical!!!!
        m.update()

        m.addConstr(d['beta'] - d['alpha_1']>= 0, name="beta - alpha_1 > 0")
        m.addConstr(d['beta'] - d['alpha_2']>= 0, name="beta - alpha_2 > 0")

        m.update()
        grbUtils.addObjective(m, Q,f,[d['alpha_1'], d['alpha_2']],constant=1)
        m.optimize()

        d['model'] = m
        return d


