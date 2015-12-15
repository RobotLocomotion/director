__author__ = 'manuelli'

from gurobipy import *
import numpy as np



def addQuadraticObjective(model, Q, varList):

    shape = np.shape(Q)
    numVars = len(varList)
    if shape != (numVars, numVars):
        raise ValueError("Q must be n x n, where n = len(varList)")

    # this takes care of the corner case where model.getObjective() is a linear expression
    quadExpr = QuadExpr(expr=model.getObjective())

    for idx_1, var_1 in enumerate(varList):
        for idx_2, var_2 in enumerate(varList):
            quadExpr.add(var_1*var_2, Q[idx_1,idx_2])


    model.setObjective(quadExpr)
    model.update()


def addLinearObjective(model, f, varList):

    shape = np.shape(f)
    if shape != (len(varList),):
        raise ValueError("f must be shape (n,) where n = len(varList)")


    quadExpr = model.getObjective()
    for idx, var in enumerate(varList):
        quadExpr.add(var, f[idx])

    model.setObjective(quadExpr)
    model.update()


def addObjective(model, Q, f, varList):
    addQuadraticObjective(model, Q, varList)
    addLinearObjective(model, f, varList)


def clearObjective(model):
    model.setObjective(QuadExpr())
    model.update()

