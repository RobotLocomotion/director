__author__ = 'manuelli'
from ddapp import contactfilter
from contactfilter import ContactFilter


__author__ = 'manuelli'
import ddapp
from ddapp import roboturdf
import numpy as np
import vtkAll as vtk
import PythonQt
import matplotlib.pyplot as plt
import Queue
import collections


import contactfilterutils as cfUtils


from PythonQt import QtCore, QtGui
from ddapp import transformUtils
from ddapp import lcmUtils

from ddapp.debugVis import DebugData
from ddapp import visualization as vis

from ddapp import objectmodel as om

import drc as lcmdrc


class ContactFilterVisualizer(object):

    def __init__(self, robotSystem, robotStateModel):
        self.robotStateModel = robotStateModel
        self.robotSystem = robotSystem
        self.addSubscribers()
        self.visualize = True

    def start(self):
        self.visualize = True

    def stop(self):
        self.visualize = False

    def addSubscribers(self):
        lcmUtils.addSubscriber("CONTACT_PARTICLE_FILTER_DATA", lcmdrc.CPF_data_t, self.onContactFilterMsg)

    def getCurrentPose(self):
        return self.robotSystem.robotStateJointController.q

    def drawParticleSet(self, particleSet, name="particle set", color=None, drawMostLikely=True,
                        drawHistoricalMostLikely=True):

        # set the color if it was passed in
        defaultColor = [0.5,0,0.5]
        mostLikelyColor = [1,0.4,0.7] # hot pink
        historicalMostLikelyColor = [1,0,0]

        if color is not None:
            defaultColor = color

        numParticlesAtCFP = {}
        numTotalParticles = len(particleSet.particleList)

        for particle in particleSet.particleList:
            cfp = particle.cfp
            if numParticlesAtCFP.has_key(cfp):
                numParticlesAtCFP[cfp] += 1
            else:
                numParticlesAtCFP[cfp] = 1

        # now we need to draw this
        plungerMaxLength = 0.4
        plungerMinLength = 0.02

        d = DebugData()
        q = self.getCurrentPose()
        for cfp, numParticles in numParticlesAtCFP.iteritems():
            color = defaultColor

            # if particleSet.mostLikelyParticle is not None:
            #     if cfp == particleSet.mostLikelyParticle.cfp:
            #         color = mostLikelyColor

            rayLength = plungerMinLength + 1.0*numParticles/numTotalParticles*plungerMaxLength
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, cfp.contactNormal, rayLength, color)
            # forceDirectionWorldFrame, forceLocationWorldFrame =\
            #     cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
            #                                                             cfp.linkName,
            #                                                             cfp.contactLocation,
            #                                                             cfp.contactNormal)
            #
            # rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
            # d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
            # d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)

        if drawHistoricalMostLikely and (particleSet.historicalMostLikely['particle'] is not None):
            particle = particleSet.historicalMostLikely['particle']
            cfp = particle.cfp
            color = historicalMostLikelyColor
            rayLength = 0.3
            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        if drawMostLikely and (particleSet.mostLikelyParticle is not None):
            particle = particleSet.mostLikelyParticle
            cfp = particle.cfp
            color = mostLikelyColor
            rayLength = 0.4

            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)

            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        vis.updatePolyData(d.getPolyData(), name, colorByName='RGB255')

    def addPlungerToDebugData(self, d, linkName, contactLocation, contactDirection, rayLength, color):
        q = self.getCurrentPose()
        forceDirectionWorldFrame, forceLocationWorldFrame =\
                cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
                                                                        linkName,
                                                                        contactLocation,
                                                                        contactDirection)

        rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
        d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
        d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)


    def drawParticleSetList(self, particleSetList, drawMostLikely=True, drawHistoricalMostLikely=True):


        numParticleSets = len(particleSetList)
        maxNumParticleSets = 4
        for i in xrange(0, maxNumParticleSets):
            name = "particle set " + str(i+1)
            om.removeFromObjectModel(om.findObjectByName(name))

        for i, particleSet in enumerate(particleSetList):
            name = "particle set " + str(i+1)

            if i < numParticleSets:
                self.drawParticleSet(particleSet, name=name, color=particleSet.color,
                                     drawMostLikely=drawMostLikely, drawHistoricalMostLikely=drawHistoricalMostLikely)

    def onContactFilterMsg(self, msg):
        particleSetList = ContactFilter.decodeCPFData(msg)

        self.drawParticleSetList(particleSetList)