import json
from collections import OrderedDict

from director import plannerPublisher
from director import ikconstraintencoder
from director import lcmUtils

import drake as lcm_drake

class GenericDrakePlannerPublisher(plannerPublisher.PlannerPublisher):

  def setupMessage(self, fields):

    # todo, exotica should migrate to new style option names.
    # for backward compatibility, override the options field here
    # and insert jointLimits to options list
    options = OrderedDict(self.ikPlanner.getIkOptions()._properties)
    options['jointLimits'] = fields.jointLimits

    msg = lcm_drake.lcmt_generic_planner_request()
    msg.utime = fields.utime
    msg.poses = json.dumps(fields.poses)
    msg.constraints = ikconstraintencoder.encodeConstraints(fields.constraints)
    msg.seed_pose = fields.seedPose
    msg.nominal_pose = fields.nominalPose
    msg.end_pose = fields.endPose
    msg.joint_names = json.dumps(fields.jointNames)
    msg.affordances = fields.affordances
    msg.options = json.dumps(options)

    return msg


  def processIK(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName=""):


    fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName)
    msg = self.setupMessage(fields)

    # create listener
    responseChannel = 'CANDIDATE_MANIP_IKPLAN'
    responseMessageClass = lcm_drake.lcmt_plan
    listener = lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)
    
    #publish and listen
    lcmUtils.publish('IK_REQUEST', msg)
    ikplan = listener.waitForResponse(timeout=12000)
    listener.finish()

    if ikplan is None:
      raise Exception('Timeout: No plan returned')

    endPose = [0] * self.ikPlanner.jointController.numberOfJoints
    if ikplan.num_states>0:
      endPose = ikplan.plan[ikplan.num_states-1].joint_position
      info = ikplan.info[ikplan.num_states-1]
    else: 
      info = -1
    self.ikPlanner.ikServer.infoFunc(info)
    return endPose, info

  def processTraj(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):

    # Temporary fix / HACK / TODO (should be done in exotica_json)
    largestTspan = [0, 0]
    for constraintIndex, _ in enumerate(constraints):
      # Get tspan extend to normalise time-span
      if np.isfinite(constraints[constraintIndex].tspan[0]) and np.isfinite(constraints[constraintIndex].tspan[1]):
        largestTspan[0] = constraints[constraintIndex].tspan[0] if (constraints[constraintIndex].tspan[0] < largestTspan[0]) else largestTspan[0]
        largestTspan[1] = constraints[constraintIndex].tspan[1] if (constraints[constraintIndex].tspan[1] > largestTspan[1]) else largestTspan[1]

    # Temporary fix / HACK/ TODO to normalise time spans
    for constraintIndex, _ in enumerate(constraints):
      if np.isfinite(constraints[constraintIndex].tspan[0]) and np.isfinite(constraints[constraintIndex].tspan[1]):
        if largestTspan[1] != 0:
          constraints[constraintIndex].tspan[0] = constraints[constraintIndex].tspan[0] / largestTspan[1]
          constraints[constraintIndex].tspan[1] = constraints[constraintIndex].tspan[1] / largestTspan[1]
    
    fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName, endPoseName)
    msg = self.setupMessage(fields)

    # create listener
    responseChannel = 'CANDIDATE_MANIP_IKPLAN'
    responseMessageClass = lcm_drake.lcmt_plan
    listener = lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)

    if ikplan is None:
      raise Exception('Timeout: No plan returned')

    lcmUtils.publish('PLANNER_REQUEST', msg)
    lastManipPlan = listener.waitForResponse(timeout=20000)
    listener.finish()

    self.ikPlanner.ikServer.infoFunc(lastManipPlan.info[0])
    return lastManipPlan, lastManipPlan.info[0]