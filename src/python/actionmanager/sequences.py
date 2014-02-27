import actions
from actions import *

#Create a list of all sequences
sequenceList = []

for name, value in actions.__dict__.iteritems():

    if hasattr(value, '__bases__') and Action in value.__bases__:
        if name == 'Goal' or name == 'Fail':
            continue
        simpleDict = {name : [value, 'goal', 'fail', [None]]}
        sequenceList.append( [name, simpleDict, name.lower(), 'Primitive'] )


#Create new sequences below and add each to the list.
# Format for adding is: (name, sequence_definition_dictionary, start_state, 'Sequence')

simple_reach_seq    = {'walk_plan'     : [WalkPlan,       'walk',          'fail',      {'target':'grasp stance'} ],
                       'walk'          : [Walk,           'wait_for_scan', 'fail',      [None] ],
                       'wait_for_scan' : [WaitForScan,    'fit',           'fail',      [None] ],
                       'fit'           : [Fit,            'reach_plan',    'fail',      {'affordance':'drill'} ],
                       'reach_plan'    : [ReachPlan,      'reach',         'walk_plan', {'target':'grasp frame', 'hand':'left'} ],
                       'reach'         : [Reach,          'grip',          'fail',      [None] ],
                       'grip'          : [Grip,           'retract_plan1', 'fail',      [None] ],
                       'retract_plan1' : [JointMovePlan,  'retract_move1', 'fail',      {'group':'General','name':'shooter','side':'left'} ],
                       'retract_move1' : [JointMove,      'retract_plan2', 'fail',      [None] ] }

sequenceList.insert(0, ['SimpleReach', simple_reach_seq, 'wait_for_scan', 'Sequence'] )

pregrasp_reach_seq = {'walk_plan'     : [WalkPlan,       'walk',          'fail',      {'target':'grasp stance'} ],
                      'walk'          : [Walk,           'wait_for_scan', 'fail',      [None] ],
                      'wait_for_scan' : [WaitForScan,    'fit',           'fail',      [None] ],
                      'fit'           : [Fit,            'ready_plan',    'fail',      {'affordance':'drill'} ],
                      'ready_plan'    : [JointMovePlan,  'ready_move',    'fail',      {'group':'General','name':'shooter','side':'left'} ],
                      'ready_move'    : [JointMove,      'reach_plan',    'fail',      [None] ],
                      'reach_plan'    : [ReachPlan,      'reach',         'walk_plan', {'target':'grasp frame', 'hand':'left'} ],
                      'reach'         : [Reach,          'grip',          'fail',      [None] ],
                      'grip'          : [Grip,           'retract_plan1', 'fail',      [None] ],
                      'retract_plan1' : [JointMovePlan,  'retract_move1', 'fail',      {'group':'General','name':'shooter','side':'left'} ],
                      'retract_move1' : [JointMove,      'retract_plan2', 'fail',      [None] ],
                      'retract_plan2' : [JointMovePlan,  'retract_move2', 'fail',      {'group':'General','name':'handdown','side':'left'} ],
                      'retract_move2' : [JointMove,      'goal',          'fail',      [None] ] }


sequenceList.insert(0, ['PreGraspReach', pregrasp_reach_seq, 'wait_for_scan', 'Sequence'] )

