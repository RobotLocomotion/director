import actions
from actions import *
from collections import OrderedDict

#Create a list of all sequences
sequenceDict = OrderedDict([])

for name, value in actions.__dict__.iteritems():

    if hasattr(value, '__bases__') and Action in value.__bases__:
        if name == 'Goal' or name == 'Fail':
            continue
        argDict = {}
        for arg in value.inputs:
            argDict[arg] = 'USER'
        sequenceDict[name] = [{name : [value, 'goal', 'fail', argDict]}, name, 'Primitive']


#Create new sequences below and add each to the list.
# Format for adding is: (name, sequence_definition_dictionary, start_state, 'Sequence')

simple_reach_seq    = {'wait_for_scan1': [WaitForScan,    'pose_search',   'fail',      {} ],
                       'pose_search'   : [PoseSearch,     'reach_plan',    'walk_plan', {'Affordance': 'USER', 'Hand': 'USER', 'Constraints': 'USER'} ],
                       'walk_plan'     : [WalkPlan,       'walk',          'fail',      {'WalkTarget': 'pose_search'} ],
                       'walk'          : [Walk,           'wait_for_scan2','fail',      {'WalkPlan': 'walk_plan'} ],
                       'wait_for_scan2': [WaitForScan,    'fit',           'fail',      {} ],
                       'fit'           : [Fit,            'reach_plan',    'fail',      {'Affordance': 'USER'} ],
                       'reach_plan'    : [ReachPlan,      'reach',         'fail',      {'TargetPose': 'pose_search', 'Hand': 'pose_search', 'Constraints': 'USER'} ],
                       'reach'         : [Reach,          'grip',          'fail',      {'JointPlan': 'reach_plan'} ],
                       'grip'          : [Grip,           'retract_plan1', 'fail',      {'Hand': 'pose_search'} ],
                       'retract_plan1' : [JointMovePlan,  'retract_move1', 'fail',      {'RobotPose': 'USER'} ],
                       'retract_move1' : [JointMove,      'goal',          'fail',      {'JointPlan': 'retract_plan1'} ]}

#sequenceDict['PlannedReach'] = [simple_reach_seq, 'wait_for_scan1', 'Sequence']

simple_reach_seq = {'walk_plan'    : [WalkPlan,      'walk',          'fail',      {'WalkTarget': 'USER'} ],
                    'walk'         : [Walk,          'wait_for_scan', 'fail',      {'WalkPlan': 'walk_plan'} ],
                    'wait_for_scan': [WaitForScan,   'fit',           'fail',      {} ],
                    'fit'          : [Fit,           'reach_plan',    'fail',      {'Affordance': 'USER'} ],
                    'reach_plan'   : [ReachPlan,     'reach',         'walk_plan', {'TargetFrame': 'USER', 'Hand': 'USER', 'Constraints': 'USER'} ],
                    'reach'        : [Reach,         'grip',          'fail',      {'JointPlan': 'reach_plan'} ],
                    'grip'         : [Grip,          'retract_plan',  'fail',      {'Hand': 'pose_search'} ],
                    'retract_plan' : [JointMovePlan, 'retract_move',  'fail',      {'RobotPose': '1 walking with hose', 'Group': 'hose', 'Hand': 'USER'} ],
                    'retract_move' : [JointMove,     'goal',          'fail',      {'JointPlan': 'retract_plan'} ]}

sequenceDict['SimpleReach'] = [simple_reach_seq, 'wait_for_scan', 'Sequence']


drill_reach_seq = {'walk_plan'    : [WalkPlan,      'walk',          'fail',      {'WalkTarget': 'grasp stance right'} ],
                   'walk'         : [Walk,          'wait_for_scan', 'fail',      {'WalkPlan': 'walk_plan'} ],
                   'wait_for_scan': [WaitForScan,   'fit',           'fail',      {} ],
                   'fit'          : [Fit,           'reach_plan',    'fail',      {'Affordance': 'drill'} ],
                   'reach_plan'   : [ReachPlan,     'reach',         'walk_plan', {'TargetFrame': 'grasp frame', 'Hand': 'right', 'Constraints': 'none'} ],
                   'reach'        : [Reach,         'grip',          'fail',      {'JointPlan': 'reach_plan'} ],
                   'grip'         : [Grip,          'retract_plan',  'fail',      {'Hand': 'pose_search'} ],
                   'retract_plan' : [JointMovePlan, 'retract_move',  'fail',      {'RobotPose': '1 walking with hose', 'Group': 'hose', 'Hand': 'right'} ],
                   'retract_move' : [JointMove,     'goal',          'fail',      {'JointPlan': 'retract_plan'} ]}

sequenceDict['DrillReachRight'] = [drill_reach_seq, 'wait_for_scan', 'Sequence']

drill_reach_seq = {'walk_plan'    : [WalkPlan,      'walk',          'fail',      {'WalkTarget': 'grasp stance left'} ],
                   'walk'         : [Walk,          'wait_for_scan', 'fail',      {'WalkPlan': 'walk_plan'} ],
                   'wait_for_scan': [WaitForScan,   'fit',           'fail',      {} ],
                   'fit'          : [Fit,           'reach_plan',    'fail',      {'Affordance': 'drill'} ],
                   'reach_plan'   : [ReachPlan,     'reach',         'walk_plan', {'TargetFrame': 'grasp frame', 'Hand': 'left', 'Constraints': 'none'} ],
                   'reach'        : [Reach,         'grip',          'fail',      {'JointPlan': 'reach_plan'} ],
                   'grip'         : [Grip,          'retract_plan',  'fail',      {'Hand': 'pose_search'} ],
                   'retract_plan' : [JointMovePlan, 'retract_move',  'fail',      {'RobotPose': '1 walking with hose', 'Group': 'hose', 'Hand': 'left'} ],
                   'retract_move' : [JointMove,     'goal',          'fail',      {'JointPlan': 'retract_plan'} ]}

sequenceDict['DrillReachLeft'] = [drill_reach_seq, 'wait_for_scan', 'Sequence']


#Make a list out of the sequence ditionary, don't touch this line, just add to the dictionary
sequenceList = [[key]+sequenceDict[key] for key in sequenceDict.keys()]
