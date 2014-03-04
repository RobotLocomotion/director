import actions
from actions import *

#Create a list of all sequences
sequenceDict = {}

for name, value in actions.__dict__.iteritems():

    if hasattr(value, '__bases__') and Action in value.__bases__:
        if name == 'Goal' or name == 'Fail':
            continue
        sequenceDict[name] = [{name : [value, 'goal', 'fail', value.inputs]}, name.lower(), 'Primitive']


#Create new sequences below and add each to the list.
# Format for adding is: (name, sequence_definition_dictionary, start_state, 'Sequence')

simple_reach_seq    = {'wait_for_scan1': [WaitForScan,    'pose_search',   'fail',      {} ],
                       'pose_search'   : [PoseSearch,     'reach_plan',    'walk_plan', {'Affordance': 'USER', 'Hand': 'USER', 'Constraints': 'USER'} ],
                       'walk_plan'     : [WalkPlan,       'walk',          'fail',      {'WalkTarget': 'pose_search'} ],
                       'walk'          : [Walk,           'wait_for_scan2','fail',      {'WalkPlan': 'walk_plan'} ],
                       'wait_for_scan2': [WaitForScan,    'fit',           'fail',      {} ],
                       'fit'           : [Fit,            'reach_plan',    'fail',      {'Affordance': 'USER'} ],
                       'reach_plan'    : [ReachPlan,      'reach',         'fail',      {'RobotPose': 'pose_search', 'Hand': 'pose_search', 'Constraints': 'USER'} ],
                       'reach'         : [Reach,          'grip',          'fail',      {'JointPlan': 'reach_plan'} ],
                       'grip'          : [Grip,           'retract_plan1', 'fail',      {'Hand': 'pose_search'} ],
                       'retract_plan1' : [JointMovePlan,  'retract_move1', 'fail',      {'RobotPose': 'USER'} ],
                       'retract_move1' : [JointMove,      'goal',          'fail',      {'JointPlan': 'retract_plan1'} ]}

sequenceDict['SimpleReach'] = [simple_reach_seq, 'wait_for_scan', 'Sequence']


sequenceList = [[key]+sequenceDict[key] for key in sequenceDict.keys()]
