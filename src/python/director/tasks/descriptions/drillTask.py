from director.tasks import robottasks as rt


side = 'Left'
mirror = 1 if side == 'Right' else -1

drillTaskRight = [

    ['drill wall', [

        ['fit wall targets', [
          [rt.SnapshotMultisensePointcloud, {}],
          [rt.UserAnnotatePointCloud, {'Annotation name':'wall annotation', 'Number of points':3}],
          [rt.FitWallFrameFromAnnotation, {'Annotation input name':'wall annotation'}],
        ]],

        ['walk to wall', [
          [rt.ComputeRobotFootFrame, {}],
          [rt.ProjectAffordanceToGround, {'Affordance name':'wall', 'Ground frame name':'robot foot frame', 'Frame output name':'wall ground frame'}],
          [rt.TransformFrame, {'Frame input name':'wall ground frame', 'Frame output name':'wall stance frame', 'Translation':[-0.7, -0.3, 0.0]}],
          [rt.RequestFootstepPlan, {'Stance frame name':'wall stance frame'}],
          [rt.RequestWalkingPlan, {'Footstep plan name':'wall stance frame footstep plan'}],
          [rt.UserPromptTask, {'Message':'Commit footstep plan?'}],
          [rt.CommitFootstepPlan, {'Plan name':'wall stance frame footstep plan'}],
          [rt.WaitForWalkExecution, {}]
        ]],

        ['plan drill trajectory', [
          [rt.TransformFrame, {'Frame input name':'wall frame', 'Frame output name':'wall gaze frame', 'Translation':[0.0, 0.0, 0.0], 'Rotation':[0, 0, 90]}],
          [rt.PlanGazeTrajectory, {'Target frame name':'wall gaze frame', 'Annotation input name':'wall annotation'}],
        ]],

    ]],

    ['drill pickup', [

        ['fit drill on table', [
          [rt.SnapshotMultisensePointcloud, {}],
          [rt.UserAnnotatePointCloud, {'Annotation name':'rotary drill annotation', 'Number of points':1}],
          [rt.FindRotaryDrillByAnnotation, {}],
        ]],

        ['plan walk to drill', [
          [rt.ComputeRobotFootFrame, {}],
          [rt.ProjectAffordanceToGround, {'Affordance name':'drill', 'Ground frame name':'robot foot frame', 'Frame output name':'drill ground frame'}],
          [rt.TransformFrame, {'Frame input name':'drill ground frame', 'Frame output name':'drill stance frame', 'Translation':[-0.7, 0.25*mirror, 0.0]}],
          [rt.RequestFootstepPlan, {'Stance frame name':'drill stance frame'}],
          [rt.RequestWalkingPlan, {'Footstep plan name':'drill stance frame footstep plan'}],
          [rt.UserPromptTask, {'Message':'Commit footstep plan?'}],
          [rt.CommitFootstepPlan, {'Plan name':'drill stance frame footstep plan'}],
          [rt.WaitForWalkExecution, {}]
        ]],

        ['refit drill on table', [
          [rt.SnapshotMultisensePointcloud, {}],
          [rt.UserAnnotatePointCloud, {'Annotation name':'rotary drill annotation', 'Number of points':1}],
          [rt.FindRotaryDrillByAnnotation, {}],
        ]],

        ['pick up drill on table', [

          ['compute manipulation frames', [
            [rt.TransformFrame, {'Frame input name':'drill frame', 'Frame output name':'drill grasp frame', 'Translation':[-0.045, 0.0, 0.0275], 'Rotation':[0, -90*mirror, -90]}],
            [rt.TransformFrame, {'Frame input name':'drill grasp frame', 'Frame output name':'drill reach frame', 'Translation':[0.0, -0.17, 0.0]}],
            [rt.TransformFrame, {'Frame input name':'drill grasp frame', 'Frame output name':'drill lift frame', 'Translation':[0.1*mirror, -0.05, 0.0]}],
          ]],

          ['raise arm', [
            [rt.PlanPostureGoal, {'Posture group':'General', 'Posture name':'arm up pregrasp', 'Side':side}],
            [rt.UserPromptTask, {'Message':'Commit manipulation plan?'}],
            [rt.CommitManipulationPlan, {'Plan name':'arm up pregrasp posture plan'}],
            [rt.WaitForManipulationPlanExecution, {}],
          ]],

          ['pre reach', [
            [rt.PlanReachToFrame, {'Frame input name':'drill reach frame', 'Side':side}],
            [rt.UserPromptTask, {'Message':'Commit manipulation plan?'}],
            [rt.CommitManipulationPlan, {'Plan name':'drill reach frame reach plan'}],
            [rt.WaitForManipulationPlanExecution, {}],
          ]],

          ['reach', [
            [rt.PlanReachToFrame, {'Frame input name':'drill grasp frame', 'Side':side}],
            [rt.UserPromptTask, {'Message':'Commit manipulation plan?'}],
            [rt.CommitManipulationPlan, {'Plan name':'drill grasp frame reach plan'}],
            [rt.WaitForManipulationPlanExecution, {}],
          ]],

          ['close gripper', [
            [rt.UserPromptTask, {'Message':'Close gripper?'}],
            [rt.CloseHand, {'Side':side}],
            [rt.DelayTask, {'Delay time':2.0}],
          ]],

          ['lift', [
            [rt.PlanReachToFrame, {'Frame input name':'drill lift frame', 'Side':side}],
            [rt.UserPromptTask, {'Message':'Commit manipulation plan?'}],
            [rt.CommitManipulationPlan, {'Plan name':'drill lift frame reach plan'}],
            [rt.WaitForManipulationPlanExecution, {}],
          ]],

          ['extract', [
            [rt.PlanPostureGoal, {'Posture group':'General', 'Posture name':'arm up pregrasp', 'Side':side}],
            [rt.UserPromptTask, {'Message':'Commit manipulation plan?'}],
            [rt.CommitManipulationPlan, {'Plan name':'arm up pregrasp posture plan'}],
            [rt.WaitForManipulationPlanExecution, {}],
          ]],

        ]],

     ]],
]

