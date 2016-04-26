import director.tasks.robottasks as rt

taskLibrary = [

  ['utils', [
    [rt.PrintTask, {}],
    [rt.UserPromptTask, {}],
    [rt.DelayTask, {}],
    [rt.PauseTask, {}],
    [rt.QuitTask, {}],
  ]],

  ['perception sensors', [
    [rt.WaitForMultisenseLidar, {}],
    [rt.SnapshotMultisensePointcloud, {}],
    [rt.SnapshotSelectedPointcloud, {}],
    [rt.SnapshotStereoPointcloud, {}],
    [rt.FindHorizontalSurfaces, {}],
  ]],

  ['fitting', [
    [rt.UserAnnotatePointCloud, {}],
    [rt.UserSelectAffordanceCandidate, {}],
    [rt.ProjectAffordanceToGround, {}],
    [rt.FindHorizontalSurfaces, {}],
    [rt.FitWallFrameFromAnnotation, {}],
    [rt.FitShelfItem, {}],
    [rt.FindRotaryDrillByAnnotation, {}],
    [rt.ComputeRobotFootFrame, {}],
    [rt.TransformFrame, {}],
  ]],

  ['spawn affordances', [
    [rt.SpawnDrillBarrelAffordance, {}],
    [rt.SpawnDrillRotaryAffordance, {}],
    [rt.SpawnValveAffordance, {}],
  ]],

  ['planning', [
      [rt.RequestFootstepPlan, {}],
      [rt.RequestWalkingPlan, {}],
      [rt.PlanPostureGoal, {}],
      [rt.PlanReachToFrame, {}],
      [rt.PlanGazeTrajectory, {}],
      [rt.PlanStandPosture, {}],
      [rt.PlanNominalPosture, {}],
  ]],

  ['execution', [
      [rt.CommitManipulationPlan, {}],
      [rt.CommitFootstepPlan, {}],
      [rt.WaitForManipulationPlanExecution, {}],
      [rt.WaitForWalkExecution, {}],
      [rt.WaitForAtlasBehavior, {}],
  ]],

  ['hand control', [
      [rt.CloseHand, {}],
      [rt.OpenHand, {}],
  ]],

]
