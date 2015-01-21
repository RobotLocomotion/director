function robot = weldFingerJoints(robot)

  finger_joints = {...
    'right_f0_j0'
    'right_f0_j1'
    'right_f0_j2'
    'right_f1_j0'
    'right_f1_j1'
    'right_f1_j2'
    'right_f2_j0'
    'right_f2_j1'
    'right_f2_j2'
    'right_f3_j0'
    'right_f3_j1'
    'right_f3_j2'
    'left_f0_j0'
    'left_f0_j1'
    'left_f0_j2'
    'left_f1_j0'
    'left_f1_j1'
    'left_f1_j2'
    'left_f2_j0'
    'left_f2_j1'
    'left_f2_j2'
    'left_f3_j0'
    'left_f3_j1'
    'left_f3_j2'}';

  for joint_name = finger_joints
    robot = robot.weldJoint(joint_name{:}, 1);
  end

end
