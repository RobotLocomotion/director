if(NOT BUILD_TESTING)
  return()
endif()

include(python-coverage/setup.cmake)

set(python_tests_core
  testAffordancePanel.py
  testCameraControl.py
  testConsoleApp.py
  testDepthScanner.py
  testFrameSync.py
  testMainWindowApp.py
  testObjectModel.py
  testPackagePath.py
  testPropertiesPanel.py
  testPythonConsole.py
  testTaskQueue.py
  testTransformations.py
)

set(python_tests_lcm
  testDrakeVisualizer.py
)

set(python_tests_robot
  testActionSequence.py
  testAffordanceItems.py
  testAtlasDriver.py
  testCameraView.py
  testContinuousWalking.py
  testDrawRobotLog.py
  testEndEffectorIk.py
  testImageView.py
  testLoadUrdf.py
  testOtdfParser.py
  testPlanConstraints.py
  testPlaneSegmentation.py
  testPyDrakeIk.py
  testRobotPoseGui.py
  testRobotSystem.py
  testTableFit.py
  testTableFitStereo.py
  testTeleopPanel.py
  testValveFit.py
  testValveFitStereo.py
  testAmazonPod.py
)

# Tests are run by default on the default robot
#
# If intending to test on a particular robot only, please set _extra_args to the
# director argument for that robot.
#
# Tests can also be run on multiple robots by setting an additional
# _extra_args_{n} to the director argument for the n-th robot to test, n >= 2.


# these tests use both Atlas v5 and Valkyrie
set(testTeleopPanel_extra_args_2 -v5)
set(testContinuousWalking_extra_args_2 -v5)

set(testEndEffectorIk_extra_args --director_config ${CMAKE_SOURCE_DIR}/../models/IRB140/director_config.json)
set(testEndEffectorIk_extra_args_2 -v5)

set(python_test_args
  --testing --data-dir ${CMAKE_SOURCE_DIR}/../../../drc-testing-data --output-dir ${CMAKE_BINARY_DIR}/Testing/Temporary
)

set(python_exe ${CMAKE_INSTALL_PREFIX}/bin/directorPython)

macro(add_python_test name label)
  get_filename_component(base_name ${name} NAME_WE)
  set(test_name test_${base_name})
  add_test(${test_name} ${python_exe} ${CMAKE_CURRENT_SOURCE_DIR}/${name} ${python_test_args} ${${base_name}_extra_args})
  set_property(TEST ${test_name} PROPERTY ENVIRONMENT ${python_coverage_environment_arg})
  set_property(TEST ${test_name} PROPERTY LABELS ${label})

  # Check whether additional test cases have been defined
  set(n 2)
  while(DEFINED ${base_name}_extra_args_${n})
    set(test_name test_${base_name}_${n})
    add_test(${test_name} ${python_exe} ${CMAKE_CURRENT_SOURCE_DIR}/${name} ${python_test_args} ${${base_name}_extra_args_${n}})
    set_property(TEST ${test_name} PROPERTY ENVIRONMENT ${python_coverage_environment_arg})
    set_property(TEST ${test_name} PROPERTY LABELS ${label})
    math(EXPR n "${n}+1")
  endwhile()
endmacro()


foreach(name ${python_tests_core})
  add_python_test(${name} core)
endforeach()

if(USE_LCM)
  foreach(name ${python_tests_lcm})
    add_python_test(${name} lcm)
  endforeach()
endif()

if(USE_DRC)
  foreach(name ${python_tests_robot})
    add_python_test(${name} robot)
  endforeach()
endif()
